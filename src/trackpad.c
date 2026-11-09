/*
 * This file is part of DeskHop (https://github.com/hrvach/deskhop).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * See the file LICENSE for the full license text.
 */

/*
 * Apple Magic Trackpad 2 multitouch support.
 *
 * Out of the box the trackpad acts as a plain boot mouse (pointer + click, no
 * scrolling). Sending SET_REPORT(Feature, ID 0x02) {0x02, 0x01} to its mouse
 * interface switches it to multitouch mode (same as Linux hid-magicmouse).
 * It then sends report ID 0x02 frames of 12 header bytes + 9 bytes per finger,
 * which we turn into ordinary mouse values:
 *
 *   1 finger            -> pointer movement
 *   2 fingers           -> vertical / horizontal scroll
 *   click               -> left button
 *   click with 2+ down  -> right button
 *
 * Verified on the USB-C model (05ac:0324). If the mode switch never takes
 * effect, plain 8-byte mouse reports keep going through normal processing.
 *
 * DH_TRACKPAD_PROBE (debug builds only) additionally logs USB mounts and raw
 * trackpad frames over CDC. USB host callbacks run on core1 while CDC runs on
 * core0, so log lines are queued and written by trackpad_probe_task on core0.
 */

#include "main.h"
#include "trackpad.h"

#define MT_REPORT_ID      0x02
#define MT_HEADER_LEN     12
#define MT_TOUCH_LEN      9
#define MT_CLICK_OFFSET   1

/* Contact state lives in the top bits of touch byte 3. 0x80/0x90 = finger
   down and tracking; 0x20-0x70 = landing; 0xC0 and up = lifting. */
#define MT_STATE_MASK     0xC0
#define MT_STATE_CONTACT  0x80

typedef struct {
    uint8_t id;
    uint8_t pressure;
    int16_t x;
    int16_t y;
} contact_t;

typedef struct {
    uint8_t dev_addr;
    uint8_t instance;
    bool switch_pending;
    bool switch_sent;

    contact_t prev[TRACKPAD_MAX_CONTACTS];
    uint8_t prev_count;
    uint8_t settle;

    int32_t acc_x, acc_y, acc_wheel, acc_pan;
    uint8_t buttons;
    bool clicked;

    /* Finger travel since the click started, and whether it became a drag. */
    int32_t click_travel_x, click_travel_y;
    bool dragging;
} trackpad_state_t;

static trackpad_state_t tp = {0};

/* Must stay valid until the control transfer completes. */
static uint8_t mt_mode_feature[] = {MT_REPORT_ID, 0x01};

/* ================================================== *
 * Probe logging (debug only)
 * ================================================== */
#ifdef DH_TRACKPAD_PROBE

#define PROBE_MAX_LINES_PER_SEC 40
#define PROBE_HEX_BYTES         64
#define PROBE_LINE_LEN          240
#define PROBE_QUEUE_LEN         48

typedef struct {
    char text[PROBE_LINE_LEN];
} probe_line_t;

static queue_t probe_queue;
static uint32_t queue_overflows;
static uint64_t window_start_us;
static uint32_t lines_in_window;
static uint32_t dropped_lines;

static void probe_log(const char *format, ...) {
    probe_line_t line;
    va_list args;

    va_start(args, format);
    vsnprintf(line.text, sizeof(line.text), format, args);
    va_end(args);

    if (!queue_try_add(&probe_queue, &line))
        queue_overflows++;
}

static bool rate_ok(void) {
    uint64_t now = time_us_64();

    if (now - window_start_us > 1000000) {
        if (dropped_lines)
            probe_log("TP rate-limited %lu lines\r\n", (unsigned long)dropped_lines);
        window_start_us = now;
        lines_in_window = 0;
        dropped_lines   = 0;
    }

    if (lines_in_window >= PROBE_MAX_LINES_PER_SEC) {
        dropped_lines++;
        return false;
    }

    lines_in_window++;
    return true;
}

static void probe_log_report(uint8_t instance, uint8_t const *report, uint16_t len) {
    if (!rate_ok())
        return;

    char hex[PROBE_HEX_BYTES * 3 + 1];
    int n = len < PROBE_HEX_BYTES ? len : PROBE_HEX_BYTES;

    for (int i = 0; i < n; i++)
        snprintf(hex + i * 3, 4, "%02x ", report[i]);
    hex[n * 3] = '\0';

    probe_log("TP %lu i%u len=%u: %s\r\n", (unsigned long)(time_us_64() / 1000), instance, len, hex);
}

void tuh_hid_set_report_complete_cb(uint8_t dev_addr, uint8_t idx, uint8_t report_id, uint8_t report_type, uint16_t len) {
    probe_log("TP set_report complete dev=%u inst=%u id=%u type=%u len=%u\r\n",
              dev_addr, idx, report_id, report_type, len);
}

/* Device-level enumeration callbacks (any device, not just HID). */
void tuh_mount_cb(uint8_t dev_addr) {
    uint16_t vid = 0, pid = 0;
    tuh_vid_pid_get(dev_addr, &vid, &pid);
    probe_log("DEV mount addr=%u vid=%04x pid=%04x\r\n", dev_addr, vid, pid);
}

void tuh_umount_cb(uint8_t dev_addr) {
    probe_log("DEV umount addr=%u\r\n", dev_addr);
}

void trackpad_probe_log_hid_mount(uint8_t dev_addr, uint8_t instance, uint8_t itf_protocol, uint16_t desc_len) {
    uint16_t vid = 0, pid = 0;
    tuh_vid_pid_get(dev_addr, &vid, &pid);
    probe_log("HID mount addr=%u inst=%u proto=%u desc_len=%u vid=%04x pid=%04x\r\n",
              dev_addr, instance, itf_protocol, desc_len, vid, pid);
}

void trackpad_probe_init(void) {
    queue_init(&probe_queue, sizeof(probe_line_t), PROBE_QUEUE_LEN);
}

/* Core0: drain queued lines to CDC and emit a heartbeat identifying the board. */
void trackpad_probe_task(device_t *state) {
    static uint64_t next_heartbeat_us;
    probe_line_t line;

    for (int i = 0; i < 4 && queue_try_remove(&probe_queue, &line); i++)
        dh_debug_printf("%s", line.text);

    uint64_t now = time_us_64();
    if (now >= next_heartbeat_us) {
        next_heartbeat_us = now + 2000000;
        dh_debug_printf("TP alive board=%c mouse_connected=%u queue_overflows=%lu\r\n",
                        BOARD_ROLE == OUTPUT_A ? 'A' : 'B', state->mouse_connected,
                        (unsigned long)queue_overflows);
    }
}

#define PROBE_LOG(...) probe_log(__VA_ARGS__)
#else
#define PROBE_LOG(...) do {} while (0)
#endif /* DH_TRACKPAD_PROBE */

/* ================================================== *
 * Mode switch
 * ================================================== */

void trackpad_retry_mode_switch(void) {
    if (!tp.switch_pending)
        return;

    /* Fails if the control pipe is busy (e.g. set_protocol in flight); retried
       from the set_protocol callback and on every report until it goes out. */
    if (tuh_hid_set_report(tp.dev_addr, tp.instance, MT_REPORT_ID, HID_REPORT_TYPE_FEATURE,
                           mt_mode_feature, sizeof(mt_mode_feature))) {
        tp.switch_pending = false;
        tp.switch_sent    = true;
        PROBE_LOG("TP mode switch sent dev=%u inst=%u\r\n", tp.dev_addr, tp.instance);
    }
}

bool trackpad_is_magic_trackpad(uint8_t dev_addr) {
    uint16_t vid, pid;

    if (!tuh_vid_pid_get(dev_addr, &vid, &pid))
        return false;

    return vid == APPLE_VID && (pid == APPLE_PID_MAGICTRACKPAD2 || pid == APPLE_PID_MAGICTRACKPAD2_C);
}

void trackpad_mount(uint8_t dev_addr, uint8_t instance, uint8_t itf_protocol, uint16_t desc_len) {
    PROBE_LOG("TP mount dev=%u inst=%u proto=%u desc_len=%u\r\n", dev_addr, instance, itf_protocol, desc_len);
    (void)desc_len;

    /* Touch frames come in on the boot mouse interface. */
    if (itf_protocol != HID_ITF_PROTOCOL_MOUSE)
        return;

    memset(&tp, 0, sizeof(tp));
    tp.dev_addr       = dev_addr;
    tp.instance       = instance;
    tp.switch_pending = true;
    trackpad_retry_mode_switch();
}

void trackpad_umount(uint8_t dev_addr, uint8_t instance) {
    PROBE_LOG("TP umount dev=%u inst=%u\r\n", dev_addr, instance);

    if (dev_addr == tp.dev_addr && instance == tp.instance)
        memset(&tp, 0, sizeof(tp));
}

/* ================================================== *
 * Touch frame decoding and gestures
 * ================================================== */

/* Sign-extend a 13-bit value. */
static inline int16_t sign13(uint32_t v) {
    return (int16_t)((v & 0x1000) ? (int32_t)v - 0x2000 : (int32_t)v);
}

/* Decode the fingers that are down. X grows to the right, Y grows towards the user. */
static uint8_t decode_contacts(uint8_t const *report, uint16_t len, contact_t *out) {
    uint8_t count  = 0;
    int touches    = (len - MT_HEADER_LEN) / MT_TOUCH_LEN;

    for (int i = 0; i < touches && count < TRACKPAD_MAX_CONTACTS; i++) {
        uint8_t const *d = report + MT_HEADER_LEN + i * MT_TOUCH_LEN;

        if ((d[3] & MT_STATE_MASK) != MT_STATE_CONTACT)
            continue;

        out[count].id       = d[8] & 0x0F;
        out[count].pressure = d[7];
        out[count].x  = sign13(((uint32_t)(d[1] & 0x1F) << 8) | d[0]);
        out[count].y  = -sign13(((uint32_t)(d[3] & 0x03) << 11) | ((uint32_t)d[2] << 3) | (d[1] >> 5));
        count++;
    }

    return count;
}

/* Average movement of fingers present in both this frame and the last one,
   and the largest pressure change among them. */
static void average_delta(contact_t const *cur, uint8_t count, int32_t *dx, int32_t *dy, int32_t *max_dp) {
    int32_t sum_x = 0, sum_y = 0, matched = 0;

    *max_dp = 0;

    for (int i = 0; i < count; i++) {
        for (int j = 0; j < tp.prev_count; j++) {
            if (tp.prev[j].id != cur[i].id)
                continue;

            int32_t dp = (int32_t)cur[i].pressure - tp.prev[j].pressure;
            if (dp < 0)
                dp = -dp;
            if (dp > *max_dp)
                *max_dp = dp;

            sum_x += cur[i].x - tp.prev[j].x;
            sum_y += cur[i].y - tp.prev[j].y;
            matched++;
            break;
        }
    }

    *dx = matched ? sum_x / matched : 0;
    *dy = matched ? sum_y / matched : 0;
}

/* Hold the pointer still while a click is pressed until the finger has clearly
   moved, so pressing (which shifts the finger slightly) doesn't drag. */
static void apply_click_dead_zone(int32_t *dx, int32_t *dy) {
    if (tp.dragging)
        return;

    tp.click_travel_x += *dx;
    tp.click_travel_y += *dy;

    int64_t travel_sq = (int64_t)tp.click_travel_x * tp.click_travel_x
                      + (int64_t)tp.click_travel_y * tp.click_travel_y;

    /* Start the drag from here; the dead-zone travel itself is never applied. */
    if (travel_sq > (int64_t)TRACKPAD_DRAG_THRESHOLD * TRACKPAD_DRAG_THRESHOLD)
        tp.dragging = true;

    *dx = *dy = 0;
}

/* Take whole steps out of an accumulator, keeping the remainder for later. */
static int32_t take_steps(int32_t *acc, int32_t div, int32_t limit) {
    int32_t steps = *acc / div;

    if (steps > limit)
        steps = limit;
    else if (steps < -limit)
        steps = -limit;

    *acc -= steps * div;
    return steps;
}

static void process_frame(uint8_t const *report, uint16_t len) {
    contact_t cur[TRACKPAD_MAX_CONTACTS];
    uint8_t count = decode_contacts(report, len, cur);
    bool click    = report[MT_CLICK_OFFSET] & 0x01;
    mouse_values_t values = {0};
    int32_t dx, dy, max_dp;

    average_delta(cur, count, &dx, &dy, &max_dp);

    /* Finger count changed: drop leftovers and let positions settle so lifting
       or adding a finger doesn't jerk the pointer or scroll. */
    if (count != tp.prev_count) {
        tp.settle    = TRACKPAD_SETTLE_FRAMES;
        tp.acc_wheel = tp.acc_pan = 0;
        if (count == 0)
            tp.acc_x = tp.acc_y = 0;
    }

    if (tp.settle) {
        tp.settle--;
        dx = dy = 0;
    }

    /* Choose the button when the click starts and hold it until release. */
    if (click && !tp.clicked) {
        tp.buttons        = (count >= 2) ? MOUSE_BUTTON_RIGHT : MOUSE_BUTTON_LEFT;
        tp.dragging       = false;
        tp.click_travel_x = tp.click_travel_y = 0;
        tp.acc_x = tp.acc_y = 0;
    } else if (!click && tp.clicked) {
        tp.buttons = 0;
        tp.settle  = TRACKPAD_RELEASE_FRAMES;
        tp.acc_x = tp.acc_y = 0;
        dx = dy = 0;
    }
    tp.clicked = click;

    if (count == 1 || (tp.clicked && count > 0)) {
        /* Pressure ramping up or down means a press or release is under way,
           not intentional movement. */
        if (max_dp >= TRACKPAD_PRESSURE_FREEZE)
            dx = dy = 0;

        if (tp.clicked)
            apply_click_dead_zone(&dx, &dy);

        /* Pointer, including click-and-drag with any number of fingers. */
        tp.acc_x += dx;
        tp.acc_y += dy;
        values.move_x = take_steps(&tp.acc_x, TRACKPAD_POINTER_DIV, INT16_MAX);
        values.move_y = take_steps(&tp.acc_y, TRACKPAD_POINTER_DIV, INT16_MAX);
    } else if (count == 2) {
        /* Natural: content follows the fingers. Wheel > 0 scrolls up, pan > 0 right. */
        int32_t sign = (global_state.config.trackpad_scroll_dir == TRACKPAD_SCROLL_TRADITIONAL) ? -1 : 1;
        tp.acc_wheel += sign * dy;
        tp.acc_pan   -= sign * dx;
        values.wheel = take_steps(&tp.acc_wheel, TRACKPAD_SCROLL_DIV, INT8_MAX);
        values.pan   = take_steps(&tp.acc_pan, TRACKPAD_SCROLL_DIV, INT8_MAX);
    }

    values.buttons = tp.buttons;

    memcpy(tp.prev, cur, count * sizeof(contact_t));
    tp.prev_count = count;

    /* No-ops (nothing moved, buttons unchanged) are dropped inside. */
    process_mouse_values(&values);
}

static bool is_touch_frame(uint8_t const *report, uint16_t len) {
    return len >= MT_HEADER_LEN
        && report[0] == MT_REPORT_ID
        && (len - MT_HEADER_LEN) % MT_TOUCH_LEN == 0;
}

bool trackpad_report(uint8_t dev_addr, uint8_t instance, uint8_t const *report, uint16_t len) {
#ifdef DH_TRACKPAD_PROBE
    probe_log_report(instance, report, len);
#endif

    if (dev_addr != tp.dev_addr || instance != tp.instance)
        return false;

    trackpad_retry_mode_switch();

    /* Before the switch takes effect we get plain 8-byte mouse reports; let
       normal processing handle those so the trackpad still works. */
    if (len == 8)
        return false;

    /* Anything else on this interface is ours. Frames with more fingers than
       fit in one 64-byte packet arrive split and fail this check; drop them. */
    if (is_touch_frame(report, len))
        process_frame(report, len);

    return true;
}
