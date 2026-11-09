/*
 * This file is part of DeskHop (https://github.com/hrvach/deskhop).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * See the file LICENSE for the full license text.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define APPLE_VID                  0x05AC
#define APPLE_PID_MAGICTRACKPAD2   0x0265 /* Lightning */
#define APPLE_PID_MAGICTRACKPAD2_C 0x0324 /* USB-C */

/* Tuning. Trackpad units are roughly 47 per mm. */
#define TRACKPAD_POINTER_DIV    3   /* Trackpad units per pointer count, before speed/acceleration */
#define TRACKPAD_SCROLL_DIV     64  /* Trackpad units per wheel/pan step */
#define TRACKPAD_SETTLE_FRAMES  2   /* Frames of motion ignored after the finger count changes */
#define TRACKPAD_MAX_CONTACTS   16

/* config.trackpad_scroll_dir values (set from the config page) */
#define TRACKPAD_SCROLL_NATURAL     0 /* Content follows the fingers */
#define TRACKPAD_SCROLL_TRADITIONAL 1 /* Like a mouse wheel; the host OS setting then decides */

/* Click stabilisation. Pressing a finger down to click (and lifting off again)
   shifts its reported position a little, which would otherwise move the pointer
   or turn the click into a drag. */
#define TRACKPAD_PRESSURE_FREEZE 5   /* Per-frame pressure change that freezes pointer motion (normal sliding stays within ~3) */
#define TRACKPAD_DRAG_THRESHOLD  90  /* Finger travel (units, ~2 mm) needed while clicked before it becomes a drag */
#define TRACKPAD_RELEASE_FRAMES  3   /* Frames of motion ignored after a click is released */

bool trackpad_is_magic_trackpad(uint8_t dev_addr);
void trackpad_mount(uint8_t dev_addr, uint8_t instance, uint8_t itf_protocol, uint16_t desc_len);
void trackpad_umount(uint8_t dev_addr, uint8_t instance);
void trackpad_retry_mode_switch(void);

/* Returns true if the report was consumed and must not go through normal processing. */
bool trackpad_report(uint8_t dev_addr, uint8_t instance, uint8_t const *report, uint16_t len);

#ifdef DH_TRACKPAD_PROBE
/* Requires main.h (device_t) to be included first. */
void trackpad_probe_init(void);
void trackpad_probe_task(device_t *state);
void trackpad_probe_log_hid_mount(uint8_t dev_addr, uint8_t instance, uint8_t itf_protocol, uint16_t desc_len);
#endif
