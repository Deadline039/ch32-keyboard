/**
 * Copyright (c) 2026, Deadline039
 *
 * SPDX-License-Identifier: MIT-License
 */

#ifndef __USB_HID_KB_H
#define __USB_HID_KB_H

#include <usbd_core.h>
#include <usb_hid.h>

#define USBD_VID           0xFFFF
#define USBD_PID           0xFFFF
#define USBD_MAX_POWER     100
#define USBD_LANGID_STRING 1033

#define HID_INT_EP          0x81
#define HID_INT_EP_SIZE     8
#define HID_INT_EP_INTERVAL 10

#define USB_CONFIG_SIZE               34
#define HID_KEYBOARD_REPORT_DESC_SIZE 63

void usb_hid_kbd_init(uint8_t busid, uintptr_t reg_base);
void usb_hid_kbd_test(uint8_t busid);

#endif /* __USB_HID_KB_H */
