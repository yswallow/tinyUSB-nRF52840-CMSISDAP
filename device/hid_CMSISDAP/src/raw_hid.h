#include "tusb.h"

#ifndef __RAW_HID_H
#define __RAW_HID_H

#define RAW_REP_SIZE 64

static inline void raw_hid_send(uint8_t* buffer, uint16_t bufsize) {
	tud_hid_report(0, buffer, bufsize);
}

#endif //　__RAW_HID_H
