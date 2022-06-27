#include <stdint.h>

#ifndef __DAP_GLUE_H
#define __DAP_GLUE_H

void raw_hid_receive(const uint8_t* buf, const uint16_t len);
void DAP_init(void);

#endif // __DAP_GLUE_H
