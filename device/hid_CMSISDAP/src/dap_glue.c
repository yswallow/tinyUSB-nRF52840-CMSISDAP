#include "raw_hid.h"

#include "DAP_const.h"
#include "DAP.h"
#include "usb_desc.h"

#include <string.h>
#include <stdint.h>

static uint32_t free_count;
static uint32_t send_count;
static uint32_t recv_idx;
static uint32_t send_idx;
static volatile uint8_t  USB_ResponseIdle;
static uint8_t USB_Request[DAP_PACKET_COUNT][DAP_PACKET_SIZE];  // Request  Buffer
uint8_t rawhidResponse[RAW_REP_SIZE];
#define FREE_COUNT_INIT          (DAP_PACKET_COUNT)
#define SEND_COUNT_INIT          0


void hid_send_packet(void)
{
    if (send_count) {
        send_count--;
        raw_hid_send(USB_Request[send_idx], DAP_PACKET_SIZE);
        send_idx = (send_idx + 1) % DAP_PACKET_COUNT;
        free_count++;
    }
}

//uint8_t buf64[64];
void raw_hid_receive(const uint8_t* buf, const uint16_t len)
{
    if (buf[0] == ID_DAP_TransferAbort) {
        DAP_TransferAbort = 1;
    }

    // Store data into request packet buffer
    // If there are no free buffers discard the data
    if (free_count > 0) {
        free_count--;
        memcpy(USB_Request[recv_idx], buf, len);
        //memcpy(buf64, buf, len);
        //buf64[63] = 0;
        //USB_Request[recv_idx][63] = 0;
        DAP_ExecuteCommand(buf, USB_Request[recv_idx]);
        recv_idx = (recv_idx + 1) % DAP_PACKET_COUNT;
        send_count++;
        if (USB_ResponseIdle) {
            hid_send_packet();            
        }
    }
}

void DAP_init(void) {
    DAP_Setup();

    recv_idx = 0;
    send_idx = 0;
    USB_ResponseIdle = 1;
    free_count = FREE_COUNT_INIT;
    send_count = SEND_COUNT_INIT;
}
