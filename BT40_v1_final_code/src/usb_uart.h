/*usb serial port output for debugging*/

#ifndef USB_UART_H
#define USB_UART_H 1
#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include <nrfx_log.h>

#include <zephyr/console/console.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/drivers/uart.h>


struct serial_peer {
	const struct device *dev;
//	struct serial_peer *data;
//	struct ring_buf rb;
};

#define DEFINE_SERIAL_PEER(node_id) { .dev = DEVICE_DT_GET(node_id),},

static struct serial_peer peers[] = {
	DT_FOREACH_STATUS_OKAY(zephyr_cdc_acm_uart, DEFINE_SERIAL_PEER)
};

BUILD_ASSERT(ARRAY_SIZE(peers) >= 1, "Not enough CDC ACM instances");

#define COM12 peers[0].dev


// Callback for writing to UART. Called when transaction is completed.
volatile int UART_DONE = 0;
void UART_Callback(const struct device *dev, struct uart_event *evt, void *user_data)
{
	UART_DONE = 1;
	printk("UART_Callback was called.\n");
}