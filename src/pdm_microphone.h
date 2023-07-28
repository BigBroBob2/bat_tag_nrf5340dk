/*PDM microphone: MP34DT01-M*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include <zephyr/audio/dmic.h>

#include "usb_uart.h"

#define PDM_pin_clk   25 // P0.25
#define PDM_pin_din   26 // P0.26

// #define PDM_BUF_SIZE 16
// static int16_t PDM_buff1[PDM_BUF_SIZE];
// static int16_t PDM_buff2[PDM_BUF_SIZE];
// static bool PDM_flag =0;
// static bool PDM_WriteFlag = 0;

#define MAX_SAMPLE_RATE  16000
#define SAMPLE_BIT_WIDTH 16
#define BYTES_PER_SAMPLE sizeof(int16_t)

/* Milliseconds to wait for a block to be read. */
#define READ_TIMEOUT     10000

/* Size of a block for 1000/10 = 100 ms of audio data. */
#define BLOCK_SIZE(_sample_rate, _number_of_channels) \
	(BYTES_PER_SAMPLE * (_sample_rate / 1) * _number_of_channels)

/* Driver will allocate blocks from this slab to receive audio data into them.
 * Application, after getting a given block from the driver and processing its
 * data, needs to free that block.
 */
#define MAX_BLOCK_SIZE   BLOCK_SIZE(MAX_SAMPLE_RATE, 2)
#define BLOCK_COUNT      4
K_MEM_SLAB_DEFINE_STATIC(mem_slab, MAX_BLOCK_SIZE, BLOCK_COUNT, 4);


static int do_pdm_transfer(const struct device *dmic_dev,
			   struct dmic_cfg *cfg,
			   size_t block_count)
{
	int error;

	printk("PCM output rate: %u, channels: %u\n",
		cfg->streams[0].pcm_rate, cfg->channel.req_num_chan);

	error = dmic_configure(dmic_dev, cfg);
	if (error < 0) {
		printk("Failed to configure the driver: %d\n", error);
		return error;
	}

	error = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
	if (error < 0) {
		printk("START trigger failed: %d\n", error);
		return error;
	}

	for (int i = 0; i < block_count; ++i) {
		void *buffer;
		uint32_t size;
		int error;

		error = dmic_read(dmic_dev, 0, &buffer, &size, READ_TIMEOUT);
		if (error < 0) {
			printk("%d - read failed: %d\n", i, error);
			return error;
		}

		printk("%d - got buffer %p of %u bytes\n", i, buffer, size);

        PDM_WriteBufBinary(buffer,size);

		k_mem_slab_free(&mem_slab, &buffer);
	}

	error = dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
	if (error < 0) {
		printk("STOP trigger failed: %d\n", error);
		return error;
	}

	return error;
}

void PDM_WriteBufBinary(int16_t *buf, uint32_t size){

    uint32_t N;

    uart_poll_out(COM12, 0xff);
	uart_poll_out(COM12, 0xff);

    N = size;
    for(int r=0;r<N;r++) {
        uart_poll_out(COM12,buf[r]);
    }


}
