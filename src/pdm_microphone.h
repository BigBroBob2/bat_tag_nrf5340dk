/*PDM microphone: MP34DT01-M*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include <zephyr/audio/dmic.h>

#ifndef USB_UART_H
#define USB_UART_H 1
#include "usb_uart.h"
#endif

#ifndef SDCARD_H
#define SDCARD_H 1
#include "SDcard.h"
#endif

#define PDM_pin_clk   25 // P0.25
#define PDM_pin_din   26 // P0.26

// #define PDM_BUF_SIZE 16
// static int16_t PDM_buff1[PDM_BUF_SIZE];
// static int16_t PDM_buff2[PDM_BUF_SIZE];
// static bool PDM_flag =0;
// static bool PDM_WriteFlag = 0;


/* 
To get higher PDM sampling rate, need to manually increase PDM CLK freq:

ncs\v2.4.0\modules\hal\nordic\nrfx\mdk\nrf5340_application_bitfields.h, line 8783:
- add more PDMCLKCTRL options for higher CLK freq

ncs\v2.4.0\modules\hal\nordic\nrfx\hal\nrf_pdm.h, line 116:
- add more PDM_freq according to CLK

F:\ncs\v2.4.0\zephyr\drivers\audio\dmic_nrfx_pdm.c, line 185:
- add more PDM freq

*/
#define MAX_SAMPLE_RATE  50000



#define SAMPLE_BIT_WIDTH 16
#define BYTES_PER_SAMPLE sizeof(int16_t)
#define NUMBER_OF_CHANNELS  1
#define SAMPLES_PER_BLOCK 256 // ((MAX_SAMPLE_RATE / 10) * NUMBER_OF_CHANNELS)
#define BLOCK_SIZE_foo (BYTES_PER_SAMPLE * SAMPLES_PER_BLOCK)
#define INITIAL_BLOCKS   16
#define BLOCK_COUNT      (INITIAL_BLOCKS+2)
/* Milliseconds to wait for a block to be read. */
#define READ_TIMEOUT     1000

/* Size of a block for 1000/10 = 100 ms of audio data. */
// #define BLOCK_SIZE(_sample_rate, _number_of_channels) \
// 	(BYTES_PER_SAMPLE * (_sample_rate / 1) * _number_of_channels)

/* Driver will allocate blocks from this slab to receive audio data into them.
 * Application, after getting a given block from the driver and processing its
 * data, needs to free that block.
 */
// #define MAX_BLOCK_SIZE   BLOCK_SIZE(MAX_SAMPLE_RATE, 2)

K_MEM_SLAB_DEFINE_STATIC(mem_slab, BLOCK_SIZE_foo, BLOCK_COUNT, 4);

static bool configure_streams(const struct device *dmic_dev)
{
	int ret;

	struct pcm_stream_cfg stream = {
		.pcm_width = SAMPLE_BIT_WIDTH,
		.mem_slab  = &mem_slab,
	};
	struct dmic_cfg cfg = {
		.io = {
			/* These fields can be used to limit the PDM clock
			 * configurations that the driver is allowed to use
			 * to those supported by the microphone.
			 */
			.min_pdm_clk_freq = 3000000,
			.max_pdm_clk_freq = 3250000,
			.min_pdm_clk_dc   = 40,
			.max_pdm_clk_dc   = 60,
		},
		.streams = &stream,
		.channel = {
			.req_num_streams = 1,
		},
	};

    cfg.channel.req_num_chan = NUMBER_OF_CHANNELS;
	cfg.channel.req_chan_map_lo =
		dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
	cfg.streams[0].pcm_rate = MAX_SAMPLE_RATE;
    
	cfg.streams[0].block_size = BLOCK_SIZE_foo;

	ret = dmic_configure(dmic_dev, &cfg);
	if (ret < 0) {
		printk("Failed to configure stream: %d\n", ret);
		return false;
	}

	return true;
}

static bool prepare_transfer(const struct device *dmic_dev)
{
	int ret;

	for (int i = 0; i < INITIAL_BLOCKS; ++i) {
		void *mem_block;
		size_t block_size;

		ret = k_mem_slab_alloc(&mem_slab, &mem_block, K_NO_WAIT);
		if (ret < 0) {
			printk("Failed to allocate block %d: %d\n", i, ret);
			return false;
		}
		memset(mem_block, 0, BLOCK_SIZE_foo);
		ret = dmic_read(dmic_dev, 0, &mem_block, &block_size, READ_TIMEOUT);
		if (ret < 0) {
			printk("Failed to write block %d: %d\n", i, ret);
			return false;
		}
	}

	return true;
}

static bool trigger_command(const struct device *dmic_dev, enum dmic_trigger cmd)
{
	int ret;

	ret = dmic_trigger(dmic_dev, cmd);
	if (ret < 0) {
		printk("Failed to trigger command %d: %d\n", cmd, ret);
		return false;
	}

	return true;
}


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

////////////////////////////////////////////////////////////
// I2S interface for PDM microphone
////////////////////////////////////////////////////////////

