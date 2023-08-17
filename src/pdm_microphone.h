/*
Use I2S interface to transfer bit stream from PDM microphone
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include <zephyr/drivers/i2s.h>
#include <nrfx_i2s.h>


// These parameters will "choose" SCK
#define MAX_SAMPLE_RATE  100000 
#define SAMPLE_BIT_WIDTH 16 // bit word
#define NUMBER_OF_CHANNELS  2 // keep both 2 channels. 1 channel will throw away half data

// parameters for mem slab
#define BYTES_PER_SAMPLE sizeof(int16_t)
#define SAMPLES_PER_BLOCK  (8192) // need to consider enlarging block_size (long enough time) to avoid stream blocking 
#define BLOCK_SIZE_foo (BYTES_PER_SAMPLE * SAMPLES_PER_BLOCK)
#define BLOCK_COUNT      16
// Statically define and initialize a memory slab
K_MEM_SLAB_DEFINE_STATIC(mem_slab, BLOCK_SIZE_foo, BLOCK_COUNT, 4);



/*
Configure i2s driver using zephyr functions, 
which are kind of convenient and by using them we don't need to set register values and later define RX buffers.

We only care about SCK (bit stream freq) and LRCK is in fact meaningless, 
so we need to play around word_size and frame_clk_freq to let it find a feasiable SCK
*/
static bool configure_i2s_rx(const struct device *i2s_rx_dev)
{	
	int ret;
	struct i2s_config cfg;

	cfg.word_size = SAMPLE_BIT_WIDTH;
	cfg.channels = NUMBER_OF_CHANNELS;
	cfg.format = I2S_FMT_DATA_FORMAT_I2S;
	cfg.options = I2S_OPT_BIT_CLK_CONT | I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER;
	cfg.frame_clk_freq = MAX_SAMPLE_RATE;
	cfg.block_size = BLOCK_SIZE_foo;
	cfg.mem_slab = &mem_slab;
	cfg.timeout = SYS_FOREVER_MS;

	ret = i2s_configure(i2s_rx_dev, I2S_DIR_RX, &cfg);
	if (ret < 0) {
		printk("Failed to configure i2s rx: %d\n", ret);
		return false;
	}

	return true;
}


