/*PDM microphone: MP34DT01-M*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include <zephyr/audio/dmic.h>
#include <zephyr/drivers/i2s.h>
#include <nrfx_i2s.h>

// #ifndef USB_UART_H
// #define USB_UART_H 1
// #include "usb_uart.h"
// #endif

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
DO NOT USE THIS TRICK ANYMORE!!!
This will damage the hardware PDM controller on the board.
dmic_build_channel_map(0, 0, PDM_CHAN_LEFT), will need to change to the other PDM_CHAN_LEFT/RIGHT after damage
//////////////////////////////////////
To get higher PDM sampling rate, need to manually increase PDM CLK freq:

ncs\v2.4.0\modules\hal\nordic\nrfx\mdk\nrf5340_application_bitfields.h, line 8783:
- add more PDMCLKCTRL options for higher CLK freq

ncs\v2.4.0\modules\hal\nordic\nrfx\hal\nrf_pdm.h, line 116:
- add more PDM_freq according to CLK

F:\ncs\v2.4.0\zephyr\drivers\audio\dmic_nrfx_pdm.c, line 185:
- add more PDM freq

*/
#define MAX_SAMPLE_RATE  55000



#define SAMPLE_BIT_WIDTH 32
#define BYTES_PER_SAMPLE sizeof(int32_t)
#define NUMBER_OF_CHANNELS  2
#define SAMPLES_PER_BLOCK  (4096) // need to consider enlarging block_size (long enough time) to avoid stream blocking 
#define BLOCK_SIZE_foo (BYTES_PER_SAMPLE * SAMPLES_PER_BLOCK)
#define INITIAL_BLOCKS   16
#define BLOCK_COUNT      INITIAL_BLOCKS
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

// static bool configure_streams(const struct device *dmic_dev)
// {
// 	int ret;

// 	struct pcm_stream_cfg stream = {
// 		.pcm_width = SAMPLE_BIT_WIDTH,
// 		.mem_slab  = &mem_slab,
// 	};
// 	struct dmic_cfg cfg = {
// 		.io = {
// 			/* These fields can be used to limit the PDM clock
// 			 * configurations that the driver is allowed to use
// 			 * to those supported by the microphone.
// 			 */
// 			.min_pdm_clk_freq = 3000000,
// 			.max_pdm_clk_freq = 3250000,
// 			.min_pdm_clk_dc   = 40,
// 			.max_pdm_clk_dc   = 60,
// 		},
// 		.streams = &stream,
// 		.channel = {
// 			.req_num_streams = 1,
// 		},
// 	};

//     cfg.channel.req_num_chan = NUMBER_OF_CHANNELS;
// 	cfg.channel.req_chan_map_lo =
// 		dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
// 	cfg.streams[0].pcm_rate = MAX_SAMPLE_RATE;
    
// 	cfg.streams[0].block_size = BLOCK_SIZE_foo;

// 	ret = dmic_configure(dmic_dev, &cfg);
// 	if (ret < 0) {
// 		printk("Failed to configure stream: %d\n", ret);
// 		return false;
// 	}

// 	return true;
// }

// static bool prepare_transfer(const struct device *dmic_dev)
// {
// 	int ret;

// 	for (int i = 0; i < INITIAL_BLOCKS; ++i) {
// 		void *mem_block;
// 		size_t block_size;

// 		ret = k_mem_slab_alloc(&mem_slab, &mem_block, K_NO_WAIT);
// 		if (ret < 0) {
// 			printk("Failed to allocate block %d: %d\n", i, ret);
// 			return false;
// 		}
// 		memset(mem_block, 0, BLOCK_SIZE_foo);
// 		ret = dmic_read(dmic_dev, 0, &mem_block, &block_size, READ_TIMEOUT);
// 		if (ret < 0) {
// 			printk("Failed to write block %d: %d\n", i, ret);
// 			return false;
// 		}
// 	}

// 	return true;
// }

// static bool dmic_trigger_command(const struct device *dmic_dev, enum dmic_trigger cmd)
// {
// 	int ret;

// 	ret = dmic_trigger(dmic_dev, cmd);
// 	if (ret < 0) {
// 		printk("Failed to trigger command %d: %d\n", cmd, ret);
// 		return false;
// 	}

// 	return true;
// }


// static int do_pdm_transfer(const struct device *dmic_dev,
// 			   struct dmic_cfg *cfg,
// 			   size_t block_count)
// {
// 	int error;

// 	printk("PCM output rate: %u, channels: %u\n",
// 		cfg->streams[0].pcm_rate, cfg->channel.req_num_chan);

// 	error = dmic_configure(dmic_dev, cfg);
// 	if (error < 0) {
// 		printk("Failed to configure the driver: %d\n", error);
// 		return error;
// 	}

// 	error = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
// 	if (error < 0) {
// 		printk("START trigger failed: %d\n", error);
// 		return error;
// 	}

// 	for (int i = 0; i < block_count; ++i) {
// 		void *buffer;
// 		uint32_t size;
// 		int error;

// 		error = dmic_read(dmic_dev, 0, &buffer, &size, READ_TIMEOUT);
// 		if (error < 0) {
// 			printk("%d - read failed: %d\n", i, error);
// 			return error;
// 		}

// 		printk("%d - got buffer %p of %u bytes\n", i, buffer, size);

//         PDM_WriteBufBinary(buffer,size);

// 		k_mem_slab_free(&mem_slab, &buffer);
// 	}

// 	error = dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
// 	if (error < 0) {
// 		printk("STOP trigger failed: %d\n", error);
// 		return error;
// 	}

// 	return error;
// }

// void PDM_WriteBufBinary(int16_t *buf, uint32_t size){

//     uint32_t N;

//     uart_poll_out(COM12, 0xff);
// 	uart_poll_out(COM12, 0xff);

//     N = size;
//     for(int r=0;r<N;r++) {
//         uart_poll_out(COM12,buf[r]);
//     }


// }

////////////////////////////////////////////////////////////
// I2S interface for PDM microphone
////////////////////////////////////////////////////////////

// /* Use two pairs of rx/tx buffers for double buffering,
//    i.e process/render one pair of buffers while the other is
//    being received/transfered. */
// static int32_t __attribute__((aligned(4))) rx_1[SAMPLES_PER_BLOCK];
// static int32_t __attribute__((aligned(4))) rx_2[SAMPLES_PER_BLOCK];

// nrfx_i2s_buffers_t nrfx_i2s_buffers_1 = {
//     .p_rx_buffer = (uint32_t *)(&rx_1), .p_tx_buffer = NULL
// };

// nrfx_i2s_buffers_t nrfx_i2s_buffers_2 = {
//     .p_rx_buffer = (uint32_t *)(&rx_2), .p_tx_buffer = NULL
// };

// // /* Indicates if the current buffer pair is being processed/rendered. */
// atomic_t processing_in_progress = ATOMIC_INIT(0x00);
// // /* Indicates if a dropout occurred, i.e that audio buffers were
// //    not processed in time for the next buffer swap. */
// atomic_t dropout_occurred = ATOMIC_INIT(0x00);

// uint32_t processing_sem_give_time = 0;

// K_SEM_DEFINE(processing_thread_semaphore, 0, 1);

// static bool i2s_buf_flag = false;

// void nrfx_i2s_data_handler(nrfx_i2s_buffers_t const *p_released, uint32_t status)
// {	

//     if(NRF_I2S0->EVENTS_RXPTRUPD != 0)
// 	{
// 		if (!i2s_buf_flag) {
// 			NRF_I2S0->RXD.PTR = &nrfx_i2s_buffers_2.p_rx_buffer;
// 			NRF_I2S0->EVENTS_RXPTRUPD = 0;
// 			i2s_buf_flag = true;
// 		}
// 		else {
// 			NRF_I2S0->RXD.PTR = &nrfx_i2s_buffers_1.p_rx_buffer;
// 			NRF_I2S0->EVENTS_RXPTRUPD = 0;
// 			i2s_buf_flag = false;
// 		}
// 	}
// }


// #define I2S_pin_SCK   25
// #define I2S_pin_DIN  26

static bool configure_i2s_rx(const struct device *i2s_rx_dev)
{	

	// ///////////////////////// register configuration
	// // enable rx
	// NRF_I2S0->CONFIG.RXEN = I2S_CONFIG_RXEN_RXEN_Enabled;

	// // enable MCK generator
	// NRF_I2S0->CONFIG.MCKEN = I2S_CONFIG_MCKEN_MCKEN_Enabled;

	// // MCKFREQ = 4 MHz
    // NRF_I2S0->CONFIG.MCKFREQ = I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV11  << I2S_CONFIG_MCKFREQ_MCKFREQ_Pos;

	// // Ratio = 64
	// NRF_I2S0->CONFIG.RATIO = I2S_CONFIG_RATIO_RATIO_64X << I2S_CONFIG_RATIO_RATIO_Pos;

	// // Master mode, 32Bit, left aligned
    // NRF_I2S0->CONFIG.MODE = NRF_I2S_MODE_MASTER << I2S_CONFIG_MODE_MODE_Pos;
    // NRF_I2S0->CONFIG.SWIDTH = NRF_I2S_SWIDTH_32BIT << I2S_CONFIG_SWIDTH_SWIDTH_Pos;
    // NRF_I2S0->CONFIG.ALIGN = NRF_I2S_ALIGN_LEFT << I2S_CONFIG_ALIGN_ALIGN_Pos;

	// // Format = I2S
    // NRF_I2S0->CONFIG.FORMAT = I2S_CONFIG_FORMAT_FORMAT_I2S << I2S_CONFIG_FORMAT_FORMAT_Pos;

	// // Use stereo 
    // NRF_I2S0->CONFIG.CHANNELS = NRF_I2S_CHANNELS_STEREO << I2S_CONFIG_CHANNELS_CHANNELS_Pos;

	// // Configure pins
    // // NRF_I2S0->PSEL.MCK = (PIN_MCK << I2S_PSEL_MCK_PIN_Pos);
    // NRF_I2S0->PSEL.SCK = (I2S_pin_SCK << I2S_PSEL_SCK_PIN_Pos); 
    // // NRF_I2S0->PSEL.LRCK = (PIN_LRCK << I2S_PSEL_LRCK_PIN_Pos); 
    // NRF_I2S0->PSEL.SDIN = (I2S_pin_DIN << I2S_PSEL_SDIN_PIN_Pos);

	// // configure data pointer
	// NRF_I2S0->RXD.PTR = (uint32_t)&mem_slab;
	// NRF_I2S0->RXTXD.MAXCNT =  SAMPLES_PER_BLOCK;

	// NRF_I2S0->ENABLE = 1;

	// Start transmitting I2S data
    // NRF_I2S0->TASKS_START = 1;

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

	// set sampling rate by configurating registers

	// // MCKFREQ = 32/6 = 5.3 MHz
    // NRF_I2S0->CONFIG.MCKFREQ = I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV6  << I2S_CONFIG_MCKFREQ_MCKFREQ_Pos;
	// // RATIO = 32, LRCK = MCK/32 = 166.7 kHz
	// NRF_I2S0->CONFIG.RATIO = I2S_CONFIG_RATIO_RATIO_32X << I2S_CONFIG_RATIO_RATIO_Pos;
	
	// nrfx_i2s_config_t nrfx_i2s_cfg = {
    //     .sck_pin = I2S_pin_SCK,
    //     .sdin_pin = PDM_pin_din,
    //     .mck_setup = NRF_I2S_MCK_32MDIV11,
    //     .ratio = NRF_I2S_RATIO_32X,
    //     .irq_priority = NRFX_I2S_DEFAULT_CONFIG_IRQ_PRIORITY,
    //     .mode = NRF_I2S_MODE_MASTER,
    //     .format = NRF_I2S_FORMAT_I2S,
    //     .alignment = NRF_I2S_ALIGN_LEFT,
    //     .channels = NRF_I2S_CHANNELS_LEFT,
    //     .sample_width = NRF_I2S_SWIDTH_32BIT
    // };

    // nrfx_err_t result = nrfx_i2s_init(&nrfx_i2s_cfg, &nrfx_i2s_data_handler);

	// result = nrfx_i2s_start(&nrfx_i2s_buffers_1, BLOCK_SIZE_foo / 4, 0);
    // if (result != NRFX_SUCCESS) {
    //     printk("nrfx_i2s_start failed with result %d", result);
    //     __ASSERT(result == NRFX_SUCCESS, "nrfx_i2s_start failed with result %d", result);
    //     return result;
    // }

	return true;
}


