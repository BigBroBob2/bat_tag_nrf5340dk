#include "icm42688.h"
#include "pdm_microphone.h"

#ifndef BT_H
#define BT_H 1
#include "bt.h"
#endif

#ifndef SDCARD_H
#define SDCARD_H 1
#include "SDcard.h"
#endif

#include "circular_buffer.h"
#include <stdio.h>
#include <nrfx_log.h>
#include <nrfx_gpiote.h>

#include <power/reboot.h>

// define order is important, must define before use
static int trial_count = 0;

/////////////////////////////////////////////////////////////////////////// Define circular buffer
// IMU circular buffer
static short imu_cbuf[N_circular_buf]; 
// IMU sample data circular buffer
static circular_buf imu_circle_buf = {
    .buf = imu_cbuf,
    .write_idx = 0,
    .read_idx = 0
};

// IMU count circular buffer
// static uint16_t imu_count_idx[1]={0};
// static short imu_count_buf[N_circular_buf]; 
// // IMU sample count circular buffer
// circular_buf count_circle_buf = {
//     .buf = imu_count_buf,
//     .write_idx = 0,
//     .read_idx = 0
// };



// 2nd SPI IMU circular buffer
static short H_imu_cbuf[N_circular_buf]; 
// 2nd SPI IMU sample data circular buffer
static circular_buf H_imu_circle_buf = {
    .buf = H_imu_cbuf,
    .write_idx = 0,
    .read_idx = 0
};

// 2nd SPI IMU count circular buffer
// static uint16_t H_imu_count_idx[1]={0};
// static short H_imu_count_buf[N_circular_buf]; 
// // 2nd SPI IMU sample count circular buffer
// circular_buf H_count_circle_buf = {
//     .buf = H_imu_count_buf,
//     .write_idx = 0,
//     .read_idx = 0
// };
/////////////////////////////////////////////////////////////////////////// finish define circular buf

/// SD card data file
static struct fs_file_t mic_file;

// static struct fs_file_t imu_t_file;

static struct fs_file_t imu_file;
// static struct fs_file_t imu_count_file;

static struct fs_file_t H_imu_file;
// static struct fs_file_t H_imu_count_file;

// unsigned long int next = 1;
// int rand(void)
// {
//     return k_cycle_get_32();
// }

// void tmpnam(char *name, int length) {
//     const char alphanum[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuv";
//     int stringLength = sizeof(alphanum) - 1;

//     for(int i = 0; i < length; ++i)
//     {
//         name[i] = alphanum[rand() % stringLength];
//     }
// }

static int define_files() {
    int error;
    char filename[30];

    ///// get trial name
    // char trial_name[8]; 
    // tmpnam(&trial_name[0], 8);

    //////// mic_file for microphone audio data
    fs_file_t_init(&mic_file);
    printk("Opening mic_file path\n");
	sprintf(&filename, "/SD:/audio_%02d.dat", trial_count); 

    // delete mic_file if exist
    fs_unlink(filename);

	error = fs_open(&mic_file, filename, FS_O_CREATE | FS_O_WRITE);
    if (error) {
			printk("Error opening mic_file [%03d]\n", error);
			return 0;
		}

    ///////// imu_file for IMU data
    fs_file_t_init(&imu_file);
    printk("Opening imu_file path\n");

	sprintf(&filename, "/SD:/imu_%02d.dat", trial_count); 

    // delete imu_file if exist
    fs_unlink(filename);

	error = fs_open(&imu_file, filename, FS_O_CREATE | FS_O_WRITE);
    if (error) {
			printk("Error opening imu_file [%03d]\n", error);
			return 0;
		}

    //////// imu_t_file for ICM time data
    // fs_file_t_init(&imu_t_file);
    // printk("Opening imu_t_file path\n");

	// sprintf(&filename, "/SD:/imu_t_%02d.dat", trial_count); 

    // // delete imu_t_file if exist
    // fs_unlink(filename);

	// error = fs_open(&imu_t_file, filename, FS_O_CREATE | FS_O_WRITE);
    // if (error) {
	// 		printk("Error opening imu_t_file [%03d]\n", error);
	// 		return 0;
	// 	}

    ///////// H_imu_file for 2nd head IMU data
    fs_file_t_init(&H_imu_file);
    printk("Opening H_imu_file path\n");

	sprintf(&filename, "/SD:/h_imu_%02d.dat", trial_count); 

    // delete H_imu_file if exist
    fs_unlink(filename);

	error = fs_open(&H_imu_file, filename, FS_O_CREATE | FS_O_WRITE);
    if (error) {
			printk("Error opening H_imu_file [%03d]\n", error);
			return 0;
		}


    //////// imu_count_file for ICM count data
    // fs_file_t_init(&imu_count_file);
    // printk("Opening imu_count_file path\n");

	// sprintf(&filename, "/SD:/imu_c_%02d.dat",trial_count); 

    // // delete imu_count_file if exist
    // fs_unlink(filename);

	// error = fs_open(&imu_count_file, filename, FS_O_CREATE | FS_O_WRITE);
    // if (error) {
	// 		printk("Error opening imu_count_file [%03d]\n", error);
	// 		return 0;
	// 	}

    // return 0;
}

//////////////////////////////////////////////////////////////////////////////// ICM thread
// SPI ICM thread
static short imu_rbuf[9]; // small read buf to get direct data each time IRQ
static short imu_buf[N_circular_buf]; // IMU buffer
static short imu_cw_buf[N_circular_buf]; // IMU count write buffer
static int32_t imu_t; // timestamp for each IMU writing 
static uint16_t ICM_count = 0; // sample count within the buffer, be really careful with uint8_t and int8_t
static uint16_t ICM_cw=0;

struct k_thread ICM_thread_data;
/*ICM processing thread*/
#define ICM_THREAD_STACK_SIZE 2048
/* ICM thread is a priority cooperative thread, less than PDM mic*/
#define ICM_THREAD_PRIORITY -16
K_THREAD_STACK_DEFINE(ICM_thread_stack_area, ICM_THREAD_STACK_SIZE);
K_SEM_DEFINE(ICM_thread_semaphore, 0, 1);

static void ICM_thread_entry_point(void *p1, void *p2, void *p3) {
    while (true) {
        if (k_sem_take(&ICM_thread_semaphore, K_FOREVER) == 0) {

            ICM_readSensor();
             
            imu_t = k_cycle_get_32();
            IMU_data[6] = imu_t >> 16;
            IMU_data[7] = imu_t & 0x0000FFFF;
            IMU_data[8]++; 

            memcpy(imu_rbuf,&IMU_data[0],sizeof(IMU_data)); 

            // write data into circular buf
            write_in_buf(&imu_circle_buf,imu_rbuf,9);

    }
}
}

// 2nd SPI ICM thread
static short H_imu_rbuf[9]; // small read buf to get direct data each time IRQ
static short H_imu_buf[N_circular_buf]; // IMU buffer
static short H_imu_cw_buf[N_circular_buf]; // IMU count write buffer
static int32_t H_imu_t; // timestamp for each IMU writing 
static uint16_t H_ICM_count = 0; // sample count within the buffer, be really careful with uint8_t and int8_t
static uint16_t H_ICM_cw=0;

struct k_thread H_ICM_thread_data;
/*ICM processing thread*/
#define H_ICM_THREAD_STACK_SIZE 2048
/* ICM thread is a priority cooperative thread, less than PDM mic*/
#define H_ICM_THREAD_PRIORITY -16
K_THREAD_STACK_DEFINE(H_ICM_thread_stack_area, H_ICM_THREAD_STACK_SIZE);
K_SEM_DEFINE(H_ICM_thread_semaphore, 0, 1);

static void H_ICM_thread_entry_point(void *p1, void *p2, void *p3) {
    while (true) {
        if (k_sem_take(&H_ICM_thread_semaphore, K_FOREVER) == 0) {
            
            // NRF_P0->PIN_CNF[31] = 1;
            // NRF_P0->OUT |= 1 << 31;

            H_ICM_readSensor();

            H_imu_t = k_cycle_get_32();
            H_IMU_data[6] = H_imu_t >> 16;
            H_IMU_data[7] = H_imu_t & 0x0000FFFF;
            H_IMU_data[8]++;

            memcpy(H_imu_rbuf,&H_IMU_data[0],sizeof(H_IMU_data)); 

            // write data into circular buf
            write_in_buf(&H_imu_circle_buf,H_imu_rbuf,9);


            // NRF_P0->OUTCLR |= 1 << 31;
    }
}
}

ISR_DIRECT_DECLARE(ICM_handler)
{   
    // https://github.com/gcmcnutt/HeadTracker/blob/4d5e3fb4793d519110b5812090c64401381d3b56/firmware/src/src/targets/nrf52/PPMIn.cpp#L191
    ISR_DIRECT_HEADER();
    // printk("EVENTS_IN[7]=%d, CONFIG[7].PSEL=0x%x\n",NRF_GPIOTE0->EVENTS_IN[7],NRF_GPIOTE0->CONFIG[7]);
    if (NRF_GPIOTE0->EVENTS_IN[6]) {

            NRF_GPIOTE0->EVENTS_IN[6] = 0;
            k_sem_give(&H_ICM_thread_semaphore);
        
        }
    else if (NRF_GPIOTE0->EVENTS_IN[7]) {
        NRF_GPIOTE0->EVENTS_IN[7] = 0;
            k_sem_give(&ICM_thread_semaphore);
            
        }

    return 0;
}

//////////////////////////////////////////////////////////////////////////////////// I2S PDM mic

/* Flag indicating which buffer pair is currently available for
   processing/rendering. */
static int processing_buffers_1 = 0;

/* Indicates if the current buffer pair is being processed/rendered. */
atomic_t processing_in_progress = ATOMIC_INIT(0x00);
/* Indicates if a dropout occurred, i.e that audio buffers were
   not processed in time for the next buffer swap. */
atomic_t dropout_occurred = ATOMIC_INIT(0x00);

static uint32_t mic_t[1];
int32_t *rx;


#define BYTES_PER_SAMPLE 4
#define AUDIO_BUFFER_N_SAMPLES 4096
#define AUDIO_BUFFER_BYTE_SIZE (BYTES_PER_SAMPLE * AUDIO_BUFFER_N_SAMPLES)
#define AUDIO_BUFFER_WORD_SIZE (AUDIO_BUFFER_BYTE_SIZE / 4)

/* Use two pairs of rx/tx buffers for double buffering,
   i.e process/render one pair of buffers while the other is
   being received/transfered. */
static int32_t __attribute__((aligned(4))) rx_1[AUDIO_BUFFER_N_SAMPLES];
static int32_t __attribute__((aligned(4))) rx_2[AUDIO_BUFFER_N_SAMPLES];
nrfx_i2s_buffers_t nrfx_i2s_buffers_1 = {
    .p_rx_buffer = (uint32_t *)(&rx_1), .p_tx_buffer = NULL
};
nrfx_i2s_buffers_t nrfx_i2s_buffers_2 = {
    .p_rx_buffer = (uint32_t *)(&rx_2), .p_tx_buffer = NULL
};

// struct __attribute__((packed)) Bool_Byte {
// 	bool x0 : 1;
// 	bool x1 : 1;
// 	bool x2 : 1;
// 	bool x3 : 1;
// 	bool x4 : 1;
// 	bool x5 : 1;
// 	bool x6 : 1;
// 	bool x7 : 1;
// };
// struct Bool_Byte rx_downsample[AUDIO_BUFFER_N_SAMPLES];

// int get_bit_every_bits(uint32_t *data_in,struct Bool_Byte *data_out, int N)
// {
//     int c = 0;
//     int32_t test = 1;

//     for (c = 0;c<N;c++) {
//       data_out[c].x0 = (data_in[c] & 0x00000001) >> 0;
//       data_out[c].x1 = (data_in[c] & 0x00000010) >> 1;
//       data_out[c].x2 = (data_in[c] & 0x00000100) >> 2;
//       data_out[c].x3 = (data_in[c] & 0x00001000) >> 3;
//       data_out[c].x4 = (data_in[c] & 0x00010000) >> 4;
//       data_out[c].x5 = (data_in[c] & 0x00100000) >> 5;
//       data_out[c].x6 = (data_in[c] & 0x01000000) >> 6;
//       data_out[c].x7 = (data_in[c] & 0x10000000) >> 7;
//     }
//     return 0;
// };

// learn I2S thread from https://github.com/stuffmatic/microdsp-zephyr-demos/blob/497e0bb7fc23103827d3e283d5374ed00ea315c7/src/i2s.c#L63
/*Audio processing thread*/
#define PROCESSING_THREAD_STACK_SIZE 2048
/* Processing thread is a top priority cooperative thread */
#define PROCESSING_THREAD_PRIORITY -15
K_THREAD_STACK_DEFINE(processing_thread_stack_area, PROCESSING_THREAD_STACK_SIZE);
K_SEM_DEFINE(processing_thread_semaphore, 0, 1);
struct k_thread processing_thread_data;

K_SEM_DEFINE(SDcard_thread_semaphore, 0, 1);

static void processing_thread_entry_point(void *p1, void *p2, void *p3) {
    while (true) {
        

        if (k_sem_take(&processing_thread_semaphore, K_USEC(1)) == 0) {

            // can use this as timestamp of this audio buffer
            mic_t[0] = k_cycle_get_32();
            // fs_write(&mic_file,&mic_t,sizeof(mic_t));

            if (atomic_test_bit(&dropout_occurred, 0)) {
                printk("dropout_occurred\n");
                atomic_clear_bit(&dropout_occurred, 0);
            }

            

            if (!atomic_test_bit(&processing_in_progress, 0)) {
                printk("processing_in_progress is not set");
                __ASSERT(atomic_test_bit(&processing_in_progress, 0), "processing_in_progress is not set");
            }

            nrfx_i2s_buffers_t* buffers_to_process = processing_buffers_1 ? &nrfx_i2s_buffers_1 : &nrfx_i2s_buffers_2;
            rx = (int32_t *)buffers_to_process->p_rx_buffer;

            
            ///////////////// somehow mic_data and IMU_data fs_write together can work

            /* Swap buffers */
            nrfx_err_t result = nrfx_i2s_next_buffers_set(buffers_to_process);
            if (result != NRFX_SUCCESS) {
                /* printk("nrfx_i2s_next_buffers_set failed with %d\n", result); */
                __ASSERT(result == NRFX_SUCCESS, "nrfx_i2s_next_buffers_set failed with result %d", result);
            }
            processing_buffers_1 = !processing_buffers_1;   


            k_sem_give(&SDcard_thread_semaphore);

        
            /* Done audio processing */
            atomic_clear_bit(&processing_in_progress, 0);
        }
        
    }
}

ISR_DIRECT_DECLARE(i2s_isr_handler)
{
    nrfx_i2s_irq_handler();
    // ISR_DIRECT_PM();
    return 1;
}

void nrfx_i2s_data_handler(nrfx_i2s_buffers_t const *p_released, uint32_t status)
{   
    
    if (status == NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED) {
        if (atomic_test_bit(&processing_in_progress, 0) == false) {
            atomic_set_bit(&processing_in_progress, 0);
            k_sem_give(&processing_thread_semaphore);
        } else {
            /* Missed deadline! */
            atomic_set_bit(&dropout_occurred, 0);
        }
    }
    else if (status == NRFX_I2S_STATUS_TRANSFER_STOPPED) {
        /* */
    }

}



/*
Configure i2s driver using zephyr functions, 
which are kind of convenient and by using them we don't need to set register values and later define RX buffers.

We only care about SCK (bit stream freq) and LRCK is in fact meaningless;
change nrfx_i2s_cfg.mck_setup to choose different MCK freq
*/
static bool configure_i2s_rx(const struct device *i2s_rx_dev)
{	
	int ret;

     

    IRQ_DIRECT_CONNECT(I2S0_IRQn, 0, i2s_isr_handler, 0);

    nrfx_i2s_config_t nrfx_i2s_cfg = {
        .sck_pin = I2S_SCK,
        .lrck_pin = I2S_LRCK,
        .mck_pin = I2S_MCK,
        .sdin_pin = I2S_SDIN,
        .mck_setup = I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV32,
        .ratio = I2S_CONFIG_RATIO_RATIO_32X,
        .irq_priority = NRFX_I2S_DEFAULT_CONFIG_IRQ_PRIORITY,
        .mode = NRF_I2S_MODE_MASTER,
        .format = NRF_I2S_FORMAT_I2S,
        .alignment = NRF_I2S_ALIGN_LEFT,
        .channels = NRF_I2S_CHANNELS_STEREO,
        .sample_width = NRF_I2S_SWIDTH_32BIT
    };

    nrfx_err_t result = nrfx_i2s_init(&nrfx_i2s_cfg, &nrfx_i2s_data_handler);
    if (result != NRFX_SUCCESS) {
        printk("nrfx_i2s_init failed with result %d\n", result);
        __ASSERT(result == NRFX_SUCCESS, "nrfx_i2s_init failed with result %d", result);
        return result;
    }


	if (ret < 0) {
		printk("Failed to configure i2s rx: %d\n", ret);
		return false;
	}

	return true;
}

//////////////////////////////////////////////////////////////////////////////// thread for writing into SD card

struct k_thread SDcard_thread_data;
#define SDcard_THREAD_STACK_SIZE 1024
#define SDcard_THREAD_PRIORITY -14
K_THREAD_STACK_DEFINE(SDcard_thread_stack_area, SDcard_THREAD_STACK_SIZE);


static void SDcard_thread_entry_point(void *p1, void *p2, void *p3) {
    while (true) {
        if (k_sem_take(&SDcard_thread_semaphore, K_FOREVER) == 0) {
            // audio
            fs_write(&mic_file,&mic_t,sizeof(mic_t));
            fs_write(&mic_file,&rx[0],AUDIO_BUFFER_BYTE_SIZE);

            // IMU 
            read_out_buf(&imu_circle_buf,imu_buf,&ICM_count);
            fs_write(&imu_file, &imu_buf,ICM_count*sizeof(short));
            read_out_buf(&H_imu_circle_buf,imu_buf,&H_ICM_count);
            fs_write(&H_imu_file, &imu_buf,H_ICM_count*sizeof(short));
    }
}
}

struct k_work_delayable bl5340_vregh_reset_delayed_work;
/** @brief Delayed work handler used to trigger a reset when VREGHVOUT changes.
 *
 *  @param [in]item - Delayed work item data.
 */
static void bl5340_vregh_reset_handler(struct k_work *item)
{
	/* Now reset to allow changes to take effect */
	sys_reboot(SYS_REBOOT_WARM);
}

void main(void)
{   
    int error;
    
    ////// set high-voltage regulator to convert VDDH=4.2V to VDD=3.7V 
    /* Block writes if the register is already set */
	if ((NRF_UICR->VREGHVOUT) != UICR_VREGHVOUT_VREGHVOUT_3V3) {
    /* Enable writes to the UICR */
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen
			   << NVMC_CONFIG_WEN_Pos;
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
	}
	/* Now go ahead and apply the voltage */
	NRF_UICR->VREGHVOUT = UICR_VREGHVOUT_VREGHVOUT_3V3;
	/* And finalise the write */
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren
			   << NVMC_CONFIG_WEN_Pos;
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
	}
    k_work_init_delayable(
					&bl5340_vregh_reset_delayed_work,
					bl5340_vregh_reset_handler);
	/* Then trigger it */
	k_work_schedule(
					&bl5340_vregh_reset_delayed_work,
					K_MSEC(1000));
    }
	/* Read back to validate */
    // while (true) {
	    // printk("NRF_UICR->VREGHVOUT=%x\n", NRF_UICR->VREGHVOUT);     
        // k_msleep(1000);
    // }
    // NRF_REGULATORS->MAINREGSTATUS = REGULATORS_MAINREGSTATUS_VREGH_Active;
    

    //////////////////////////// SD card
	
    setup_disk();

    k_msleep(1000);

    trial_count = lsdir(disk_mount_pt);

    //////////////////////////////////////////////// bluetooth uart 
    error = bt_enable(NULL);
	if (error) {
		led_show_error();
	}

    printk("Bluetooth initialized\n");

    k_sem_give(&ble_init_ok);

    settings_load();

    error = bt_nus_init(&nus_cb);
	if (error) {
		printk("Failed to initialize UART service (error: %d)\n", error);
		return 0;
	}

    // start advertising
    error = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd,
			      ARRAY_SIZE(sd));
	if (error) {
		printk("Advertising failed to start (error %d)\n", error);
		return 0;
	}

    
    k_tid_t SDcard_thread_tid = k_thread_create(
        &SDcard_thread_data,
        SDcard_thread_stack_area,
        K_THREAD_STACK_SIZEOF(SDcard_thread_stack_area),
        SDcard_thread_entry_point,
        NULL, NULL, NULL,
        SDcard_THREAD_PRIORITY, 0, K_NO_WAIT
    );
    
    ///////////////////////////////////////// Here starts multiple trial loop
    while (true) {
    // wait for bluetooth command
    while (!current_conn) {
        printk("Trial %02d, Wait for bluetooth connection..\n", trial_count);
        k_msleep(1000);
    } 


    
    define_files();

    // int8_t vreg[1];
    // vreg[0] = NRF_UICR->VREGHVOUT;
    // fs_write(&imu_t_file,&vreg[0],sizeof(int8_t));

    //////////////////////////////////////////////// SPI for second IMU

    // check ICM binding
    if (!spi_dev1) {
        printk("spi_dev1 Binding failed.");
        return;
    }

    printk("Value of NRF_SPIM1->PSEL.SCK : %d \n",NRF_SPIM1->PSEL.SCK);
    printk("Value of NRF_SPIM1->PSEL.MOSI : %d \n",NRF_SPIM1->PSEL.MOSI);
    printk("Value of NRF_SPIM1->PSEL.MISO : %d \n",NRF_SPIM1->PSEL.MISO);

    /// config SPI first
    H_ICM_SPI_config();

    // set ICM samping rate
    H_ICM_setSamplingRate(rate_idx);

    // enable interrupt
    H_ICM_enableINT();

    // enable sensor
    H_ICM_enableSensor();

    

    

    ////////////////////////////////////////////  SPI devices IMU

    // range scale
    double ICM_GyroScale =  ICM_ADC2Float(ICM_GyroRangeVal_dps(ICM_GyroRange_idx));
    double ICM_AccelScale = ICM_ADC2Float(ICM_AccelRangeVal_G(ICM_AccelRange_idx));
    double ICM_Samplerate = ICM_SampRate(ICM_DataRate_idx);

    // check ICM binding
    if (!spi_dev2) {
        printk("spi_dev2 Binding failed.");
        return;
    }
    printk("Value of NRF_SPIM2->PSEL.SCK : %d \n",NRF_SPIM2->PSEL.SCK);
    printk("Value of NRF_SPIM2->PSEL.MOSI : %d \n",NRF_SPIM2->PSEL.MOSI);
    printk("Value of NRF_SPIM2->PSEL.MISO : %d \n",NRF_SPIM2->PSEL.MISO);
    // printk("Value of NRF_SPIM2 frequency : %d \n",NRF_SPIM2->FREQUENCY);

    /// config SPI first
    ICM_SPI_config();
    
    // set ICM samping rate
    ICM_setSamplingRate(rate_idx);

    // enable interrupt
    ICM_enableINT();

    // enable sensor
    ICM_enableSensor();
    
    // while(1) {

    //     H_ICM_readSensor();
    //     printk("%d, %d, %d, ", H_IMU_data[0],H_IMU_data[1],H_IMU_data[2]);
    //     k_msleep(5);

    //     ICM_readSensor();
    //     printk("%d, %d, %d \n", IMU_data[0],IMU_data[1],IMU_data[2]);
        
    //     k_msleep(5);
    // }

    // GPIO configure interrupt
    // gpio_pin_interrupt_configure(GPIO_dev,ICM_INT_pin,GPIO_INT_EDGE_RISING);
    // gpio_init_callback(&ICM_INT_cb,ICM_INT_handler,BIT(ICM_INT_pin));
    // GPIOTE0 for secure
    IRQ_DIRECT_CONNECT(GPIOTE0_IRQn, 0, ICM_handler, 0);

    


    // try give semaphore as IMU interrupt
    // Event mode, P0.30 for interrupt, when rising edge
    // check NRF_GPIOTE->EVENTS_IN[0]
    nrf_gpiote_event_configure(NRF_GPIOTE0,6,9,GPIOTE_CONFIG_POLARITY_LoToHi);

    // try give semaphore as IMU interrupt
    // Event mode, P0.25 for interrupt, when rising edge
    // NRF_GPIOTE->CONFIG[0] = 0x00011901;
    // check NRF_GPIOTE->EVENTS_IN[0]
    nrf_gpiote_event_configure(NRF_GPIOTE0,7,25,GPIOTE_CONFIG_POLARITY_LoToHi);
    

    k_msleep(10);

    //////////////////////////////////////////////// I2S for PDM mic

    /* Start a dedicated, high priority thread for audio processing. */
    k_tid_t processing_thread_tid = k_thread_create(
        &processing_thread_data,
        processing_thread_stack_area,
        K_THREAD_STACK_SIZEOF(processing_thread_stack_area),
        processing_thread_entry_point,
        NULL, NULL, NULL,
        PROCESSING_THREAD_PRIORITY, 0, K_NO_WAIT
    );

    if (!device_is_ready(i2s_rx_dev)) {
		printk("unable to find i2s_rx device\n");	
	}

    if (!configure_i2s_rx(i2s_rx_dev)) {
        printk("Failed to config streams\n", i2s_rx_dev->name);
		return 0;
    }
    printk("I2S configured\n");

    // f_actual = f_source / floor(1048576*4096/MCKFREQ)
    printk("NRF I2S0 CONFIG.MCKFREQ = %d, f_actual = %.3f\n", NRF_I2S0->CONFIG.MCKFREQ, 0.00745*NRF_I2S0->CONFIG.MCKFREQ);
   

    


    ///////////////////////////////////////////////////////////////// START SAMPLE
    // try to use thread to read ICM data
    k_tid_t ICM_thread_tid = k_thread_create(
        &ICM_thread_data,
        ICM_thread_stack_area,
        K_THREAD_STACK_SIZEOF(ICM_thread_stack_area),
        ICM_thread_entry_point,
        NULL, NULL, NULL,
        ICM_THREAD_PRIORITY, 0, K_NO_WAIT
    );
    // gpio_add_callback(GPIO_dev, &ICM_INT_cb);

    // use another thread to read 2nd ICM data
    k_tid_t H_ICM_thread_tid = k_thread_create(
        &H_ICM_thread_data,
        H_ICM_thread_stack_area,
        K_THREAD_STACK_SIZEOF(H_ICM_thread_stack_area),
        H_ICM_thread_entry_point,
        NULL, NULL, NULL,
        H_ICM_THREAD_PRIORITY, 0, K_NO_WAIT
    );

    // reset cirular buffer idx and processing_buffers
        // Reset variables at the start of each trial
    // imu_count_idx[0]=0;
    imu_circle_buf.read_idx = 0;
    imu_circle_buf.write_idx = 0;
    // count_circle_buf.read_idx = 0;
    // count_circle_buf.write_idx = 0;

    // H_imu_count_idx[0]=0;
    H_imu_circle_buf.read_idx = 0;
    H_imu_circle_buf.write_idx = 0;
    // H_count_circle_buf.read_idx = 0;
    // H_count_circle_buf.write_idx = 0;

    processing_buffers_1 = 0;  

    // clear count
    IMU_data[8] = 0;
    H_IMU_data[8] = 0;

    
    printk("Starting I2S stream..\n");

    // start I2S
    error = nrfx_i2s_start(&nrfx_i2s_buffers_1, AUDIO_BUFFER_WORD_SIZE, 0);
    
    nrf_i2s_int_enable(NRF_I2S0, NRF_I2S_INT_RXPTRUPD_MASK |
                                  NRF_I2S_INT_TXPTRUPD_MASK);

    // enable GPIOTE
    NRF_GPIOTE0->EVENTS_IN[7] = 0;
    NRF_GPIOTE->INTENSET |= 0x00000080;
    NRF_GPIOTE0->EVENTS_IN[6] = 0;
    NRF_GPIOTE->INTENSET |= 0x00000040;
    // enable gpiote event (IMU INT)
    nrf_gpiote_event_enable(NRF_GPIOTE0, 7);
    nrf_gpiote_event_enable(NRF_GPIOTE0, 6);
    // enable GPIOTE IRQ
    NRFX_IRQ_ENABLE(GPIOTE0_IRQn);
    NVIC_EnableIRQ(GPIOTE0_IRQn);
    irq_enable(GPIOTE0_IRQn);

    // while (current_conn) 
    // {   
    //     printk("buf_length_imu=%d, buf_length_H_imu=%d\n",buf_length(&imu_circle_buf),buf_length(&H_imu_circle_buf));
    //     k_msleep(1);
    // }
    
    ///////////////////////////////////////// loop
    while (current_conn){
        k_msleep(1000);
    }
    
    ///////////////////// after loop

    nrf_i2s_int_disable(NRF_I2S0, NRF_I2S_INT_RXPTRUPD_MASK |
                                  NRF_I2S_INT_TXPTRUPD_MASK);
    // nrfx_i2s_stop();

    k_thread_abort(ICM_thread_tid);
    k_thread_abort(H_ICM_thread_tid);
    k_thread_abort(processing_thread_tid);

    NRFX_IRQ_DISABLE(GPIOTE0_IRQn);
    NVIC_DisableIRQ(GPIOTE0_IRQn);
    irq_disable(GPIOTE0_IRQn);

    NRF_GPIOTE->INTENCLR |= 0x00000080;
    nrf_gpiote_event_disable(NRF_GPIOTE0, 7);
    NRF_GPIOTE->INTENCLR |= 0x00000040;
    nrf_gpiote_event_disable(NRF_GPIOTE0, 6);

    

    k_msleep(1000);

	printk("Stream stopped\n");

	printk("File named in trial %02d successfully created\n", trial_count);		
	fs_close(&mic_file);
    fs_close(&imu_file);
    fs_close(&H_imu_file);
    // fs_close(&imu_t_file);
    // fs_close(&imu_count_file);

    k_msleep(1000);
    lsdir(disk_mount_pt);
    k_msleep(1000);

    // bt_le_adv_stop();

    trial_count = trial_count + 1;
}
}
