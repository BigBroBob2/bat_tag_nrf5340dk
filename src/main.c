#ifndef BT_H
#define BT_H 1
#include "bt.h"
#endif

#ifndef ICM42688_H
#define ICM42688_H 1
#include "icm42688.h"
#endif

#ifndef MIC_H
#define MIC_H 1
#include "pdm_microphone.h"
#endif

#ifndef SDCARD_H
#define SDCARD_H 1
#include "SDcard.h"
#endif


#ifndef CIRBUF_H
#define CIRBUF_H 1
#include "circular_buffer.h"
#endif


#ifndef NANEYEC_H
#define NANEYEC_H 1
#include "NanEyeC.h"
#endif


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

// 2nd SPI IMU circular buffer
static short H_imu_cbuf[N_circular_buf]; 
// 2nd SPI IMU sample data circular buffer
static circular_buf H_imu_circle_buf = {
    .buf = H_imu_cbuf,
    .write_idx = 0,
    .read_idx = 0
};

/////////////////////////////////////////////////////////////////////////// finish define circular buf

//// SD card data file

// save file for audio
static struct fs_file_t mic_file;
// static struct fs_file_t imu_t_file;

// save file for camera frame
static struct fs_file_t video_file;

// save file for body imu
static struct fs_file_t imu_file;

// save file for head imu
static struct fs_file_t H_imu_file;

static int define_files() {
    int error;
    char filename[30];
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

    //////// video_file for camera
    fs_file_t_init(&video_file);
    printk("Opening video_file path\n");

	sprintf(&filename, "/SD:/video_%02d.dat",trial_count); 

    // delete video_file if exist
    fs_unlink(filename);

	error = fs_open(&video_file, filename, FS_O_CREATE | FS_O_WRITE);
    if (error) {
			printk("Error opening video_file [%03d]\n", error);
			return 0;
		}

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


    //////// imu_count_file for IMU count data
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

//////////////////////////////////////////////////////////////////////////////// camera thread
// bufer for write into video files
static uint8_t camera_buf[frame_block_size*frame_block_circular_buf_num];

static uint8_t camera_cbuf[frame_block_size*frame_block_circular_buf_num];
static uint16_t camera_count;

static uint32_t cam_t[1];

// static char camera_cbuf[frame_block_circular_buf_size]; 
// IMU sample data circular buffer
static frame_block_circular_buf camera_circle_buf = {
    .buf = &camera_cbuf,
    .write_idx = 0,
    .read_idx = 0
};

// image processing thread
struct k_thread image_processing_thread_data;
#define image_processing_THREAD_STACK_SIZE 512
#define image_processing_THREAD_PRIORITY -12
K_THREAD_STACK_DEFINE(image_processing_thread_stack_area, image_processing_THREAD_STACK_SIZE);
K_SEM_DEFINE(image_processing_thread_semaphore, 0, 1);

static void image_processing_thread_entry_point(void *p1, void *p2, void *p3) {
    while (true) {
        if (k_sem_take(&image_processing_thread_semaphore, K_FOREVER) == 0) {

            NRF_P0->PIN_CNF[13] = 1;
            NRF_P0->OUTSET |= 1 << 13;

            image_compress();
            // add timestamp
            memcpy(&Image_binary[6400],&cam_t[0],4); 

            frame_block_write_in_buf(&camera_circle_buf,&Image_binary[0]);

            NRF_P0->OUTCLR |= 1 << 13;
        }
    }
}

// SPI camera thread
struct k_thread camera_thread_data;
#define camera_THREAD_STACK_SIZE 512
#define camera_THREAD_PRIORITY -16
K_THREAD_STACK_DEFINE(camera_thread_stack_area, camera_THREAD_STACK_SIZE);
K_SEM_DEFINE(camera_thread_semaphore, 0, 1);

static void camera_thread_entry_point(void *p1, void *p2, void *p3) {
    while (true) {
        if (k_sem_take(&camera_thread_semaphore, K_FOREVER) == 0) {

            // can use this as timestamp of one frame
            cam_t[0] = k_cycle_get_32();
            
            NRF_P0->PIN_CNF[11] = 1;
            NRF_P0->OUTSET |= 1 << 11;

            // NanEye_ReadFrame();
            int key = irq_lock();
            
            for(int i=0; i < 4; ++i) {
                spi_ReadBuffer_sleep(&ImageBuf[i*80][0], 492*80, 200);
            }

            irq_unlock(key);

            k_sem_give(&image_processing_thread_semaphore);
            
            spi_ReadBytes(12); // End of frame, should be all zeros - read and discard.
            NanEye_WriteConfig();
            spi_WriteBuffer(camera_rwbuf, 966);
            spi_ReadBuffer(camera_rwbuf, 4*492);

            NRF_P0->OUTCLR |= 1 << 11;
    }
}
}

//////////////////////////////////////////////////////////////////////////////// IMU thread
// SPI IMU thread
static short imu_rbuf[9]; // small read buf to get direct data each time IRQ
static short imu_buf[N_circular_buf]; // IMU buffer
static short imu_cw_buf[N_circular_buf]; // IMU count write buffer
static int32_t imu_t; // timestamp for each IMU writing 
static uint16_t ICM_count = 0; // sample count within the buffer, be really careful with uint8_t and int8_t
// static uint16_t ICM_cw=0;

struct k_thread ICM_thread_data;
/*IMU processing thread*/
#define ICM_THREAD_STACK_SIZE 1024
/* IMU thread is a priority cooperative thread, less than PDM mic*/
#define ICM_THREAD_PRIORITY -16
K_THREAD_STACK_DEFINE(ICM_thread_stack_area, ICM_THREAD_STACK_SIZE);
K_SEM_DEFINE(ICM_thread_semaphore, 0, 1);

static void ICM_thread_entry_point(void *p1, void *p2, void *p3) {
    while (true) {
        if (k_sem_take(&ICM_thread_semaphore, K_FOREVER) == 0) {

            // get timestamp asap
            imu_t = k_cycle_get_32();

            NRF_P0->PIN_CNF[12] = 1;
            NRF_P0->OUTSET |= 1 << 12;

            ICM_readSensor();
            
            IMU_data[6] = imu_t >> 16;
            IMU_data[7] = imu_t & 0x0000FFFF;
            IMU_data[8]++; 

            // memcpy(imu_rbuf,&IMU_data[0],sizeof(IMU_data)); 

            // write data into circular buf
            write_in_buf(&imu_circle_buf,IMU_data,9);

            NRF_P0->OUTCLR |= 1 << 12;
    }
}
}

// 2nd SPI IMU thread
static short H_imu_rbuf[9]; // small read buf to get direct data each time IRQ
// static short H_imu_buf[N_circular_buf]; // IMU buffer
static short H_imu_cw_buf[N_circular_buf]; // IMU count write buffer
static int32_t H_imu_t; // timestamp for each IMU writing 
static uint16_t H_ICM_count = 0; // sample count within the buffer, be really careful with uint8_t and int8_t
// static uint16_t H_ICM_cw=0;

struct k_thread H_ICM_thread_data;
/*IMU processing thread*/
#define H_ICM_THREAD_STACK_SIZE 1024
/* IMU thread is a priority cooperative thread, less than PDM mic*/
#define H_ICM_THREAD_PRIORITY -16
K_THREAD_STACK_DEFINE(H_ICM_thread_stack_area, H_ICM_THREAD_STACK_SIZE);
K_SEM_DEFINE(H_ICM_thread_semaphore, 0, 1);

static void H_ICM_thread_entry_point(void *p1, void *p2, void *p3) {
    while (true) {
        if (k_sem_take(&H_ICM_thread_semaphore, K_FOREVER) == 0) {
            
            H_imu_t = k_cycle_get_32();

            H_ICM_readSensor();
            
            H_IMU_data[6] = H_imu_t >> 16;
            H_IMU_data[7] = H_imu_t & 0x0000FFFF;
            H_IMU_data[8]++;

            // memcpy(H_imu_rbuf,&H_IMU_data[0],sizeof(H_IMU_data)); 

            // write data into circular buf
            write_in_buf(&H_imu_circle_buf,H_IMU_data,9);

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
static int32_t *rx;


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

            // can use this as timestamp of next audio buffer start time
            mic_t[0] = k_cycle_get_32();
            // fs_write(&mic_file,&mic_t,sizeof(mic_t));

            NRF_P0->PIN_CNF[31] = 1;
            NRF_P0->OUTSET |= 1 << 31;

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
            
        
            NRF_P0->OUTCLR |= 1 << 31;
            
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
        .mck_setup = i2s_mckfreq,
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
        
            // camera
            frame_block_read_out_buf(&camera_circle_buf,camera_buf, &camera_count);
            // printk("camera_count=%d\n",camera_count);
		    fs_write(&video_file, &camera_buf,camera_count*frame_block_size);

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
    ////// high-voltage mode VDD can be very noisy and because of this SD card cannot work normally, need to add capacitor between VDD and GND 
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
	printk("NRF_REGULATORS->MAINREGSTATUS=0x%x, NRF_UICR->VREGHVOUT=0x%x\n", NRF_REGULATORS->MAINREGSTATUS, NRF_UICR->VREGHVOUT);     

    // NRF_REGULATORS->MAINREGSTATUS = REGULATORS_MAINREGSTATUS_VREGH_Active;
    

    //////////////////////////// SD card
	
    setup_disk();

    k_msleep(1000);

    trial_count = lsdir(disk_mount_pt);

    //////////////////////////////////////////////// bluetooth uart 
    error = bt_enable(NULL);
	if (error) {
		printk("Bluetooth initialize error\n");
	}

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
    	

    k_tid_t image_processing_thread_tid = k_thread_create(
        &image_processing_thread_data,
        image_processing_thread_stack_area,
        K_THREAD_STACK_SIZEOF(image_processing_thread_stack_area),
        image_processing_thread_entry_point,
        NULL, NULL, NULL,
        image_processing_THREAD_PRIORITY, 0, K_NO_WAIT
    );

    /////////// start SD card thread
    k_tid_t SDcard_thread_tid = k_thread_create(
        &SDcard_thread_data,
        SDcard_thread_stack_area,
        K_THREAD_STACK_SIZEOF(SDcard_thread_stack_area),
        SDcard_thread_entry_point,
        NULL, NULL, NULL,
        SDcard_THREAD_PRIORITY, 0, K_NO_WAIT
    );

    ////////////////// wait for bluetooth connection
    while (!current_conn) {
        printk("Wait for bluetooth connection..\n");
        k_msleep(1000);
    } 
    // connected

    ///////////////////////////////////////// Here starts multiple trial loop
    while (current_conn) {
    while(!trial_start && current_conn) {
        printk("Trial %02d, Wait for command..\n", trial_count);
        k_msleep(1000);
    }

    // Somehow this is not working, probably only 4 files at the same time
    // Write configuration into a file
    // struct fs_file_t cfg_file;
    // char filename[30];
    // char str_buf[50];
    // fs_file_t_init(&cfg_file);
    // printk("Opening cfg_file path\n");
	// sprintf(&filename, "/SD:/config_%02d.dat", trial_count); 
    // fs_unlink(filename);
	// error = fs_open(&cfg_file, filename, FS_O_CREATE | FS_O_WRITE);
    // sprintf(&str_buf, "imu_fs=0x%02x, mic_fs=0x%08x\n", rate_idx, i2s_mckfreq);
    // fs_write(&cfg_file,&str_buf,sizeof(str_buf));
    // fs_close(&cfg_file);

    // k_msleep(1000);
    // lsdir(disk_mount_pt);
    // k_msleep(1000);
    
    
    // define data files
    define_files();

    //////////////// Init NanEyeC camera
    NRF_P0->PIN_CNF[SCLK_pin] = 0;
	NRF_P0->PIN_CNF[SDAT_pin] = 0;
    InitSPI();
	NanEye_InitialConfig();
    ReSYNC();


    //////////////////////////////////////////////// SPI for second IMU

    if (enable_H_imu) {
        // check IMU binding
    if (!spi_dev1) {
        printk("spi_dev1 Binding failed.");
        return;
    }

    printk("H_IMU enabled, enable_H_imu=%d\n", enable_H_imu);
    printk("H_IMU sampling rate imu_fs=%.2f\n", ICM_SampRate(rate_idx));

    /// config SPI first
    H_ICM_SPI_config();

    // set IMU samping rate
    H_ICM_setSamplingRate(rate_idx);

    // enable interrupt
    H_ICM_enableINT();

    // enable sensor
    H_ICM_enableSensor();
    }
    else {
        printk("H_IMU disabled, enable_H_imu=%d\n", enable_H_imu);
    }

    ////////////////////////////////////////////  SPI devices IMU

    if (enable_imu) {
        // check IMU binding
    if (!spi_dev2) {
        printk("spi_dev2 Binding failed.");
        return;
    }
    
    printk("IMU enabled, enable_imu=%d\n", enable_imu);
    printk("IMU sampling rate imu_fs=%.2f\n", ICM_SampRate(rate_idx));
    
    /// config SPI first
    ICM_SPI_config();
    
    // set IMU samping rate
    ICM_setSamplingRate(rate_idx);

    // enable interrupt
    ICM_enableINT();

    // enable sensor
    ICM_enableSensor();
    }
    else {
        printk("IMU disabled, enable_imu=%d\n", enable_imu);
    }

    // GPIO configure interrupt
    // gpio_pin_interrupt_configure(GPIO_dev,ICM_INT_pin,GPIO_INT_EDGE_RISING);
    // gpio_init_callback(&ICM_INT_cb,ICM_INT_handler,BIT(ICM_INT_pin));
    // GPIOTE0 for secure
    IRQ_DIRECT_CONNECT(GPIOTE0_IRQn, 0, ICM_handler, 0);

    // try give semaphore as IMU interrupt
    // Event mode, P0.30 for interrupt, when rising edge
    // check NRF_GPIOTE->EVENTS_IN[0]
    nrf_gpiote_event_configure(NRF_GPIOTE0,6,30,GPIOTE_CONFIG_POLARITY_LoToHi);

    // try give semaphore as IMU interrupt
    // Event mode, P0.25 for interrupt, when rising edge
    // NRF_GPIOTE->CONFIG[0] = 0x00011901;
    // check NRF_GPIOTE->EVENTS_IN[0]
    nrf_gpiote_event_configure(NRF_GPIOTE0,7,25,GPIOTE_CONFIG_POLARITY_LoToHi);
    

    k_msleep(10);

    /////////////////////// printk enable_cam

    if (enable_cam) {
        printk("cam enabled, enable_cam=%d\n", enable_cam);
    }
    else {
        printk("cam disabled, enable_cam=%d\n", enable_cam);
    }

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

        // f_actual = f_source / floor(1048576*4096/MCKFREQ)
        printk("NRF I2S0 CONFIG.MCKFREQ = %d, f_actual = %.3f\n", NRF_I2S0->CONFIG.MCKFREQ, 0.00745*NRF_I2S0->CONFIG.MCKFREQ);


    ///////////////////////////////////////////////////////////////// START SAMPLE

     // thread for SD card
    
    // try to use thread to read IMU data
    k_tid_t ICM_thread_tid = k_thread_create(
        &ICM_thread_data,
        ICM_thread_stack_area,
        K_THREAD_STACK_SIZEOF(ICM_thread_stack_area),
        ICM_thread_entry_point,
        NULL, NULL, NULL,
        ICM_THREAD_PRIORITY, 0, K_NO_WAIT
    );
    // gpio_add_callback(GPIO_dev, &ICM_INT_cb);

    // use another thread to read 2nd IMU data
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
    imu_circle_buf.read_idx = 0;
    imu_circle_buf.write_idx = 0;

    H_imu_circle_buf.read_idx = 0;
    H_imu_circle_buf.write_idx = 0;

    camera_circle_buf.read_idx = 0;
    camera_circle_buf.write_idx = 0;


    processing_buffers_1 = 0;  

    // clear count
    IMU_data[8] = 0;
    H_IMU_data[8] = 0;

    
    printk("Start recording..\n");

    // start I2S
    error = nrfx_i2s_start(&nrfx_i2s_buffers_1, AUDIO_BUFFER_WORD_SIZE, 0);
    
    nrf_i2s_int_enable(NRF_I2S0, NRF_I2S_INT_RXPTRUPD_MASK |
                                  NRF_I2S_INT_TXPTRUPD_MASK);
    

    // enable GPIOTE
    if (enable_imu) {
         NRF_GPIOTE0->EVENTS_IN[7] = 0;
        NRF_GPIOTE->INTENSET |= 0x00000080;
    }
    if (enable_H_imu) {
        NRF_GPIOTE0->EVENTS_IN[6] = 0;
        NRF_GPIOTE->INTENSET |= 0x00000040;
    }
    
    // enable gpiote event (IMU INT)
    if (enable_imu) {
        nrf_gpiote_event_enable(NRF_GPIOTE0, 7);
    }
    
    if (enable_H_imu) {
        nrf_gpiote_event_enable(NRF_GPIOTE0, 6);
    }
    
    // enable GPIOTE IRQ
    NRFX_IRQ_ENABLE(GPIOTE0_IRQn);
    NVIC_EnableIRQ(GPIOTE0_IRQn);
    irq_enable(GPIOTE0_IRQn);


    // start camera
    k_tid_t camera_thread_tid = k_thread_create(
        &camera_thread_data,
        camera_thread_stack_area,
        K_THREAD_STACK_SIZEOF(camera_thread_stack_area),
        camera_thread_entry_point,
        NULL, NULL, NULL,
        camera_THREAD_PRIORITY, 0, K_NO_WAIT
    );
    
    ///////////////////////////////////////// loop
    if (enable_cam) {
        while (trial_start && current_conn){
            k_sem_give(&camera_thread_semaphore);
            k_msleep(cam_interval);
        }
    }
    else {
        while (trial_start && current_conn){
            k_msleep(1000);
        }
    }
    
    ///////////////////// after loop
    nrf_i2s_int_disable(NRF_I2S0, NRF_I2S_INT_RXPTRUPD_MASK |
                                  NRF_I2S_INT_TXPTRUPD_MASK);
    nrfx_i2s_uninit();
    
    k_thread_abort(camera_thread_tid);
    k_thread_abort(ICM_thread_tid);
    k_thread_abort(H_ICM_thread_tid);
    k_thread_abort(processing_thread_tid);
    

    NRFX_IRQ_DISABLE(GPIOTE0_IRQn);
    NVIC_DisableIRQ(GPIOTE0_IRQn);
    irq_disable(GPIOTE0_IRQn);

    if (enable_imu) {
        NRF_GPIOTE->INTENCLR |= 0x00000080;
        nrf_gpiote_event_disable(NRF_GPIOTE0, 7);
    }
    if (enable_H_imu) {
        NRF_GPIOTE->INTENCLR |= 0x00000040;
    nrf_gpiote_event_disable(NRF_GPIOTE0, 6);
    } 

    k_msleep(1000);

	printk("Recording stopped\n");

	printk("File named in trial %02d successfully created\n", trial_count);		
	fs_close(&mic_file);
    fs_close(&imu_file);
    fs_close(&H_imu_file);
    fs_close(&video_file);
    // fs_close(&imu_count_file);

    k_msleep(1000);
    lsdir(disk_mount_pt);
    k_msleep(1000);

    // bt_le_adv_stop();

    trial_count = trial_count + 1;
}

printk("Bluetooth disconnected...Stop all\n");
return 0;
}
