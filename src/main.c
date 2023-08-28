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

#include <nrfx_log.h>

/// SD card data file
static struct fs_file_t mic_file, mic_t_file, imu_file, imu_t_file;

static uint32_t mic_t[1];

static short imu_buf[6000]; // IMU buffer
static int32_t imu_t[1]; // timestamp for each IMU sample
static short *imu_p = &imu_buf[0]; // pointer to current buffer head
static int8_t ICM_count = 0; // sample count within the buffer

/////// ICM thread
struct k_thread ICM_thread_data;
/*ICM processing thread*/
#define ICM_THREAD_STACK_SIZE 2048
/* ICM thread is a priority cooperative thread, less than PDM mic*/
#define ICM_THREAD_PRIORITY -16
K_THREAD_STACK_DEFINE(ICM_thread_stack_area, ICM_THREAD_STACK_SIZE);
K_SEM_DEFINE(ICM_thread_semaphore, 0, 1);
K_SEM_DEFINE(processing_thread_semaphore, 0, 1);

static void ICM_thread_entry_point(void *p1, void *p2, void *p3) {
    while (true) {
        if (k_sem_take(&ICM_thread_semaphore, K_FOREVER) == 0) {
            
            ICM_readSensor();
            memcpy(imu_p,&IMU_data[0],sizeof(IMU_data));   
            // imu_t[ICM_count] = k_cycle_get_32();

            ICM_count++;
            imu_p = &imu_buf[6*ICM_count];

            // fs_write(&imu_file,&IMU_data[0],sizeof(IMU_data));    
    }
}
}

////////////////////////// I2S PDM mic

/* Flag indicating which buffer pair is currently available for
   processing/rendering. */
static int processing_buffers_1 = 0;

/* Indicates if the current buffer pair is being processed/rendered. */
atomic_t processing_in_progress = ATOMIC_INIT(0x00);
/* Indicates if a dropout occurred, i.e that audio buffers were
   not processed in time for the next buffer swap. */
atomic_t dropout_occurred = ATOMIC_INIT(0x00);


// #define rx_buf_len  8192
// static int32_t rx_buf1[rx_buf_len] = {0};
// static int32_t rx_buf2[rx_buf_len] = {0};
// static int32_t rx_wbuf1[rx_buf_len] = {0};
// static int32_t rx_wbuf2[rx_buf_len] = {0};

static uint32_t mic_t[1];

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

/*Audio processing thread*/
#define PROCESSING_THREAD_STACK_SIZE 2048
/* Processing thread is a top priority cooperative thread */
#define PROCESSING_THREAD_PRIORITY -15
K_THREAD_STACK_DEFINE(processing_thread_stack_area, PROCESSING_THREAD_STACK_SIZE);
struct k_thread processing_thread_data;

uint32_t processing_sem_give_time = 0;

static void processing_thread_entry_point(void *p1, void *p2, void *p3) {
    while (true) {
        if (k_sem_take(&processing_thread_semaphore, K_FOREVER) == 0) {
            if (atomic_test_bit(&dropout_occurred, 0)) {
                printk("dropout_occurred\n");
                atomic_clear_bit(&dropout_occurred, 0);
            }
            // uint32_t processing_sem_take_time = k_cycle_get_32();
            // uint32_t cycles_spent = processing_sem_take_time - processing_sem_give_time;
            // uint32_t ns_spent = k_cyc_to_ns_ceil32(cycles_spent);
            // printk("processing thread start took %d ns\n", ns_spent); 

            if (!atomic_test_bit(&processing_in_progress, 0)) {
                printk("processing_in_progress is not set");
                __ASSERT(atomic_test_bit(&processing_in_progress, 0), "processing_in_progress is not set");
            }

            nrfx_i2s_buffers_t* buffers_to_process = processing_buffers_1 ? &nrfx_i2s_buffers_1 : &nrfx_i2s_buffers_2;
            int32_t *rx = (int32_t *)buffers_to_process->p_rx_buffer;

            // somehow mic_data and IMU_data fs_write together can work
            fs_write(&mic_file,&rx[0],AUDIO_BUFFER_BYTE_SIZE); 

            fs_write(&imu_file, &imu_buf,ICM_count*sizeof(IMU_data));
            ICM_count = 0;
            imu_p = &imu_buf[ICM_count];

            // printk("fs_write\n");

            // mic_t[0] = k_cycle_get_32();
            // fs_write(&mic_t_file,&mic_t[0],sizeof(mic_t));

            /* Swap buffers */
            nrfx_err_t result = nrfx_i2s_next_buffers_set(buffers_to_process);
            if (result != NRFX_SUCCESS) {
                /* printk("nrfx_i2s_next_buffers_set failed with %d\n", result); */
                __ASSERT(result == NRFX_SUCCESS, "nrfx_i2s_next_buffers_set failed with result %d", result);
            }
            processing_buffers_1 = !processing_buffers_1;   

            /* Done processing */
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
            processing_sem_give_time = k_cycle_get_32();
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

We only care about SCK (bit stream freq) and LRCK is in fact meaningless, 
so we need to play around word_size and frame_clk_freq to let it find a feasiable SCK
*/
static bool configure_i2s_rx(const struct device *i2s_rx_dev)
{	
	int ret;

     /* Start a dedicated, high priority thread for audio processing. */
    k_tid_t processing_thread_tid = k_thread_create(
        &processing_thread_data,
        processing_thread_stack_area,
        K_THREAD_STACK_SIZEOF(processing_thread_stack_area),
        processing_thread_entry_point,
        NULL, NULL, NULL,
        PROCESSING_THREAD_PRIORITY, 0, K_NO_WAIT
    );

      IRQ_DIRECT_CONNECT(I2S0_IRQn, 0, i2s_isr_handler, 0);

    nrfx_i2s_config_t nrfx_i2s_cfg = {
        .sck_pin = I2S_SCK,
        .lrck_pin = I2S_LRCK,
        .mck_pin = I2S_MCK,
        .sdin_pin = I2S_SDIN,
        .mck_setup = I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV16,
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


void main(void)
{   
    int error;

    //////////////////////////// SD card
	
    setup_disk();

    lsdir(disk_mount_pt);

    //////// mic_file for microphone audio data
    fs_file_t_init(&mic_file);
    printk("Opening mic_file path\n");
    char mic_filename[30];
	sprintf(&mic_filename, "/SD:/audio001.dat"); 

    // delete mic_file if exist
    fs_unlink(mic_filename);

	error = fs_open(&mic_file, mic_filename, FS_O_CREATE | FS_O_WRITE);
    if (error) {
			printk("Error opening mic_file [%03d]\n", error);
			return 0;
		}

    //////// mic_t_file for microphone time data
    fs_file_t_init(&mic_t_file);
    printk("Opening mic_t_file path\n");
    char mic_t_filename[30];
	sprintf(&mic_t_filename, "/SD:/audio_t.dat"); 

    // delete mic_t_file if exist
    fs_unlink(mic_t_filename);

	error = fs_open(&mic_t_file, mic_t_filename, FS_O_CREATE | FS_O_WRITE);
    if (error) {
			printk("Error opening mic_t_file [%03d]\n", error);
			return 0;
		}

    ///////// imu_file for IMU data
    fs_file_t_init(&imu_file);
    printk("Opening imu_file path\n");
    char imu_filename[30];
	sprintf(&imu_filename, "/SD:/imu.dat"); 

    // delete imu_file if exist
    fs_unlink(imu_filename);

	error = fs_open(&imu_file, imu_filename, FS_O_CREATE | FS_O_WRITE);
    if (error) {
			printk("Error opening imu_file [%03d]\n", error);
			return 0;
		}

    //////// imu_t_file for ICM time data
    fs_file_t_init(&imu_t_file);
    printk("Opening imu_t_file path\n");
    char imu_t_filename[30];
	sprintf(&imu_t_filename, "/SD:/imu_t.dat"); 

    // delete imu_t_file if exist
    fs_unlink(imu_t_filename);

	error = fs_open(&imu_t_file, imu_t_filename, FS_O_CREATE | FS_O_WRITE);
    if (error) {
			printk("Error opening imu_t_file [%03d]\n", error);
			return 0;
		}


    ////////////////////////////////////////////  SPI devices IMU

    // range scale
    double ICM_GyroScale =  ICM_ADC2Float(ICM_GyroRangeVal_dps(ICM_GyroRange_idx));
    double ICM_AccelScale = ICM_ADC2Float(ICM_AccelRangeVal_G(ICM_AccelRange_idx));
    double ICM_Samplerate = ICM_SampRate(ICM_DataRate_idx);

    // check ICM binding
    if (!spi_dev2) {
        printk("SPI_2 Binding failed.");
        return;
    }
    printk("Value of NRF_SPI2->PSEL.SCK : %d \n",NRF_SPIM2->PSEL.SCK);
    printk("Value of NRF_SPI2->PSEL.MOSI : %d \n",NRF_SPIM2->PSEL.MOSI);
    printk("Value of NRF_SPI2->PSEL.MISO : %d \n",NRF_SPIM2->PSEL.MISO);
    printk("Value of NRF_SPI2 frequency : %d \n",NRF_SPIM2->FREQUENCY);

    /// config SPI first
    ICM_SPI_config();
    // enable sensor
    ICM_enableSensor();
    
    //////////////////////////////////////////////// I2S for PDM mic

    const struct device *const i2s_rx_dev = DEVICE_DT_GET(DT_NODELABEL(i2s_rx_dev));

    if (!device_is_ready(i2s_rx_dev)) {
		printk("unable to find i2s_rx device\n");	
	}
    printk("I2S device is ready\n");

    if (!configure_i2s_rx(i2s_rx_dev)) {
        printk("Failed to config streams\n", i2s_rx_dev->name);
		return 0;
    }
    printk("I2S configured\n");

    // f_actual = f_source / floor(1048576*4096/MCKFREQ)
    printk("NRF I2S0 CONFIG.MCKFREQ = %d, f_actual = %.3f\n", NRF_I2S0->CONFIG.MCKFREQ, 0.00745*NRF_I2S0->CONFIG.MCKFREQ);
   

    //////////////////////////////////////////////// bluetooth uart 
    // init button
    error = dk_buttons_init(button_changed);
	if (error) {
		printk("Cannot init buttons (error: %d)\n", error);
	}

    //// init led
    error = dk_leds_init();
	if (error) {
		printk("Cannot init LEDs (error: %d)\n", error);
	}

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

    // wait for bluetooth command
    while (!current_conn) {
        printk("Wait for bluetooth connection..\n");
        k_msleep(1000);
    } 


    //////////////////////// START SAMPLE
    // try to use thread to read ICM data
    k_tid_t ICM_thread_tid = k_thread_create(
        &ICM_thread_data,
        ICM_thread_stack_area,
        K_THREAD_STACK_SIZEOF(ICM_thread_stack_area),
        ICM_thread_entry_point,
        NULL, NULL, NULL,
        ICM_THREAD_PRIORITY, 0, K_NO_WAIT
    );

    // start I2S
    error = nrfx_i2s_start(&nrfx_i2s_buffers_1, AUDIO_BUFFER_WORD_SIZE, 0);
    printk("I2S streams started\n");


    ///////////////////////////////////////// loop
    int while_count = 1;
    int while_end = 10000;

    while (true){
        // printk("While loop %d\n", while_count);
        // printk("ICM_count = %d\n", ICM_count);

        k_sem_give(&ICM_thread_semaphore);
        k_usleep(800); // not real IMU sampling rate
        
        if (while_count >= while_end) {
            break;
        }
        while_count++;
    }
    
    ///////////////////// after loop

    nrf_i2s_int_disable(NRF_I2S0, NRF_I2S_INT_RXPTRUPD_MASK |
                                  NRF_I2S_INT_TXPTRUPD_MASK);
    k_thread_abort(ICM_thread_tid);

    k_msleep(1000);

	printk("Stream stopped\n");

	printk("mic_file named \"audio001.dat\" successfully created\n");		
	fs_close(&mic_file);
    fs_close(&mic_t_file);
    fs_close(&imu_file);
    fs_close(&imu_t_file);

    k_msleep(1000);
    lsdir(disk_mount_pt);
}
