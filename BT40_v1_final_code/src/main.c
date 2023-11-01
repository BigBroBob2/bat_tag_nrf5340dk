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
#include <nrfx_gpiote.h>

/////////////////////////////////////////////////////////////////////////// Define circular buffer
#define N_circular_buf 1024
typedef struct {
    short *buf; // actuall capacity = N-1
    int write_idx; // idx to write into circular buf
    int read_idx; // idx to read out from circular buf
} circular_buf;

static short imu_cbuf[N_circular_buf]; // IMU circular buffer
// IMU sample data circular buffer
circular_buf imu_circle_buf = {
    .buf = imu_cbuf,
    .write_idx = 0,
    .read_idx = 0
};

static uint16_t imu_count_idx[1]={0};
static short imu_count_buf[N_circular_buf]; // IMU count circular buffer
// IMU sample count circular buffer
circular_buf count_circle_buf = {
    .buf = imu_count_buf,
    .write_idx = 0,
    .read_idx = 0
};

bool is_empty(circular_buf *buf) {
    if (buf->read_idx == buf->write_idx) {
        return true;
    }
    else {
        return false;
    }
}

bool is_full(circular_buf *buf) {
    if ((buf->write_idx + 1) % N_circular_buf == buf->read_idx) {
        return true;
    }
    else {
        return false;
    }
}

int buf_length(circular_buf *buf) {
    // NRF_GPIOTE->INTENCLR |= 0x00000080;
    int temp = (buf->write_idx - buf->read_idx + N_circular_buf) % N_circular_buf;
    // NRF_GPIOTE->INTENSET |= 0x00000080;
    return temp;
}

int write_in_buf(circular_buf *buf, short *value, int L) {
    // L should be sizeof(value)/sizeof(short)
    if (L + buf_length(buf) > N_circular_buf-1) {
        printk("circular_buf out of range\n");
        return -1;
    }
    
    for (int i = 0;i < L;i++) {
        buf->buf[buf->write_idx] = value[i];
        buf->write_idx = (buf->write_idx + 1) % N_circular_buf;
    }
    return 0;
}

int read_out_buf(circular_buf *buf, short *value, uint16_t *buf_l) {
    uint16_t L = (uint16_t)buf_length(buf);
    buf_l[0] = L;
    for (int i=0;i<L;i++) {
        value[i] = buf->buf[buf->read_idx];
        buf->read_idx = (buf->read_idx + 1) % N_circular_buf;
    }
    // if (L > 0) {
    //     printk("buf_length=%d\n, write_idx=%d, read_idx=%d\n",L, buf->write_idx, buf->read_idx);
    // }
    return 0;
}
/////////////////////////////////////////////////////////////////////////// finish define circular buf


/// SD card data file
static struct fs_file_t mic_file, imu_file, imu_t_file, imu_count_file;

static short imu_rbuf[6]; // small read buf to get direct data each time IRQ
static short imu_buf[N_circular_buf]; // IMU buffer
static short imu_cw_buf[N_circular_buf]; // IMU count write buffer
static int32_t imu_t[1]; // timestamp for each IMU sample
static short *imu_p = &imu_buf[0]; // pointer to current buffer head
static uint16_t ICM_count = 0; // sample count within the buffer, be really careful with uint8_t and int8_t

static uint16_t ICM_cw=0;

/////// ICM thread
struct k_thread ICM_thread_data;
/*ICM processing thread*/
#define ICM_THREAD_STACK_SIZE 2048
/* ICM thread is a priority cooperative thread, less than PDM mic*/
#define ICM_THREAD_PRIORITY -16
K_THREAD_STACK_DEFINE(ICM_thread_stack_area, ICM_THREAD_STACK_SIZE);
K_SEM_DEFINE(ICM_thread_semaphore, 0, 1);
K_SEM_DEFINE(processing_thread_semaphore, 0, 1);
// K_SEM_DEFINE(ICM_write_semaphore, 1, 1);

static void ICM_thread_entry_point(void *p1, void *p2, void *p3) {
    while (true) {
        if (k_sem_take(&ICM_thread_semaphore, K_FOREVER) == 0) {
            
            NRF_P1->PIN_CNF[2] = 1;
            NRF_P1->OUTSET |= 1 << 2;

            ICM_readSensor();
            memcpy(imu_rbuf,&IMU_data[0],sizeof(IMU_data));   
            // imu_t[ICM_count] = k_cycle_get_32();

            // write data into circular buf
            write_in_buf(&imu_circle_buf,imu_rbuf,6);

            imu_count_idx[0]++;
            write_in_buf(&count_circle_buf, imu_count_idx,1);

            // ICM_count++;
            // imu_p = &imu_buf[6*ICM_count];

            // fs_write(&imu_file,&IMU_data[0],sizeof(IMU_data));    
            // k_sem_give(&ICM_write_semaphore);

            NRF_P1->OUTCLR |= 1 << 2;

            // printk("ICM th end\n");
    }
}
}

ISR_DIRECT_DECLARE(ICM_handler)
{   
    // NRF_P0->PIN_CNF[11] = 1;
    // NRF_P0->OUTSET |= 1 << 11;

    // https://github.com/gcmcnutt/HeadTracker/blob/4d5e3fb4793d519110b5812090c64401381d3b56/firmware/src/src/targets/nrf52/PPMIn.cpp#L191
    ISR_DIRECT_HEADER();
    // printk("EVENTS_IN[7]=%d, CONFIG[7].PSEL=0x%x\n",NRF_GPIOTE0->EVENTS_IN[7],NRF_GPIOTE0->CONFIG[7]);
    if (NRF_GPIOTE0->EVENTS_IN[7]) {
            k_sem_give(&ICM_thread_semaphore);
            NRF_GPIOTE0->EVENTS_IN[7] = 0;
        }

    // NRF_P0->OUTCLR |= 1 << 11;

    return 0;
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

// learn I2S thread from https://github.com/stuffmatic/microdsp-zephyr-demos/blob/497e0bb7fc23103827d3e283d5374ed00ea315c7/src/i2s.c#L63
/*Audio processing thread*/
#define PROCESSING_THREAD_STACK_SIZE 2048
/* Processing thread is a top priority cooperative thread */
#define PROCESSING_THREAD_PRIORITY -15
K_THREAD_STACK_DEFINE(processing_thread_stack_area, PROCESSING_THREAD_STACK_SIZE);
struct k_thread processing_thread_data;

uint32_t processing_sem_give_time = 0;

static void processing_thread_entry_point(void *p1, void *p2, void *p3) {
    while (true) {
        

        if (k_sem_take(&processing_thread_semaphore, K_USEC(1)) == 0) {
            
            if (!atomic_test_bit(&processing_in_progress, 0)) {
                printk("processing_in_progress is not set");
                __ASSERT(atomic_test_bit(&processing_in_progress, 0), "processing_in_progress is not set");
            }

            if (atomic_test_bit(&dropout_occurred, 0)) {
                printk("dropout_occurred\n");
                atomic_clear_bit(&dropout_occurred, 0);
            }

            

            // uint32_t processing_sem_take_time = k_cycle_get_32();
            // uint32_t cycles_spent = processing_sem_take_time - processing_sem_give_time;
            // uint32_t ns_spent = k_cyc_to_ns_ceil32(cycles_spent);
            // printk("processing thread start took %d ns\n", ns_spent); 

            

            

            nrfx_i2s_buffers_t* buffers_to_process = processing_buffers_1 ? &nrfx_i2s_buffers_1 : &nrfx_i2s_buffers_2;
            int32_t *rx = (int32_t *)buffers_to_process->p_rx_buffer;

            // printk("rx_buffer_len=%d\n", sizeof(rx_1)/sizeof(int32_t));

            
            ///////////////// somehow mic_data and IMU_data fs_write together can work

            ssize_t err;

            err = fs_write(&mic_file,&rx[0],AUDIO_BUFFER_BYTE_SIZE); 
            // if (err) {
            //     printk("mic_file fs_write error = %d\n", err);
            // }
            // printk("rx = %08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x\n", rx[0],rx[1],rx[2],rx[3],rx[4],
            //             rx[5],rx[6],rx[7],rx[8],rx[9],rx[10],rx[11],rx[12],rx[13],rx[14],rx[15]);

            /* Swap buffers */
            nrfx_err_t result = nrfx_i2s_next_buffers_set(buffers_to_process);
            if (result != NRFX_SUCCESS) {
                /* printk("nrfx_i2s_next_buffers_set failed with %d\n", result); */
                __ASSERT(result == NRFX_SUCCESS, "nrfx_i2s_next_buffers_set failed with result %d", result);
            }
            processing_buffers_1 = !processing_buffers_1;   



            
            imu_t[0] = k_cycle_get_32();
            fs_write(&imu_t_file,&imu_t,sizeof(imu_t));

            // we don't want to record IMU and change ICM_count during IMU data writing
            
            

            // NRFX_IRQ_DISABLE(GPIOTE0_IRQn);
            // NVIC_DisableIRQ(GPIOTE0_IRQn);
            // irq_disable(GPIOTE0_IRQn);
            // NRF_GPIOTE->INTENCLR |= 0x00000080;
                //read out circualr buf
                read_out_buf(&imu_circle_buf,imu_buf,&ICM_count);
                
                read_out_buf(&count_circle_buf, imu_cw_buf, &ICM_cw);

                // printk("ICM_count=%d, ICM_cw=%d\n",ICM_count,ICM_cw);

                err = fs_write(&imu_count_file, &imu_cw_buf,ICM_cw*sizeof(short));
                // if (err) {
                //     printk("imu_count_file fs_write error = %d\n", err);
                // }
                err = fs_write(&imu_file, &imu_buf,ICM_count*sizeof(short));
                // if (err) {
                //     printk("imu_file fs_write error = %d\n", err);
                // }

            // NRF_GPIOTE->INTENSET |= 0x00000080;
            // NVIC_EnableIRQ(GPIOTE0_IRQn);
            // irq_enable(GPIOTE0_IRQn);
            // NRFX_IRQ_ENABLE(GPIOTE0_IRQn);

            
        

            

            
            
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
    // fs_file_t_init(&mic_t_file);
    // printk("Opening mic_t_file path\n");
    // char mic_t_filename[30];
	// sprintf(&mic_t_filename, "/SD:/audio_t.dat"); 

    // // delete mic_t_file if exist
    // fs_unlink(mic_t_filename);

	// error = fs_open(&mic_t_file, mic_t_filename, FS_O_CREATE | FS_O_WRITE);
    // if (error) {
	// 		printk("Error opening mic_t_file [%03d]\n", error);
	// 		return 0;
	// 	}

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

    //////// imu_count_file for ICM count data
    fs_file_t_init(&imu_count_file);
    printk("Opening imu_count_file path\n");
    char imu_count_filename[30];
	sprintf(&imu_count_filename, "/SD:/imu_c.dat"); 

    // delete imu_count_file if exist
    fs_unlink(imu_count_filename);

	error = fs_open(&imu_count_file, imu_count_filename, FS_O_CREATE | FS_O_WRITE);
    if (error) {
			printk("Error opening imu_count_file [%03d]\n", error);
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
    
    // set ICM samping rate
    ICM_setSamplingRate(rate_idx);

    // enable interrupt
    ICM_enableINT();

    // GPIO configure interrupt
    // gpio_pin_interrupt_configure(GPIO_dev,ICM_INT_pin,GPIO_INT_EDGE_RISING);
    // gpio_init_callback(&ICM_INT_cb,ICM_INT_handler,BIT(ICM_INT_pin));

    // GPIOTE0 for secure
    IRQ_DIRECT_CONNECT(GPIOTE0_IRQn, 0, ICM_handler, 0);
    

    // try give semaphore as IMU interrupt
    // Event mode, P0.25 for interrupt, when rising edge
    // NRF_GPIOTE->CONFIG[0] = 0x00011901;
    // check NRF_GPIOTE->EVENTS_IN[0]
    nrf_gpiote_event_configure(NRF_GPIOTE0,7,25,GPIOTE_CONFIG_POLARITY_LoToHi);
    

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

    NRF_P1->PIN_CNF[2] = 1;
    NRF_P1->OUTSET |= 1 << 2;

    // wait for bluetooth command
    while (!current_conn) {
        printk("Wait for bluetooth connection..\n");

        NRF_P1->OUT ^= 1 << 2;

        k_msleep(1000);
    } 


    //////////////////////// START SAMPLE
    

    

    // enable GPIOTE
    NRF_GPIOTE0->EVENTS_IN[7] = 0;
    NRF_GPIOTE->INTENSET |= 0x00000080;
    // enable gpiote event (IMU INT)
    nrf_gpiote_event_enable(NRF_GPIOTE0, 7);
    // enable GPIOTE IRQ
    NRFX_IRQ_ENABLE(GPIOTE0_IRQn);
    NVIC_EnableIRQ(GPIOTE0_IRQn);
    irq_enable(GPIOTE0_IRQn);

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
    printk("I2S streams started, error=%d\n", error);
    
    


    ///////////////////////////////////////// loop
    int while_count = 1;
    int while_end = 600000;

    while (current_conn){
        // printk("While loop %d\n", while_count);
        // printk("ICM_count = %d\n", ICM_count);

        // k_sem_give(&ICM_thread_semaphore);
        // k_usleep(800); // not real IMU sampling rate
        
        // if (while_count >= while_end) {
        //     break;
        // }
        // while_count++;

        // if (NRF_GPIOTE->EVENTS_IN[7]) {
        //     k_sem_give(&ICM_thread_semaphore);
        //     NRF_GPIOTE->EVENTS_IN[7] = 0;
        // }
        k_msleep(1000);
    }
    
    ///////////////////// after loop

    nrf_i2s_int_disable(NRF_I2S0, NRF_I2S_INT_RXPTRUPD_MASK |
                                  NRF_I2S_INT_TXPTRUPD_MASK);
    k_thread_abort(ICM_thread_tid);
    NRF_GPIOTE->INTENCLR |= 0x00000080;
    nrf_gpiote_event_disable(NRF_GPIOTE0, 7);
    NRFX_IRQ_DISABLE(GPIOTE0_IRQn);
    NVIC_DisableIRQ(GPIOTE0_IRQn);
    irq_disable(GPIOTE0_IRQn);
    

    k_msleep(1000);

	printk("Stream stopped\n");

	printk("mic_file named \"audio001.dat\" successfully created\n");		
	fs_close(&mic_file);
    // fs_close(&mic_t_file);
    fs_close(&imu_file);
    fs_close(&imu_t_file);
    fs_close(&imu_count_file);

    k_msleep(1000);
    lsdir(disk_mount_pt);
}
