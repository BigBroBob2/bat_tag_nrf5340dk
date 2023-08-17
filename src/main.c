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



static struct fs_file_t mic_file;

void main(void)
{   
    int error;

    ////////////////////////////////////////////  I2C devices IMU

    // range scale
    double ICM_GyroScale =  ICM_ADC2Float(ICM_GyroRangeVal_dps(ICM_GyroRange_idx));
    double ICM_AccelScale = ICM_ADC2Float(ICM_AccelRangeVal_G(ICM_AccelRange_idx));
    double ICM_Samplerate = ICM_SampRate(ICM_DataRate_idx);

    printk("The I2C scanner started\n");
    const struct device *i2c_dev2;

    // check I2C binding
    i2c_dev2 = device_get_binding("I2C_2");
    if (!i2c_dev2) {
        printk("I2C_2 Binding failed.");
        return;
    }

    /* Demonstration of runtime configuration */

    /*ISSUE with configurating multiple sensors on the same I2C bus:
    https://devzone.nordicsemi.com/f/nordic-q-a/71837/configuring-two-sensor-on-same-i2c-bus-in-zephyr-rtos
    */

    // set ClockRate
    error = i2c_configure(i2c_dev2, I2C_SPEED_SET(I2C_ClockRate));
    if (error != 0){printk("ICM i2c_configure failed\n");}
    // check WhoAmI
    printk("WhoAmI: 0x%02x (ICM should be 0x47)\n",ICM_ReadByte(i2c_dev2,ICM_WHO_AM_I,ICM_ADDR));

    // searching I2C address
    for (uint8_t i = 4; i <= 0x7F; i++) {
        struct i2c_msg msgs[1];
        uint8_t dst = 1;

        msgs[0].buf = &dst;
        msgs[0].len = 1U;
        msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

        error = i2c_transfer(i2c_dev2, &msgs[0], 1, i);
        if (error == 0) {
            printk("I2C addr 0x%2x FOUND (ICM42688 should be 0x%2x)\n", i,ICM_ADDR);
        }
    }

    // ICM need init
    ICM_Init(i2c_dev2,ICM_ADDR);

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

    if (error=i2s_trigger(i2s_rx_dev,I2S_DIR_RX,I2S_TRIGGER_START)) {
        printk("Failed to trigger i2s start, error=%d\n", error);
		return 0;
    }
    
    printk("I2S streams started\n");

    //////////////////////////// SD card
	
    setup_disk();

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

    //////////////////////////////////////////////// Data
    static uint8_t sensor_config[1];
    static short sensor_data[6];

    // loop
    int while_count = 1;
    int while_end = 500;

    // led status
    int blink_status = 0;

    // ticks for sampling
    int32_t time_between_samples = 32;
    int32_t next_tick;
    while (1){
        printk("While loop %d\n", while_count);

        // ICM_ReadSensor(i2c_dev2,sensor_data,ICM_ADDR);
        if (while_count%100 == 0) {
        
        printk("Time %f sec: sensor_config:%6d,acc_x:%6d,acc_y:%6d,acc_z:%6d,gyro_x:%6d,gyro_y:%6d,gyro_z:%6d\n",
        1.0*sys_clock_tick_get_32()/32768,
        sensor_config[0],sensor_data[0],sensor_data[1],sensor_data[2],sensor_data[3],sensor_data[4],sensor_data[5]);
        }

        /////////// Try to read IMU data at 1024 Hz, 2^16 = 32768 cycles = 1 second
        // How to get sensor data from multiple sensors at different sampling rate?
       

        // if (while_count == 1) {
        // // read sensor output
        // ICM_ReadSensor(i2c_dev2,sensor_data,ICM_ADDR);
        // printk("Data from ICM42688: sensor_config:%6d,acc_x:%6d,acc_y:%6d,acc_z:%6d,gyro_x:%6d,gyro_y:%6d,gyro_z:%6d\n",
        // sensor_config[0],sensor_data[0],sensor_data[1],sensor_data[2],sensor_data[3],sensor_data[4],sensor_data[5]);

        // ///////// bluetooth send data

        // if (current_conn) {
        //     char data[128];
        //     snprintf(data,128,"Data from ICM42688: sensor_config:%6d,acc_x:%6d,acc_y:%6d,acc_z:%6d,gyro_x:%6d,gyro_y:%6d,gyro_z:%6d",
        // sensor_config[0],sensor_data[0],sensor_data[1],sensor_data[2],sensor_data[3],sensor_data[4],sensor_data[5]);
                      
            
        //     error = bt_nus_send(NULL, data,128);
		//     if (error) {
		// 	    printk("Failed to send data:%d,%d over BLE connection, error=%d (enable notification if -128)\n",data[0],data[1],error);
		//     }
        //     else {
        //         printk("BLE send data success!\n");
        //     }

        // }

        // dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);

        // next_tick = sys_clock_tick_get_32()+time_between_samples;
        // }
        // else if (sys_clock_tick_get_32() >= next_tick) {
        //     // read sensor output
        // ICM_ReadSensor(i2c_dev2,sensor_data,ICM_ADDR);
        // printk("Data from ICM42688: sensor_config:%6d,acc_x:%6d,acc_y:%6d,acc_z:%6d,gyro_x:%6d,gyro_y:%6d,gyro_z:%6d\n",
        // sensor_config[0],sensor_data[0],sensor_data[1],sensor_data[2],sensor_data[3],sensor_data[4],sensor_data[5]);

        // ///////// bluetooth send data

        // if (current_conn) {
        //     char data[128];
        //     snprintf(data,128,"Data from ICM42688: sensor_config:%6d,acc_x:%6d,acc_y:%6d,acc_z:%6d,gyro_x:%6d,gyro_y:%6d,gyro_z:%6d",
        // sensor_config[0],sensor_data[0],sensor_data[1],sensor_data[2],sensor_data[3],sensor_data[4],sensor_data[5]);
                      
            
        //     error = bt_nus_send(NULL, data,128);
		//     if (error) {
		// 	    printk("Failed to send data:%d,%d over BLE connection, error=%d (enable notification if -128)\n",data[0],data[1],error);
		//     }
        //     else {
        //         printk("BLE send data success!\n");
        //     }

        // }

        // dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);

        // next_tick = next_tick+time_between_samples;
        // }



    /////////////////////////
    //   PDM microphone    //
    /////////////////////////
    int tic = k_cycle_get_32();
    // printk("Time %d, start i2s_read\n",tic);

    void *mem_block;
	size_t block_size;
  
    error = i2s_read(i2s_rx_dev,&mem_block,&block_size);
	if (error < 0) {
		printk("Failed to read dmic data and write block: %d\n", error);
		// return false;
	}

	error = fs_write(&mic_file, mem_block, block_size);
	if (error < 0) {
		printk("Failed to write data in SD card: %d\n", error);
		break;
	}

    k_mem_slab_free(&mem_slab,&mem_block);
    // printk("k_mem_slab_num_free_get=%d\n",k_mem_slab_num_free_get(&mem_slab));
    
    int toc = k_cycle_get_32();
    printk("finish i2s_read, elapse %d\n",toc-tic);
    
    if (while_count == while_end) {
        if (i2s_trigger(i2s_rx_dev,I2S_DIR_RX,I2S_TRIGGER_STOP)) {
        printk("Failed to trigger i2s stop\n");
		return 0;
        }

		printk("Stream stopped\n");

		printk("mic_file named \"audio001.dat\" successfully created\n");		
		fs_close(&mic_file);
        lsdir(disk_mount_pt);
        break;
    }
    while_count++;
    }
}
