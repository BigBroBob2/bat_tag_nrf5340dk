/*Main loop goes here*/

#include "icm42688.h"
#include "mpu9265.h"
#include "pdm_microphone.h"

// #ifndef USB_UART_H
// #define USB_UART_H 1
// #include "usb_uart.h"
// #endif

#ifndef BT_H
#define BT_H 1
#include "bt.h"
#endif

#ifndef SDCARD_H
#define SDCARD_H 1
#include "SDcard.h"
#endif

#include <nrfx_log.h>

// Usual clock_rate
const int I2C_ClockRate =  100e3;

// Usual timeout
// use int64_t k_uptime_get() to returns the elapsed time since the system booted, in milliseconds
const int I2C_BUS_TIMEOUT_us = 200 / (I2C_ClockRate / 400e3);



void main(void)
{   
    int error;

    // range scale
    double MPU_GyroScale = MPU_ADC2Float(MPU_GyroRangeVal_dps(MPU_GyroRange_idx));
    double MPU_AccelScale = MPU_ADC2Float(MPU_AccelRangeVal_G(MPU_AccelRange_idx));
    double MPU_Samplerate = MPU_SampRate(MPU_DataRate_idx);

    double ICM_GyroScale =  ICM_ADC2Float(ICM_GyroRangeVal_dps(ICM_GyroRange_idx));
    double ICM_AccelScale = ICM_ADC2Float(ICM_AccelRangeVal_G(ICM_AccelRange_idx));
    double ICM_Samplerate = ICM_SampRate(ICM_DataRate_idx);


    //////////////////////////////////////// USB
    // need to enable USB
    // error = usb_enable(NULL);

    // Set up our UART DMA callback function (this is a Zephyr thing).
	
    // uart_callback_set(COM12, UART_Callback, NULL);

    ////////////////////////////////////////////  I2C devices
    printk("The I2C scanner started\n");
    const struct device *i2c_dev1,*i2c_dev2;

    

    // check I2C binding
    i2c_dev1 = device_get_binding("I2C_1");
    if (!i2c_dev1) {
        printk("I2C_1 Binding failed.");
        return;
    }

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
    error = i2c_configure(i2c_dev1, I2C_SPEED_SET(I2C_ClockRate));
    if (error != 0){printk("MPU i2c_configure failed\n");}
    error = i2c_configure(i2c_dev2, I2C_SPEED_SET(I2C_ClockRate));
    if (error != 0){printk("ICM i2c_configure failed\n");}
    // check WhoAmI
    printk("WhoAmI: 0x%02x (MPU should be 0x71)\n",MPU_ReadByte(i2c_dev1,MPU_WHO_AM_I,MPU_ADDR));
    printk("WhoAmI: 0x%02x (ICM should be 0x47)\n",ICM_ReadByte(i2c_dev2,ICM_WHO_AM_I,ICM_ADDR));



    printk("Value of NRF_TWIM1->PSEL.SCL : %d \n",NRF_TWIM1->PSEL.SCL);
    printk("Value of NRF_TWIM1->PSEL.SDA : %d \n",NRF_TWIM1->PSEL.SDA);
    printk("Value of NRF_TWIM1->FREQUENCY: %d \n",NRF_TWIM1->FREQUENCY);
    printk("Value of NRF_TWIM2->PSEL.SCL : %d \n",NRF_TWIM2->PSEL.SCL);
    printk("Value of NRF_TWIM2->PSEL.SDA : %d \n",NRF_TWIM2->PSEL.SDA);
    printk("Value of NRF_TWIM2->FREQUENCY: %d \n",NRF_TWIM2->FREQUENCY);
    // printk("26738688 -> 100k\n");

    // searching I2C address
    for (uint8_t i = 4; i <= 0x7F; i++) {
        struct i2c_msg msgs[1];
        uint8_t dst = 1;

        msgs[0].buf = &dst;
        msgs[0].len = 1U;
        msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

        error = i2c_transfer(i2c_dev1, &msgs[0], 1, i);
        if (error == 0) {
            printk("I2C addr 0x%2x FOUND (MPU9265 should be 0x%2x, ICM42688 should be 0x%2x)\n", i,MPU_ADDR,ICM_ADDR);
        }
        error = i2c_transfer(i2c_dev2, &msgs[0], 1, i);
        if (error == 0) {
            printk("I2C addr 0x%2x FOUND (MPU9265 should be 0x%2x, ICM42688 should be 0x%2x)\n", i,MPU_ADDR,ICM_ADDR);
        }
    }

    //////////////////////////////////////////////// try dmic sample
    // https://github.com/zephyrproject-rtos/zephyr/blob/main/samples/drivers/audio/dmic/src/main.c

    const struct device *const dmic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));

    if (!device_is_ready(dmic_dev)) {
		printk("%s is not ready\n", dmic_dev->name);
		return 0;
	}
    if (!configure_streams(dmic_dev)) {
        printk("Failed to config streams\n", dmic_dev->name);
		return 0;
    }

    // struct pcm_stream_cfg stream = {
	// 	.pcm_width = SAMPLE_BIT_WIDTH,
	// 	.mem_slab  = &mem_slab,
	// };
	// struct dmic_cfg cfg = {
	// 	.io = {
	// 		/* These fields can be used to limit the PDM clock
	// 		 * configurations that the driver is allowed to use
	// 		 * to those supported by the microphone.
	// 		 */
	// 		.min_pdm_clk_freq = 1000000,
	// 		.max_pdm_clk_freq = 3500000,
	// 		.min_pdm_clk_dc   = 40,
	// 		.max_pdm_clk_dc   = 60,
	// 	},
	// 	.streams = &stream,
	// 	.channel = {
	// 		.req_num_streams = 1,
	// 	},
	// };

    // cfg.channel.req_num_chan = 1;
	// cfg.channel.req_chan_map_lo =
	// 	dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
	// cfg.streams[0].pcm_rate = MAX_SAMPLE_RATE;
    
	// cfg.streams[0].block_size =
	// 	BLOCK_SIZE(cfg.streams[0].pcm_rate, cfg.channel.req_num_chan);

	// error = do_pdm_transfer(dmic_dev, &cfg, 2 * BLOCK_COUNT);
	// if (error < 0) {
    //     printk("do_pdm_transfer error=%d\n",error);
	// 	return 0;
	// }

    //////////////////////////////////////////////// Data
    uint8_t sensor_config[1];
    short sensor_data[6];
    // short offset[6];

    // ICM need init
    ICM_Init(i2c_dev2,ICM_ADDR);


    // SD card
	// struct fs_file_t file;

    // setup_disk();

    // fs_file_t_init(&file);
    // printk("Opening file path\n");
    // char filename[30];
	// sprintf(&filename, "/SD:/audio001.txt"); 
	// error = fs_open(&file, filename, FS_O_CREATE | FS_O_WRITE);
    // if (error) {
	// 		printk("Error opening file [%03d]\n", error);
	// 		return 0;
	// 	}

    // printk("Streams started\n");
    // if (!trigger_command(dmic_dev, DMIC_TRIGGER_START)) {
	// 		return 0;
	// 	}

    //////////////////////////////////////////////// bluetooth uart 
    //// init button
    error = dk_buttons_init(button_changed);
	if (error) {
		printk("Cannot init buttons (error: %d)\n", error);
	}

    //// init led
    error = dk_leds_init();
	if (error) {
		printk("Cannot init LEDs (error: %d)\n", error);
	}

    // error = bt_uart_init();
    // if (error) {
	// 	led_show_error();
	// }

    // error = bt_conn_auth_cb_register(&conn_auth_callbacks);
	// if (error) {
	// 	printk("Failed to register authorization callbacks.\n");
	// 	return 0;
	// }

	// error = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
	// if (error) {
	// 	printk("Failed to register authorization info callbacks.\n");
	// 	return 0;
	// }

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

    // loop
    int while_count = 0;
    int while_end = 10000;

    int blink_status = 0;
    char addr[BT_ADDR_LE_STR_LEN];
    while (1){
       
        // read sensor output
        ICM_ReadSensor(i2c_dev2,sensor_data,ICM_ADDR);
        printk("Data from ICM42688: sensor_config:%6d,acc_x:%6d,acc_y:%6d,acc_z:%6d,gyro_x:%6d,gyro_y:%6d,gyro_z:%6d\n",
        sensor_config[0],sensor_data[0],sensor_data[1],sensor_data[2],sensor_data[3],sensor_data[4],sensor_data[5]);

        if (current_conn) {
            uint8_t data[2];
            data[0] = (uint8_t) (sensor_data[0] & 0xff);
            data[1] = (uint8_t) (sensor_data[0] >> 8);
                      
            
            error = bt_nus_send(NULL, data, 2);
		    if (error) {
			    printk("Failed to send data:%d,%d over BLE connection, error=%d (enable notification if -128)\n",data[0],data[1],error);
		    }
            else {
                printk("BLE send data success!\n");
            }

        }

        dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));

        /* check sensor_config is on*/
        // check register bank from 0 (default) to 1, then back to 0
        // ICM_ReadSensorConfig(i2c_dev2,sensor_config,ICM_ADDR);

        /* output sensor data*/        
        // printk("Data from MPU9265: acc_x:%6d,acc_y:%6d,acc_z:%6d,gyro_x:%6d,gyro_y:%6d,gyro_z:%6d\n",
        // sensor_data[0],sensor_data[1],sensor_data[2],sensor_data[3],sensor_data[4],sensor_data[5]);
        // printk("Data from MPU9265: acc_x:%9.3f,acc_y:%9.3f,acc_z:%9.3f,gyro_x:%9.3f,gyro_y:%9.3f,gyro_z:%9.3f\n",
        // sensor_data[0]*AccelScale,sensor_data[1]*AccelScale,sensor_data[2]*AccelScale,
        // sensor_data[3]*GyroScale,sensor_data[4]*GyroScale,sensor_data[5]*GyroScale);

        // printk("Data from ICM42688: sensor_config:%6d,acc_x:%6d,acc_y:%6d,acc_z:%6d,gyro_x:%6d,gyro_y:%6d,gyro_z:%6d\n",
        // sensor_config[0],sensor_data[0],sensor_data[1],sensor_data[2],sensor_data[3],sensor_data[4],sensor_data[5]);
        // printk("Data from ICM42688: acc_x:%9.3f,acc_y:%9.3f,acc_z:%9.3f,gyro_x:%9.3f,gyro_y:%9.3f,gyro_z:%9.3f\n",
        // sensor_data[0]*AccelScale,sensor_data[1]*AccelScale,sensor_data[2]*AccelScale,
        // sensor_data[3]*GyroScale,sensor_data[4]*GyroScale,sensor_data[5]*GyroScale);

    // error = do_pdm_transfer(dmic_dev, &cfg, 2 * BLOCK_COUNT);
	// if (error < 0) {
    //     printk("do_pdm_transfer error=%d\n",error);
	// 	return 0;
	// }

    /////////////////////////
    //   PDM microphone    //
    /////////////////////////

    // void *mem_block;
	// size_t block_size;

	// error = dmic_read(dmic_dev, 0, &mem_block, &block_size, READ_TIMEOUT);
	// if (error < 0) {
	// 	printk("Failed to read dmic data and write block: %d\n", error);
	// 	return false;
	// }

	// error = fs_write(&file, mem_block, block_size);
	// if (error < 0) {
	// 	printk("Failed to write data in SD card: %d\n", error);
	// 	break;
	// }

    // k_mem_slab_free(&mem_slab,&mem_block);
    // // printk("k_mem_slab_num_free_get=%d\n",k_mem_slab_num_free_get(&mem_slab));
    
    // if (while_count == while_end) {
    //     if (!trigger_command(dmic_dev, DMIC_TRIGGER_STOP)) {
	// 		return 0;
	// 	}
	// 	printk("Stream stopped\n");

	// 	printk("File named \"audio001.txt\" successfully created\n");		
	// 	fs_close(&file);
    //     lsdir(disk_mount_pt);
    //     break;
    // }
    // while_count++;



    }
}
