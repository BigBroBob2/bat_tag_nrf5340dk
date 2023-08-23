/*IMU: ICM42688*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include <zephyr/drivers/spi.h>

//register map
// Bank 0
#define ICM_ACCEL_DATA_X1  31
#define ICM_ACCEL_DATA_Y1  33
#define ICM_ACCEL_DATA_Z1  35
#define ICM_GYRO_DATA_X1   37
#define ICM_GYRO_DATA_Y1   39
#define ICM_GYRO_DATA_Z1   41

#define ICM_PWR_MGMT0      78

#define ICM_SELF_TEST_CONFIG  112
#define ICM_WHO_AM_I       117
#define ICM_REG_BANK_SEL   118

// Bank 1
#define ICM_SENSOR_CONFIG  3

#define ICM_XG_ST_DATA 95
#define ICM_YG_ST_DATA 96
#define ICM_ZG_ST_DATA 97

// Bank 2
#define ICM_XA_ST_DATA 59
#define ICM_YA_ST_DATA 60
#define ICM_ZA_ST_DATA 61



// full-scale range of accel and gyro sensors
const int ICM_AccelRange_idx = 3;   //choose 0=32G, 1=16g, 2=8g, 3=4g, 4=2g
const int  ICM_GyroRange_idx = 1;   //choose 0=2000dps, 1=1000dps, 2=500dps, 3=250dps, 4=125dps

// Usual DataRate
const int ICM_DataRate_idx = 15;

// Return the gyro full-scale range, in Degrees per Second,
// for each of the range selection settings.
double ICM_GyroRangeVal_dps(int range_idx)
{
  switch (range_idx) {
    case 0: return 2000; break;
    case 1: return 1000; break;
    case 2: return  500; break;
    case 3: return  250; break;
    case 4: return  125; break;
    case 5: return   62.5; break;
    case 6: return  31.25; break;
    case 7: return  15.625; break;
    default: return 0; break;
  }
}

// Return the accelerometer full-scale range, in G,
// for each of the range selection settings.
// Note that the DATASHEET IS WRONG!!!
// The max range is 32G, NOT 16G!!!
//
double ICM_AccelRangeVal_G(int range_idx)
{
  switch(range_idx) {
//    case 0: return 16;   // MANUAL IS WRONG!!! Off by factor of 2.
//    case 1: return 8;
//    case 2: return 4;
//    case 3: return 2;

    case 0: return 32;     // Correct mapping.
    case 1: return 16;
    case 2: return 8;
    case 3: return 4;
    case 4: return 2;      // Yes, it does go down to +/-2G, and even lower for higher index.
    default: return 0;
  }
}

// Return DataRate 
double ICM_SampRate(int rate_idx)
{
  switch(rate_idx) {
    case 1: return 32000;
    case 2: return 16000;
    case 3: return  8000;
    case 4: return  4000;
    case 5: return  2000;
    case 6: return  1000;
    case 7: return   200;
    case 8: return   100;
    case 9: return    50;
    case 10: return   25;
    case 11: return   12.5;
    case 12: return    6.25;
    case 13: return    3.125;
    case 14: return    1.5625;
    case 15: return  500;
    default: return 0;
  }
}

// Convert integer ADC values to floating point scaled values.
double ICM_ADC2Float(double val) {
    return val / 32768.0;
}

/////////////////////////////////////////////////////////////////////////////
// config SPI

#define DELAY_SPI_CS_ACTIVE_US 2

#define ICM_SPI_WRITE 0b00000000
#define ICM_SPI_READ  0b10000000

static struct spi_cs_control cs_control;
static struct spi_config cfg={0};
const struct device *spi_dev2=DEVICE_DT_GET(DT_NODELABEL(spi_dev2));

int ICM_SPI_config() {
  cfg.frequency = 8000000U; // only spi4 has 24000000
    cfg.operation = SPI_WORD_SET(8);

    cs_control.gpio.port = DEVICE_DT_GET(DT_GPIO_CTLR(DT_NODELABEL(spi_dev2),cs_gpios));
    if (!cs_control.gpio.port) {
        printk("cannot find CS GPIO device\n");
	}
    cs_control.gpio.pin = DT_GPIO_PIN(DT_NODELABEL(spi_dev2), cs_gpios);
	cs_control.gpio.dt_flags = DT_GPIO_FLAGS(DT_NODELABEL(spi_dev2), cs_gpios);
	cs_control.delay = DELAY_SPI_CS_ACTIVE_US;

	cfg.cs = &cs_control;

  return 0;
}

int ICM_enableSensor() {
  uint8_t tx_buf[2] = {0};
    struct spi_buf spi_tx_buf = {
            .buf = tx_buf,
            .len = sizeof(tx_buf)
        };
    struct spi_buf_set tx_set = {
		.buffers = &spi_tx_buf,
		.count = 1
	};
    
    
    tx_buf[0] = ICM_SPI_WRITE | ICM_PWR_MGMT0;
    // enable accel and gyro
    tx_buf[1] = 0b00001111;
    if (spi_write(spi_dev2,&cfg,&tx_set)) {
        printk("spi_write failed\n");
    }
  return 0;
}

short IMU_data[6];

int ICM_readSensor() {
  uint8_t addr_buf[1] = {0};
    struct spi_buf spi_addr_buf = {
            .buf = addr_buf,
            .len = sizeof(addr_buf)
        };
    struct spi_buf_set addr_set = {
		.buffers = &spi_addr_buf,
		.count = 1
	};

    uint8_t rx_buf[13] = {0};
    struct spi_buf spi_rx_buf = {
            .buf = rx_buf,
            .len = sizeof(rx_buf)
        };
    struct spi_buf_set rx_set = {
		.buffers = &spi_rx_buf,
		.count = 1
	};

    addr_buf[0] = ICM_SPI_READ | ICM_ACCEL_DATA_X1;
    if (spi_transceive(spi_dev2,&cfg,&addr_set,&rx_set)) {
        printk("spi_transceive failed\n");
    }

    IMU_data[0] = (short)((rx_buf[1] << 8) | rx_buf[2]);
    IMU_data[1] = (short)((rx_buf[3] << 8) | rx_buf[4]);
    IMU_data[2] = (short)((rx_buf[5] << 8) | rx_buf[6]);
    IMU_data[3] = (short)((rx_buf[7] << 8) | rx_buf[8]);
    IMU_data[4] = (short)((rx_buf[9] << 8) | rx_buf[10]);
    IMU_data[5] = (short)((rx_buf[11] << 8) | rx_buf[12]);

    return 0;
}



