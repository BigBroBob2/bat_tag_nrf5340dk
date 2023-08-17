/*IMU: ICM42688*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>


#include <zephyr/drivers/i2c.h>

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

// I2C addr
#define ICM_ADDR 0x69

// Usual clock_rate
const int I2C_ClockRate =  100e3;

// Usual timeout
// use int64_t k_uptime_get() to returns the elapsed time since the system booted, in milliseconds
const int I2C_BUS_TIMEOUT_us = 200 / (I2C_ClockRate / 400e3);

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



uint8_t ICM_ReadByte(const struct device *dev, int Reg_addr, int I2C_addr) {
    uint8_t data[1];
    uint8_t res[1];
    int err_write,err_read;
    struct i2c_msg msg;

    data[0] = Reg_addr;
    
    msg.buf = data;
	msg.len = 1;
	msg.flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	err_write = i2c_transfer(dev, &msg, 1, I2C_addr);
    if (err_write != 0){
        printk("i2c_write failed\n");
        return 0;
        }

    msg.buf = res;
	msg.len = 1;
	msg.flags = I2C_MSG_READ | I2C_MSG_STOP;

    err_read = i2c_transfer(dev, &msg, 1, I2C_addr);
    if (err_read != 0){
        printk("i2c_read failed\n");
        return 0;
        }
    
    return res[0];
}

short ICM_ReadShort(const struct device *dev, int Reg_addr, int I2C_addr) {
    uint8_t data[1];
    uint8_t res[2];
    int err_write,err_read;
    struct i2c_msg msg;

    data[0] = Reg_addr;
    
    
    msg.buf = data;
	msg.len = 1;
	msg.flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	err_write = i2c_transfer(dev, &msg, 1, I2C_addr);
    if (err_write != 0){
        printk("i2c_write failed\n");
        return 0;
        }

    msg.buf = res;
	msg.len = 2;
	msg.flags = I2C_MSG_READ | I2C_MSG_STOP;

    err_read = i2c_transfer(dev, &msg, 1, I2C_addr);
    if (err_read != 0){
        printk("i2c_read failed\n");
        return 0;
        }
    
    return (short)((res[0] << 8) | res[1]);
}

//ICM-42688 need init
int ICM_Init(const struct device *dev, int I2C_addr){
/* Init ICM42688*/

    uint8_t data[2];
    int err_write,err_read;
    struct i2c_msg msg;

    //turn on accel and gyro to Low_Noise mode
    data[0] = ICM_PWR_MGMT0;
    data[1] = 0x0F;
    
    msg.buf = data;
	msg.len = 2;
	msg.flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	err_write = i2c_transfer(dev, &msg, 1, I2C_addr);
    if (err_write != 0){
        printk("i2c_write failed\n");
        return 0;
        }
}

int ICM_ReadSensor(const struct device *dev, short *res, int I2C_addr){
    /* Read 12 bytes together, then get accel and gyro sensor data 
    */
    uint8_t data[1],buf[12];
    int err_write,err_read;
    struct i2c_msg msg;

    data[0] = ICM_ACCEL_DATA_X1;
    
    msg.buf = data;
	msg.len = 1;
	msg.flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	err_write = i2c_transfer(dev, &msg, 1, I2C_addr);
    if (err_write != 0){
        printk("i2c_write failed\n");
        return 0;
        }

    msg.buf = buf;
	msg.len = 12;
	msg.flags = I2C_MSG_READ | I2C_MSG_STOP;

    err_read = i2c_transfer(dev, &msg, 1, I2C_addr);
    if (err_read != 0){
        printk("i2c_read failed\n");
        return 0;
        }
    
    res[0] = (short)((buf[0] << 8) | buf[1]);
    res[1] = (short)((buf[2] << 8) | buf[3]);
    res[2] = (short)((buf[4] << 8) | buf[5]);
    res[3] = (short)((buf[6] << 8) | buf[7]);
    res[4] = (short)((buf[8] << 8) | buf[9]);
    res[5] = (short)((buf[10] << 8) | buf[11]);
    

    return 1;
}

int ICM_ReadSensorConfig(const struct device *dev, uint8_t *res, int I2C_addr){
    /* Read sensor_config from ICM_42688
    */
    uint8_t data[2];
    int err_write,err_read;
    struct i2c_msg msg;

    //change register bank from 0 to 1
    data[0] = ICM_REG_BANK_SEL;
    data[1] = 0x01;
    
    msg.buf = data;
	msg.len = 2;
	msg.flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	err_write = i2c_transfer(dev, &msg, 1, I2C_addr);
    if (err_write != 0){
        printk("i2c_write failed\n");
        return 0;
        }

    // read sensor_config
    //change register bank from 0 to 1
    data[0] = ICM_SENSOR_CONFIG;
    
    msg.buf = data;
	msg.len = 1;
	msg.flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	err_write = i2c_transfer(dev, &msg, 1, I2C_addr);
    if (err_write != 0){
        printk("i2c_write failed\n");
        return 0;
        }

    msg.buf = res;
	msg.len = 1;
	msg.flags = I2C_MSG_READ | I2C_MSG_STOP;

    err_read = i2c_transfer(dev, &msg, 1, I2C_addr);
    if (err_read != 0){
        printk("i2c_read failed\n");
        return 0;
        }

    //change register bank from 1 back to 0
    data[0] = ICM_REG_BANK_SEL;
    data[1] = 0x00;
    
    msg.buf = data;
	msg.len = 2;
	msg.flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	err_write = i2c_transfer(dev, &msg, 1, I2C_addr);
    if (err_write != 0){
        printk("i2c_write failed\n");
        return 0;
        }

    return 1;
}

int ICM_EnableSelfTest(const struct device *dev, int I2C_addr){
    /*self-test*/
    uint8_t data[2];
    int err_write,err_read;
    struct i2c_msg msg;

    // enable self-test
    data[0] = ICM_SELF_TEST_CONFIG;
    data[1] = 0xFF;
    
    msg.buf = data;
	msg.len = 2;
	msg.flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	err_write = i2c_transfer(dev, &msg, 1, I2C_addr);
    if (err_write != 0){
        printk("i2c_write failed\n");
        return 0;
        }
}

int ICM_DisableSelfTest(const struct device *dev, int I2C_addr){
    /*self-test*/
    uint8_t data[2];
    int err_write,err_read;
    struct i2c_msg msg;

    // enable self-test
    data[0] = ICM_SELF_TEST_CONFIG;
    data[1] = 0x00;
    
    msg.buf = data;
	msg.len = 2;
	msg.flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	err_write = i2c_transfer(dev, &msg, 1, I2C_addr);
    if (err_write != 0){
        printk("i2c_write failed\n");
        return 0;
        }
}

int ICM_ReadSelfTestData(const struct device *dev, uint8_t *res, int I2C_addr){
    /* Read self_test data
    */
    uint8_t data[2],A_data[3],G_data[3];
    int err_write,err_read;
    struct i2c_msg msg;

    /////////////////////////// read gyro self_test data on bank 1
    //change register bank from 0 to 1
    data[0] = ICM_REG_BANK_SEL;
    data[1] = 0x01;
    
    msg.buf = data;
	msg.len = 2;
	msg.flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	err_write = i2c_transfer(dev, &msg, 1, I2C_addr);
    if (err_write != 0){
        printk("i2c_write failed\n");
        return 0;
        }

    // read gyro self_test data
    data[0] = ICM_XG_ST_DATA;
    
    msg.buf = data;
	msg.len = 1;
	msg.flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	err_write = i2c_transfer(dev, &msg, 1, I2C_addr);
    if (err_write != 0){
        printk("i2c_write failed\n");
        return 0;
        }

    msg.buf = G_data;
	msg.len = 3;
	msg.flags = I2C_MSG_READ | I2C_MSG_STOP;

    err_read = i2c_transfer(dev, &msg, 1, I2C_addr);
    if (err_read != 0){
        printk("i2c_read failed\n");
        return 0;
        }

    /////////////////////////// read accel self_test data on bank 2
    //change register bank from 1 to 2
    data[0] = ICM_REG_BANK_SEL;
    data[1] = 0x02;
    
    msg.buf = data;
	msg.len = 2;
	msg.flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	err_write = i2c_transfer(dev, &msg, 1, I2C_addr);
    if (err_write != 0){
        printk("i2c_write failed\n");
        return 0;
        }

    // read accel self_test data
    data[0] = ICM_XA_ST_DATA;
    
    msg.buf = data;
	msg.len = 1;
	msg.flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	err_write = i2c_transfer(dev, &msg, 1, I2C_addr);
    if (err_write != 0){
        printk("i2c_write failed\n");
        return 0;
        }

    msg.buf = A_data;
	msg.len = 3;
	msg.flags = I2C_MSG_READ | I2C_MSG_STOP;

    err_read = i2c_transfer(dev, &msg, 1, I2C_addr);
    if (err_read != 0){
        printk("i2c_read failed\n");
        return 0;
        }

    ////////////////////////////////change register bank from 1 back to 0
    data[0] = ICM_REG_BANK_SEL;
    data[1] = 0x00;
    
    msg.buf = data;
	msg.len = 2;
	msg.flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	err_write = i2c_transfer(dev, &msg, 1, I2C_addr);
    if (err_write != 0){
        printk("i2c_write failed\n");
        return 0;
        }

    res[0] = A_data[0];
    res[1] = A_data[1];
    res[2] = A_data[2];
    res[3] = G_data[0];
    res[4] = G_data[1];
    res[5] = G_data[2];


    return 1;
}