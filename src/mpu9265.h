/*IMU: MPU9265*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>

// register map
#define MPU_ACCEL_OFFSET_X1 119
#define MPU_ACCEL_OFFSET_Y1 122
#define MPU_ACCEL_OFFSET_Z1 125
#define MPU_GYRO_OFFSET_X1  19
#define MPU_GYRO_OFFSET_Y1  21
#define MPU_GYRO_OFFSET_Z1  23

#define MPU_ACCEL_DATA_X1  59   // 0x1F
#define MPU_ACCEL_DATA_Y1  61
#define MPU_ACCEL_DATA_Z1  63
#define MPU_GYRO_DATA_X1   67
#define MPU_GYRO_DATA_Y1   69
#define MPU_GYRO_DATA_Z1   71

#define MPU_WHO_AM_I       117

// I2C address
#define MPU_ADDR 0x68

// full-scale range of accel and gyro sensors
const int MPU_AccelRange_idx = 3;   //choose 0=32G, 1=16g, 2=8g, 3=4g, 4=2g
const int  MPU_GyroRange_idx = 1;   //choose 0=2000dps, 1=1000dps, 2=500dps, 3=250dps, 4=125dps

// Usual DataRate
const int MPU_DataRate_idx = 15;

// Return the gyro full-scale range, in Degrees per Second,
// for each of the range selection settings.
double MPU_GyroRangeVal_dps(int range_idx)
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
double MPU_AccelRangeVal_G(int range_idx)
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
double MPU_SampRate(int rate_idx)
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
double MPU_ADC2Float(double val) {
    return val / 32768.0;
}



uint8_t MPU_ReadByte(const struct device *dev, int Reg_addr, int I2C_addr) {
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

short MPU_ReadShort(const struct device *dev, int Reg_addr, int I2C_addr) {
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

int MPU_ReadSensor(const struct device *dev, short *res, int I2C_addr){
    /* Read 14 bytes together, then get accel and gyro sensor data 
    */
    uint8_t data[1],buf[14];
    int err_write,err_read;
    struct i2c_msg msg;

    data[0] = MPU_ACCEL_DATA_X1;
    
    msg.buf = data;
	msg.len = 1;
	msg.flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	err_write = i2c_transfer(dev, &msg, 1, I2C_addr);
    if (err_write != 0){
        printk("i2c_write failed\n");
        return 0;
        }

    msg.buf = buf;
	msg.len = 14;
	msg.flags = I2C_MSG_READ | I2C_MSG_STOP;

    err_read = i2c_transfer(dev, &msg, 1, I2C_addr);
    if (err_read != 0){
        printk("i2c_read failed\n");
        return 0;
        }
    
    res[0] = (short)((buf[0] << 8) | buf[1]);
    res[1] = (short)((buf[2] << 8) | buf[3]);
    res[2] = (short)((buf[4] << 8) | buf[5]);
    res[3] = (short)((buf[8] << 8) | buf[9]);
    res[4] = (short)((buf[10] << 8) | buf[11]);
    res[5] = (short)((buf[12] << 8) | buf[13]);

    return 1;
}





