/****************************************************************************
* Code from: https://github.com/kriswiner/MAX21100
* MAX21100_LIS3MDL_MS5637_t3 Basic Example Code
* by: Kris Winer
* date: September 1, 2014
* license: Beerware - Use this code however you'd like. If you
* find it useful you can buy me a beer some time.
*****************************************************************************
* File: imu_task.h
* Purpose: Read temperature and pressure from sensor
* Project: DroneID
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2015-11-12 Martin Skriver, File created and modified
****************************************************************************/

#ifndef IMU_TASK_H_
#define IMU_TASK_H_

#define LIS3_DRDY_LOC (1<<30)
#define LIS3_DRDY_READ 	GPIOE_PDIR & LIS3_DRDY_LOC


// See MS5637-02BA03 Low Voltage Barometric Pressure Sensor Data Sheet
// http://www.meas-spec.com/product/pressure/MS5637-02BA03.aspx
//
#define MS5637_RESET      0b00011110
#define MS5637_ADC_READ   0b00000000
#define MS5637_PROM		  0b10100000

#define D1_OSR_256  0b01000000 // define pressure and temperature conversion rates
#define D1_OSR_512  0b01000010
#define D1_OSR_1024 0b01000100
#define D1_OSR_2048 0b01000110
#define D1_OSR_4096 0b01001000
#define D1_OSR_8192 0b01001010
#define D2_OSR_256  0b01010000
#define D2_OSR_512  0b01010010
#define D2_OSR_1024 0b01010100
#define D2_OSR_2048 0b01010110
#define D2_OSR_4096 0b01011000
#define D2_OSR_8192 0b01011010


// LIS3MDL Digital output magnetic sensor:
// http://www.st.com/web/catalog/sense_power/FM89/SC1449/PF255198
//
//Magnetometer Registers
#define LIS3MDL_ADDRESS      0x1C
#define LIS3MDL_WHO_AM_I     0x0F  // should return 0x3D
#define LIS3MDL_CTRL_REG1    0x20
#define LIS3MDL_CTRL_REG2    0x21
#define LIS3MDL_CTRL_REG3    0x22
#define LIS3MDL_CTRL_REG4    0x23
#define LIS3MDL_CTRL_REG5    0x24
#define LIS3MDL_STATUS_REG   0x27
#define LIS3MDL_OUT_X_L	     0x28  // data
#define LIS3MDL_OUT_X_H	     0x29
#define LIS3MDL_OUT_Y_L	     0x2A
#define LIS3MDL_OUT_Y_H	     0x2B
#define LIS3MDL_OUT_Z_L	     0x2C
#define LIS3MDL_OUT_Z_H	     0x2D
#define LIS3MDL_TEMP_OUT_L   0x2E
#define LIS3MDL_TEMP_OUT_H   0x2F  // data
#define LIS3MDL_INT_CFG	     0x30
#define LIS3MDL_INT_SRC	     0x31
#define LIS3MDL_INT_THS_L    0x32
#define LIS3MDL_INT_THS_H    0x33

#define LIS3MDL_ADDRESS  0x1C   //  Address of LIS3MDL magnetometer

enum Mscale {
  MFS_4Gauss = 0,  // 0.15 mG per LSB
  MFS_8Gauss,      // 0.30 mG per LSB
  MFS_12Gauss,     // 0.60 mG per LSB
  MFS_16Gauss      // 1.20 mG per LSB
};

enum Mopmode {
  MOM_lowpower = 0,
  MOM_medperf,
  MOM_hiperf,
  MOM_ultrahiperf
};

enum MSodr {        // magnetometer output data rate when slaved to the MAX21100
  MODR_div1 = 0,    // default, magnetometer ODR is 1/1 of the accel ODR
  MODR_div2,
  MODR_div4,
  MODR_div8,
  MODR_div16,
  MODR_div32,
  MODR_div64,
  MODR_div128
};

enum Modr {         // magnetometer output data rate MAX21100 is bypassed
  MODR_0_625Hz = 0,
  MODR_1_25Hz,
  MODR_2_5Hz,
  MODR_5Hz,
  MODR_10Hz,
  MODR_20Hz,
  MODR_40Hz,
  MODR_80Hz
};

/***************************************************************************/
/* Shared variables */
TaskHandle_t imu_main_task_handle;

SemaphoreHandle_t xSemaphore_pressure_temperature_mutex_handle;
double temperature, pressure;

/***************************************************************************/
/* shared funtions */
void imu_main_task(void *param);

#endif /* IMU_TASK_H_ */
