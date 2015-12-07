/****************************************************************************
* Code from: https://github.com/kriswiner/MAX21100
* MAX21100_LIS3MDL_MS5637_t3 Basic Example Code
* by: Kris Winer
* date: September 1, 2014
* license: Beerware - Use this code however you'd like. If you
* find it useful you can buy me a beer some time.
*****************************************************************************
* File: imu_task.c
* Purpose: Read temperature and pressure from sensor
* Project: DroneID
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2015-11-12 Martin Skriver, File created and modified
****************************************************************************/

/***************************************************************************/
/* application includes */
#include "free_rtos.h"
#include "i2cCom1.h"
#include "imu_task.h"

uint8_t Mscale = MFS_4Gauss;  // Select magnetometer full-scale resolution
uint8_t Mopmode = MOM_hiperf; // Select magnetometer perfomance mode
uint8_t Modr = MODR_10Hz;     // Select magnetometer ODR when in MAX21100 bypass mode  //10
uint8_t MSodr = MODR_div8;   // Select magnetometer ODR as Aodr/MODR_divx  ///8


uint8_t osr_d1_pressure = D1_OSR_4096;       // set pressure oversample rate
uint8_t osr_d2_temp = D2_OSR_4096;       // set temperature oversample rate
uint16_t prom_data[7] = {0,0,0,0,0,0,0};


void setup_imu(void)
{
	MS5637_Reset(); //Perform reset for MS5637
	MS5637_Prom_Read(prom_data); //Read PROM data from MS5637
}


uint32_t I2C_MS5637_Read(uint8_t osr)
{
	uint8_t data[3] = {0,0,0};
	uint8_t adc_read_reg = MS5637_ADC_READ;



	I2C_DRV_MasterSendDataBlocking(FSL_I2CCOM1, &MS5637_config, NULL, 0, &osr, 1, 1000); //initiate a pressure conversion
    switch (osr)
    {
      case D1_OSR_256: OSA_TimeDelay(1); break;  // delay for conversion to complete
      case D1_OSR_512: OSA_TimeDelay(2); break;
      case D1_OSR_1024: OSA_TimeDelay(3); break;
      case D1_OSR_2048: OSA_TimeDelay(5); break;
      case D1_OSR_4096: OSA_TimeDelay(10); break;
      case D1_OSR_8192: OSA_TimeDelay(18); break;
      case D2_OSR_256: OSA_TimeDelay(1); break;
      case D2_OSR_512: OSA_TimeDelay(2); break;
      case D2_OSR_1024: OSA_TimeDelay(3); break;
      case D2_OSR_2048: OSA_TimeDelay(5); break;
      case D2_OSR_4096: OSA_TimeDelay(10); break;
      case D2_OSR_8192: OSA_TimeDelay(18); break;
    }

    I2C_DRV_MasterSendDataBlocking(FSL_I2CCOM1, &MS5637_config, NULL, 0, &adc_read_reg, 1, 1000); //initiate ADC read sequence

    I2C_DRV_MasterReceiveDataBlocking(FSL_I2CCOM1, &MS5637_config, NULL, 0, data, 3, 1000); //Read ADC registers

    return (uint32_t) (((uint32_t) data[0] << 16) | (uint32_t) data[1] << 8 | data[2]); // construct PROM data for return to main program
}

void get_MS5637_temp_and_pressure(double* temperature_ptr, double* pressure_ptr, uint8_t osr_d1, uint8_t osr_d2)
{
	uint32_t raw_temperature = 0;
	uint32_t raw_pressure = 0;
	double dt, offset, sens, t2, offset2, sens2;
	double temperature_buf;

	raw_pressure = I2C_MS5637_Read(osr_d1);
	raw_temperature = I2C_MS5637_Read(osr_d2);

    dt = raw_temperature - prom_data[5]*pow(2,8);    // calculate temperature difference from reference
    offset = prom_data[2]*pow(2, 17) + dt*prom_data[4]/pow(2,6);
    sens = prom_data[1]*pow(2,16) + (dt*prom_data[3])/pow(2,7);

    temperature_buf = (2000 + (dt*prom_data[6])/pow(2, 23))/100;           // First-order Temperature in degrees Centigrade

    // Second order corrections
    if(temperature_buf >= 20)
    {
    	t2 = 5*dt*dt/powl(2, 38); // correction for high temperatures
    	offset2 = 0;
    	sens2 = 0;
    }
    if(temperature_buf < 20)                   // correction for low temperature
    {
    	t2      = 3*dt*dt/powl(2, 33);
    	offset2 = 61*(temperature_buf - 2000)*(temperature_buf - 2000)/16;
    	sens2   = 29*(temperature_buf - 2000)*(temperature_buf - 2000)/16;
    }
    if(temperature_buf < -15)                      // correction for very low temperature
    {
    	offset2 = offset2 + 17*(temperature_buf + 1500)*(temperature_buf + 1500);
    	sens2 = sens2 + 9*(temperature_buf + 1500)*(temperature_buf + 1500);
    }

    if((temperature_buf < -50) || (temperature_buf > 100))
    {
//    	uint8_t reset_ctr = 0;
//    	for(reset_ctr = 0; reset_ctr < 7; reset_ctr++)
//    	{
//        	prom_data[reset_ctr] = 0;
//    	}
//    	setup_imu();
    }
    else
    {
        // End of second order corrections
        offset = offset - offset2;
        sens = sens - sens2;

    	if(xSemaphoreTake(xSemaphore_pressure_temperature_mutex_handle, 0))
    	{
    	    *temperature_ptr = temperature_buf - t2;
    	    *pressure_ptr = (((raw_pressure*sens)/pow(2, 21) - offset)/pow(2, 15))/100;  // Pressure in mbar or kPa		xSemaphoreGive( xSemaphore_pressure_temperature_mutex_handle );
    		xSemaphoreGive( xSemaphore_pressure_temperature_mutex_handle );
    	}
    }

}


void loop_imu()
{
	vTaskDelay(1/portTICK_RATE_MS);
	get_MS5637_temp_and_pressure(&temperature, &pressure, osr_d1_pressure, osr_d2_temp);
	// Data ready configuration
	// Enable master mode to read magnetometer from MAX21100 (bit 7 = 0, default)
	// Clear data ready bits when status register is read (bits 3:2 = 10)
	// disable temperature sensor (bits 0 = 0)

	//debug_printf("Temperature(mCelsius): %d, Pressure(uBar): %d \r\n\n", (uint32_t)(temperature*1000), (uint32_t)(pressure*1000));
	//debug_printf("%d\r\n", ++p);
}

void MS5637_Prom_Read(uint16_t *destination)
{
	uint8_t prom_cmd;
	uint8_t temp[2] = {0,0};
	static uint8_t i = 0;

	for(i=0; i<7; ++i)
	{
		prom_cmd = MS5637_PROM | (i<<1);
		I2C_DRV_MasterSendDataBlocking(FSL_I2CCOM1, &MS5637_config, NULL, 0, &prom_cmd, 1, 1000); //Send read prom command
		I2C_DRV_MasterReceiveDataBlocking(FSL_I2CCOM1, &MS5637_config, NULL, 0, temp, 2, 1000); //Read Prom registers
		destination[i] = (uint16_t) (((int16_t)temp[0] << 8) | temp[1]) ;
	}
}


void MS5637_Reset()
{
	uint8_t reset_cmd = MS5637_RESET;

	I2C_DRV_MasterSendDataBlocking(FSL_I2CCOM1, &MS5637_config, NULL, 0, &reset_cmd, 1, 1000); //Reset MS5637
}

void imu_main_task(void *param)
{
	xSemaphore_pressure_temperature_mutex_handle = xSemaphoreCreateMutex();
	if(xSemaphoreTake(xSemaphore_pressure_temperature_mutex_handle, 0))
	{
		temperature = 0.0;
		pressure = 0.0;
		xSemaphoreGive( xSemaphore_pressure_temperature_mutex_handle );
	}

	setup_imu();

	while(true)
	{
//		loop_imu();

	  vTaskDelay(20/portTICK_RATE_MS); /* wait for 10ms (100Hz) */
	} /* for */
}
