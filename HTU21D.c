#include "HTU21D.h"
#include "tm_stm32f4_i2c.h"

float readHumidity(void){
	char data[3];
	char msb,lsb,checksum;
	float rh;
	float tempRH;


		/* Write "5" at location 0x00 to slave with address ADDRESS */
		//TM_I2C_WriteNoRegister(I2C1, (HTDU21D_ADDRESS<<1), TRIGGER_HUMD_MEASURE_NOHOLD);
	TM_I2C_WriteNoRegister(I2C1, (HTDU21D_ADDRESS<<1), TRIGGER_HUMD_MEASURE_HOLD);

		delay_ms(20);



		TM_I2C_ReadMultiNoRegister(I2C1,(HTDU21D_ADDRESS<<1),data,3);

		unsigned int rawHumidity = ((unsigned int) data[0] << 8) | (unsigned int) data[1];

			//if(check_crc(rawHumidity, checksum) != 0) return(999); //Error out

			//sensorStatus = rawHumidity & 0x0003; //Grab only the right two bits
			rawHumidity &= 0xFFFC; //Zero out the status bits but keep them in place

			//Given the raw humidity data, calculate the actual relative humidity
			tempRH = rawHumidity / (float)65536; //2^16 = 65536
			rh = -6 + (125 * tempRH); //From page 14

			return rh;
}

float readTemperature(void){

	char data[3];
		char msb,lsb,checksum;




			/* Write "5" at location 0x00 to slave with address ADDRESS */
			TM_I2C_WriteNoRegister(I2C1, (HTDU21D_ADDRESS<<1), TRIGGER_TEMP_MEASURE_HOLD);

			delay_ms(20);



			TM_I2C_ReadMultiNoRegister(I2C1,(HTDU21D_ADDRESS<<1),data,3);

			//if(check_crc(rawHumidity, checksum) != 0) return(999); //Error out

			unsigned int rawTemperature = ((unsigned int) data[0] << 8) | (unsigned int) data[1];

				//if(check_crc(rawTemperature, checksum) != 0) return(999); //Error out

				//sensorStatus = rawTemperature & 0x0003; //Grab only the right two bits
				rawTemperature &= 0xFFFC; //Zero out the status bits but keep them in place

				//Given the raw temperature data, calculate the actual temperature
				float tempTemperature = rawTemperature / (float)65536; //2^16 = 65536
				float realTemperature = (float)(-46.85 + (175.72 * tempTemperature)); //From page 14

				return realTemperature;

}
