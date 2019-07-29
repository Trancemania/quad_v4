/**
 * |----------------------------------------------------------------------
 * | Copyright (C) Trancemania,2019
 * |
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */

#include "mpu9250.h"

/* Default I2C address */
#define MPU9250_I2C_ADDR			0xD0
#define AK8963_I2C_ADDR   		0x18

/* Who I am register value */
#define MPU9250_I_AM				0x71		  //9250
#define AK8963_I_AM					0x48

/* ?? */
#define MPU9250_AUX_VDDIO			0x01

/* MPU9250 registers */
#define MPU9250_SELF_TEST_X_GYRO 				0x00
#define MPU9250_SELF_TEST_Y_GYRO 				0x01
#define MPU9250_SELF_TEST_Z_GYRO 				0x02

/*
#define MPU9250_X_FINE_GAIN      0x03 // [7:0] fine gain
#define MPU9250_Y_FINE_GAIN      0x04
#define MPU9250_Z_FINE_GAIN      0x05
#define MPU9250_XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define MPU9250_XA_OFFSET_L_TC   0x07
#define MPU9250_YA_OFFSET_H      0x08
#define MPU9250_YA_OFFSET_L_TC   0x09
#define MPU9250_ZA_OFFSET_H      0x0A
#define MPU9250_ZA_OFFSET_L_TC   0x0B 
*/

#define MPU9250_SELF_TEST_X_ACCEL 			0x0D
#define MPU9250_SELF_TEST_Y_ACCEL 			0x0E
#define MPU9250_SELF_TEST_Z_ACCEL 			0x0F

#define MPU9250_SMPLRT_DIV			0x19 //
#define MPU9250_CONFIG					0x1A
#define MPU9250_GYRO_CONFIG			0x1B //
#define MPU9250_ACCEL_CONFIG		0x1C //

#define MPU9250_ACCEL_CONFIG2		0x1D //
#define MPU9250_LP_ACCEL_ODR    0x1E   
#define MPU9250_WOM_THR         0x1F

//#define MPU9250_MOTION_THRESH		0x1F

/* not in reg map */
#define MPU9250_MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define MPU9250_ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define MPU9250_ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms
/**/

#define MPU9250_FIFO_EN          0x23
#define MPU9250_I2C_MST_CTRL     0x24   
#define MPU9250_I2C_SLV0_ADDR    0x25
#define MPU9250_I2C_SLV0_REG     0x26
#define MPU9250_I2C_SLV0_CTRL    0x27
#define MPU9250_I2C_SLV1_ADDR    0x28
#define MPU9250_I2C_SLV1_REG     0x29
#define MPU9250_I2C_SLV1_CTRL    0x2A
#define MPU9250_I2C_SLV2_ADDR    0x2B
#define MPU9250_I2C_SLV2_REG     0x2C
#define MPU9250_I2C_SLV2_CTRL    0x2D
#define MPU9250_I2C_SLV3_ADDR    0x2E
#define MPU9250_I2C_SLV3_REG     0x2F
#define MPU9250_I2C_SLV3_CTRL    0x30
#define MPU9250_I2C_SLV4_ADDR    0x31
#define MPU9250_I2C_SLV4_REG     0x32
#define MPU9250_I2C_SLV4_DO      0x33
#define MPU9250_I2C_SLV4_CTRL    0x34
#define MPU9250_I2C_SLV4_DI      0x35
#define MPU9250_I2C_MST_STATUS   0x36

#define MPU9250_INT_PIN_CFG			 0x37
#define MPU9250_INT_ENABLE			 0x38

#define MPU9250_DMP_INT_STATUS   0x39  // Check DMP interrupt, not in reg map

#define MPU9250_INT_STATUS			0x3A
#define MPU9250_ACCEL_XOUT_H		0x3B
#define MPU9250_ACCEL_XOUT_L		0x3C
#define MPU9250_ACCEL_YOUT_H		0x3D
#define MPU9250_ACCEL_YOUT_L		0x3E
#define MPU9250_ACCEL_ZOUT_H		0x3F
#define MPU9250_ACCEL_ZOUT_L		0x40
#define MPU9250_TEMP_OUT_H			0x41
#define MPU9250_TEMP_OUT_L			0x42
#define MPU9250_GYRO_XOUT_H			0x43
#define MPU9250_GYRO_XOUT_L			0x44
#define MPU9250_GYRO_YOUT_H			0x45
#define MPU9250_GYRO_YOUT_L			0x46
#define MPU9250_GYRO_ZOUT_H			0x47
#define MPU9250_GYRO_ZOUT_L			0x48

#define MPU9250_EXT_SENS_DATA_00 0x49
#define MPU9250_EXT_SENS_DATA_01 0x4A
#define MPU9250_EXT_SENS_DATA_02 0x4B
#define MPU9250_EXT_SENS_DATA_03 0x4C
#define MPU9250_EXT_SENS_DATA_04 0x4D
#define MPU9250_EXT_SENS_DATA_05 0x4E
#define MPU9250_EXT_SENS_DATA_06 0x4F
#define MPU9250_EXT_SENS_DATA_07 0x50
#define MPU9250_EXT_SENS_DATA_08 0x51
#define MPU9250_EXT_SENS_DATA_09 0x52
#define MPU9250_EXT_SENS_DATA_10 0x53
#define MPU9250_EXT_SENS_DATA_11 0x54
#define MPU9250_EXT_SENS_DATA_12 0x55
#define MPU9250_EXT_SENS_DATA_13 0x56
#define MPU9250_EXT_SENS_DATA_14 0x57
#define MPU9250_EXT_SENS_DATA_15 0x58
#define MPU9250_EXT_SENS_DATA_16 0x59
#define MPU9250_EXT_SENS_DATA_17 0x5A
#define MPU9250_EXT_SENS_DATA_18 0x5B
#define MPU9250_EXT_SENS_DATA_19 0x5C
#define MPU9250_EXT_SENS_DATA_20 0x5D
#define MPU9250_EXT_SENS_DATA_21 0x5E
#define MPU9250_EXT_SENS_DATA_22 0x5F
#define MPU9250_EXT_SENS_DATA_23 0x60

#define MPU9250_MOT_DETECT_STATUS	0x61 // not in reg map

#define MPU9250_I2C_SLV0_DO        0x63
#define MPU9250_I2C_SLV1_DO        0x64
#define MPU9250_I2C_SLV2_DO        0x65
#define MPU9250_I2C_SLV3_DO        0x66
#define MPU9250_I2C_MST_DELAY_CTRL 0x67

#define MPU9250_SIGNAL_PATH_RESET	0x68
#define MPU9250_MOT_DETECT_CTRL		0x69
#define MPU9250_USER_CTRL					0x6A
#define MPU9250_PWR_MGMT_1				0x6B
#define MPU9250_PWR_MGMT_2				0x6C

/* not in reg map*/
#define MPU9250_DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define MPU9250_DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define MPU9250_DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define MPU9250_DMP_REG_1        0x70
#define MPU9250_DMP_REG_2        0x71 
/**/

#define MPU9250_FIFO_COUNTH			0x72
#define MPU9250_FIFO_COUNTL			0x73
#define MPU9250_FIFO_R_W				0x74
#define MPU9250_WHO_AM_I				0x75  // Should return 0x71

#define MPU9250_XA_OFFSET_H      0x77
#define MPU9250_XA_OFFSET_L      0x78
#define MPU9250_YA_OFFSET_H      0x7A
#define MPU9250_YA_OFFSET_L      0x7B
#define MPU9250_ZA_OFFSET_H      0x7D
#define MPU9250_ZA_OFFSET_L      0x7E



/* AK8963 registers */
#define AK8963_WHO_AM_I  				0x00 // should return 0x48
#define AK8963_INFO      				0x01
#define AK8963_ST1       				0x02  // data ready status bit 0
#define AK8963_XOUT_L    				0x03  // data
#define AK8963_XOUT_H    				0x04
#define AK8963_YOUT_L    				0x05
#define AK8963_YOUT_H    				0x06
#define AK8963_ZOUT_L    				0x07
#define AK8963_ZOUT_H    				0x08
#define AK8963_ST2       				0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL1      			0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_CNTL2      			0x0B  
#define AK8963_ASTC      				0x0C  // Self test control
#define AK8963_I2CDIS    				0x0F  // I2C disable
#define AK8963_ASAX      				0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      				0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      				0x12  // Fuse ROM z-axis sensitivity adjustment value


/* Gyro sensitivities in degrees/s */
#define MPU9250_GYRO_SENS_250		((float) 32768/250)
#define MPU9250_GYRO_SENS_500		((float) 32768/500)
#define MPU9250_GYRO_SENS_1000		((float) 32768/1000)
#define MPU9250_GYRO_SENS_2000		((float) 32768/2000)

/* Acce sensitivities in g/s */
#define MPU9250_ACCE_SENS_2			((float) 32768/2)
#define MPU9250_ACCE_SENS_4			((float) 32768/4)
#define MPU9250_ACCE_SENS_8			((float) 32768/8)
#define MPU9250_ACCE_SENS_16		((float) 32768/16)
	
/* Magn sensitivities in mG */
#define MPU9250_MAGN_SENS_14			((float) 8192/10/4912)
#define MPU9250_MAGN_SENS_16			((float) 32768/10/4912)

MPU9250_Result MPU9250_Init(I2C_HandleTypeDef* I2Cx,
														MPU9250* DataStruct,
														MPU9250_Device DeviceNumber,
														MPU9250_Accelerometer AccelerometerSensitivity,
														MPU9250_Gyroscope GyroscopeSensitivity,
														MPU9250_Magnetometer MagnetometerSensitivity){
	uint8_t MPU9250_WAI = (uint8_t)MPU9250_WHO_AM_I;
	uint8_t AK8963_WAI = (uint8_t)AK8963_WHO_AM_I;
	uint8_t MPU9250_ACFG2 = (uint8_t)MPU9250_ACCEL_CONFIG2;															
	uint8_t temp1;
	uint8_t temp2;
	uint8_t temp3;														
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t d[2];


	/* Format I2C address */
	DataStruct->MPU9250_Address = MPU9250_I2C_ADDR | (uint8_t)DeviceNumber;
	DataStruct->AK8963_Address = AK8963_I2C_ADDR | (uint8_t)DeviceNumber;														
	uint8_t mpu9250_address = DataStruct->MPU9250_Address;
	uint8_t ak8963_address = DataStruct->AK8963_Address;														

	/* Check if device is connected */
	if(HAL_I2C_IsDeviceReady(Handle,mpu9250_address,2,5)!=HAL_OK)//&&HAL_I2C_IsDeviceReady(Handle,ak8963_address,2,5)
	{
				return MPU9250_Result_Error;
	}
	/* Check MPU9250 who am I */
	//------------------
		/* Send address */
		if(HAL_I2C_Master_Transmit(Handle, mpu9250_address, &MPU9250_WAI, 1, 1000) != HAL_OK) //&&HAL_I2C_Master_Transmit(Handle, ak8963_address, &AK8963_WAI, 1, 1000)
		{
			return MPU9250_Result_Error;
		}

		/* Receive multiple byte */
		if(HAL_I2C_Master_Receive(Handle, mpu9250_address, &temp1, 1, 1000) != HAL_OK) //&&HAL_I2C_Master_Receive(Handle, ak8963_address, &temp2, 1, 1000)
		{
			return MPU9250_Result_Error;
		}

		/* Activate bypass to access AK8963 directly */
		d[0] = MPU9250_INT_PIN_CFG;
		d[1] = 0x02;
		if(HAL_I2C_Master_Transmit(Handle,(uint16_t)mpu9250_address , (uint8_t *)d, 2, 1000) != HAL_OK)
		{
					return MPU9250_Result_Error;
		}		
		
	/* Check if AK8963 is connected */
	if(HAL_I2C_IsDeviceReady(Handle,ak8963_address,2,5)!=HAL_OK)//&&HAL_I2C_IsDeviceReady(Handle,ak8963_address,2,5)
	{
				return MPU9250_Result_Error;
	}
	
	/* Check AK8963 who am I */
	//------------------
		/* Send address */
		if(HAL_I2C_Master_Transmit(Handle, ak8963_address, &AK8963_WAI, 1, 1000) != HAL_OK) //&&HAL_I2C_Master_Transmit(Handle, ak8963_address, &AK8963_WAI, 1, 1000)
		{
			return MPU9250_Result_Error;
		}

		/* Receive multiple byte */
		if(HAL_I2C_Master_Receive(Handle, ak8963_address, &temp2, 1, 1000) != HAL_OK) //&&HAL_I2C_Master_Receive(Handle, ak8963_address, &temp2, 1, 1000)
		{
			return MPU9250_Result_Error;
		}
		
		/* Checking */
		while(temp1 != MPU9250_I_AM || temp2 != AK8963_I_AM) //&& temp2 != AK8963_I_AM
		{
				/* Return error */
				return MPU9250_Result_DeviceInvalid;
		}
	//------------------

	/* Wakeup MPU9250 */
	//------------------
		/* Format array to send */
		d[0] = MPU9250_PWR_MGMT_1;
		d[1] = 0x00; // clear bit 6

		/* Try to transmit via I2C */
		if(HAL_I2C_Master_Transmit(Handle,(uint16_t)mpu9250_address , (uint8_t *)d, 2, 1000) != HAL_OK)
		{
					return MPU9250_Result_Error;
		}
	//------------------
		
	/* Get stable time source */
	//------------------
		/* Format array to send */
		d[0] = MPU9250_PWR_MGMT_1;
		d[1] = 0x01;

		/* Try to transmit via I2C */
		if(HAL_I2C_Master_Transmit(Handle,(uint16_t)mpu9250_address , (uint8_t *)d, 2, 1000) != HAL_OK)
		{
					return MPU9250_Result_Error;
		}
	//------------------

	/* Configure Digital Low Pass Filter (DLPF) */
	//------------------
		/* Format array to send */
		d[0] = MPU9250_CONFIG;
		d[1] = 0x03; //Accelerometer Bandwidth 44Hz, Temperature Bandwidth 42Hz

		/* Try to transmit via I2C */
		if(HAL_I2C_Master_Transmit(Handle,(uint16_t)mpu9250_address , (uint8_t *)d, 2, 1000) != HAL_OK)
		{
					return MPU9250_Result_Error;
		}
	//------------------
		
	/* Set sample rate to 200Hz, Internal sample rate is 1KHz decided by DLPF_CFG, 200 = 1000 / (4+1)*/
	MPU9250_SetDataRate(I2Cx,DataStruct, 4);

	/* Config accelerometer */
	MPU9250_SetAccelerometer(I2Cx,DataStruct, AccelerometerSensitivity);

	/* Configure Accelerometer Digital Low Pass Filter (A-DLPF) */
	//------------------
		while(HAL_I2C_Master_Transmit(Handle, mpu9250_address, &MPU9250_ACFG2, 1, 1000) != HAL_OK);
		while(HAL_I2C_Master_Receive(Handle, mpu9250_address, &temp3, 1, 1000) != HAL_OK);
		temp3 = (temp3 & 0xF0) | 0x03; // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
		while(HAL_I2C_Master_Transmit(Handle, mpu9250_address, &temp3, 1, 1000) != HAL_OK);
	//------------------		
		
	/* Config Gyroscope */
	MPU9250_SetGyroscope(I2Cx,DataStruct, GyroscopeSensitivity);

	/* Config Magnetometer */
	MPU9250_SetMagnetometer(I2Cx,DataStruct, MagnetometerSensitivity);
	
	/* Return OK */
	return MPU9250_Result_Ok;
}

MPU9250_Result MPU9250_SetDataRate(I2C_HandleTypeDef* I2Cx,MPU9250* DataStruct, uint8_t rate)
{
	uint8_t d[2];
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t mpu9250_address = DataStruct->MPU9250_Address;
	/* Format array to send */
	d[0] = MPU9250_SMPLRT_DIV;
	d[1] = rate;

	/* Set data sample rate */
	while(HAL_I2C_Master_Transmit(Handle,(uint16_t)mpu9250_address,(uint8_t *)d,2,1000)!=HAL_OK);
	/*{
				return MPU9250_Result_Error;
	}*/

	/* Return OK */
	return MPU9250_Result_Ok;
}

MPU9250_Result MPU9250_SetAccelerometer(I2C_HandleTypeDef* I2Cx,MPU9250* DataStruct, MPU9250_Accelerometer AccelerometerSensitivity)
{
	uint8_t temp;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t mpu9250_address = DataStruct->MPU9250_Address;
	uint8_t regAdd =(uint8_t )MPU9250_ACCEL_CONFIG;

	/* Config accelerometer */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)mpu9250_address,&regAdd, 1, 1000) != HAL_OK);
	/*{
				return MPU9250_Result_Error;
	}*/
	while(HAL_I2C_Master_Receive(Handle, (uint16_t)mpu9250_address, &temp, 1, 1000) != HAL_OK);
	/*{
				return MPU9250_Result_Error;
	}*/
	temp = (temp & 0xE7) | (uint8_t)AccelerometerSensitivity << 3;
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)mpu9250_address,&temp, 1, 1000) != HAL_OK);
	/*{
				return MPU9250_Result_Error;
	}*/

	/* Set sensitivities for multiplying gyro and accelerometer data */
	switch (AccelerometerSensitivity) {
		case MPU9250_Accelerometer_2G:
			DataStruct->Acce_Mult = (float)1 / MPU9250_ACCE_SENS_2;
			break;
		case MPU9250_Accelerometer_4G:
			DataStruct->Acce_Mult = (float)1 / MPU9250_ACCE_SENS_4;
			break;
		case MPU9250_Accelerometer_8G:
			DataStruct->Acce_Mult = (float)1 / MPU9250_ACCE_SENS_8;
			break;
		case MPU9250_Accelerometer_16G:
			DataStruct->Acce_Mult = (float)1 / MPU9250_ACCE_SENS_16;
			break;
		default:
			break;
		}

	/* Return OK */
	return MPU9250_Result_Ok;
}

MPU9250_Result MPU9250_SetGyroscope(I2C_HandleTypeDef* I2Cx,MPU9250* DataStruct, MPU9250_Gyroscope GyroscopeSensitivity)
{
	uint8_t temp;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t mpu9250_address = DataStruct->MPU9250_Address;
	uint8_t regAdd =(uint8_t )MPU9250_GYRO_CONFIG;

	/* Config gyroscope */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)mpu9250_address,&regAdd, 1, 1000) != HAL_OK);
	/*{
				return MPU9250_Result_Error;
	}*/
	while(HAL_I2C_Master_Receive(Handle, (uint16_t)mpu9250_address, &temp, 1, 1000) != HAL_OK);
	/*{
				return MPU9250_Result_Error;
	}*/
	temp = (temp & 0xE4) | (uint8_t)GyroscopeSensitivity << 3;
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)mpu9250_address,&temp, 1, 1000) != HAL_OK);
	/*{
				return MPU9250_Result_Error;
	}*/

	switch (GyroscopeSensitivity) {
			case MPU9250_Gyroscope_250s:
				DataStruct->Gyro_Mult = (float)1 / MPU9250_GYRO_SENS_250;
				break;
			case MPU9250_Gyroscope_500s:
				DataStruct->Gyro_Mult = (float)1 / MPU9250_GYRO_SENS_500;
				break;
			case MPU9250_Gyroscope_1000s:
				DataStruct->Gyro_Mult = (float)1 / MPU9250_GYRO_SENS_1000;
				break;
			case MPU9250_Gyroscope_2000s:
				DataStruct->Gyro_Mult = (float)1 / MPU9250_GYRO_SENS_2000;
				break;
			default:
				break;
		}
	/* Return OK */
	return MPU9250_Result_Ok;
}

MPU9250_Result MPU9250_SetMagnetometer(I2C_HandleTypeDef* I2Cx, MPU9250* DataStruct, MPU9250_Magnetometer MagnetometerSensitivity)
{
	uint8_t d[2];
	uint8_t temp[3];
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t ak8963_address = DataStruct->AK8963_Address;
	uint8_t regAdd =(uint8_t )AK8963_ASAX;
	
	/* Config magnetometer */
	d[0] = AK8963_CNTL1;
	d[1] = 0x00;   // power down
	while(HAL_I2C_Master_Transmit(Handle,(uint16_t)ak8963_address , (uint8_t *)d, 2, 1000) != HAL_OK);
	/*{
				return MPU9250_Result_Error;
	}*/
	
	d[0] = AK8963_CNTL1;
	d[1] = 0x0F;   // Fuse ROM access mode
	while(HAL_I2C_Master_Transmit(Handle,(uint16_t)ak8963_address , (uint8_t *)d, 2, 1000) != HAL_OK);
	/*{
				return MPU9250_Result_Error;
	}*/

// Read the x-, y-, and z-axis calibration values
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)ak8963_address, &regAdd, 1, 1000) != HAL_OK);
	/*{
				return MPU9250_Result_Error;
	}*/
	while(HAL_I2C_Master_Receive(Handle, (uint16_t)ak8963_address, &temp[0], 3, 1000) != HAL_OK);
	/*{
				return MPU9250_Result_Error;
	}*/
	DataStruct->Magn_Calix = (float)(temp[0] - 128)/256.0f + 1.0f;
	DataStruct->Magn_Caliy = (float)(temp[1] - 128)/256.0f + 1.0f;
	DataStruct->Magn_Caliz = (float)(temp[2] - 128)/256.0f + 1.0f;
	
// Configure the magnetometer for continuous read and highest resolution
	d[0] = AK8963_CNTL1;
	d[1] = 0x00;   // power down
	while(HAL_I2C_Master_Transmit(Handle,(uint16_t)ak8963_address , (uint8_t *)d, 2, 1000) != HAL_OK);
	/*{
				return MPU9250_Result_Error;
	}*/

	d[0] = AK8963_CNTL1;
	d[1] = 0x04 | (uint8_t)MagnetometerSensitivity << 4;   //enable continuous mode data acquisition Mmode (bits [3:0]) at 100Hz
	while(HAL_I2C_Master_Transmit(Handle,(uint16_t)ak8963_address , (uint8_t *)d, 2, 1000) != HAL_OK);
	/*{
				return MPU9250_Result_Error;
	}*/

	switch (MagnetometerSensitivity) {
			case MPU9250_Magnetometer_14bit:
				DataStruct->Magn_Mult = (float)1 / MPU9250_MAGN_SENS_14;
				break;
			case MPU9250_Magnetometer_16bit:
				DataStruct->Magn_Mult = (float)1 / MPU9250_MAGN_SENS_16;
				break;
			default:
				break;
		}
	/* Return OK */
	return MPU9250_Result_Ok;
}


MPU9250_Result MPU9250_ReadAccelerometer(I2C_HandleTypeDef* I2Cx,MPU9250* DataStruct)
{
	uint8_t data[6];
	uint8_t reg = MPU9250_ACCEL_XOUT_H;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t mpu9250_address = DataStruct->MPU9250_Address;

	/* Read accelerometer data */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)mpu9250_address, &reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)mpu9250_address, data, 6, 1000) != HAL_OK);

	/* Format */
	DataStruct->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);
	DataStruct->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);

	/* Return OK */
	return MPU9250_Result_Ok;
}
MPU9250_Result MPU9250_ReadGyroscope(I2C_HandleTypeDef* I2Cx,MPU9250* DataStruct)
{
	uint8_t data[6];
	uint8_t reg = MPU9250_GYRO_XOUT_H;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t mpu9250_address = DataStruct->MPU9250_Address;

	/* Read gyroscope data */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)mpu9250_address, &reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)mpu9250_address, data, 6, 1000) != HAL_OK);

	/* Format */
	DataStruct->Gyroscope_X = (int16_t)(data[0] << 8 | data[1]);
	DataStruct->Gyroscope_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Gyroscope_Z = (int16_t)(data[4] << 8 | data[5]);

	/* Return OK */
	return MPU9250_Result_Ok;
}

MPU9250_Result MPU9250_ReadTemperature(I2C_HandleTypeDef* I2Cx,MPU9250* DataStruct)
{
	uint8_t data[2];
	int16_t temp;
	uint8_t reg = MPU9250_TEMP_OUT_H;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t mpu9250_address = DataStruct->MPU9250_Address;

	/* Read temperature */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)mpu9250_address, &reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)mpu9250_address, data, 2, 1000) != HAL_OK);

	/* Format temperature */
	temp = (data[0] << 8 | data[1]);
	DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);

	/* Return OK */
	return MPU9250_Result_Ok;
}

MPU9250_Result MPU9250_ReadAll(I2C_HandleTypeDef* I2Cx,MPU9250* DataStruct)
{
	uint8_t data[14];
	int16_t temp;
	uint8_t reg = MPU9250_ACCEL_XOUT_H;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t mpu9250_address = DataStruct->MPU9250_Address;

	/* Read full raw data, 14bytes */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)mpu9250_address, &reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)mpu9250_address, data, 14, 1000) != HAL_OK);

	/* Format accelerometer data */
	DataStruct->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);
	DataStruct->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);

	/* Format temperature */
	temp = (data[6] << 8 | data[7]);
	DataStruct->Temperature = (float)((float)((int16_t)temp) / (float)340.0 + (float)36.53);

	/* Format gyroscope data */
	DataStruct->Gyroscope_X = (int16_t)(data[8] << 8 | data[9]);
	DataStruct->Gyroscope_Y = (int16_t)(data[10] << 8 | data[11]);
	DataStruct->Gyroscope_Z = (int16_t)(data[12] << 8 | data[13]);

	/* Return OK */
	return MPU9250_Result_Ok;
}

MPU9250_Result MPU9250_EnableInterrupts(I2C_HandleTypeDef* I2Cx,MPU9250* DataStruct)
{
	uint8_t temp;
	uint8_t reg[2] = {MPU9250_INT_ENABLE,0x21};
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t mpu9250_address = DataStruct->MPU9250_Address;

	/* Enable interrupts for data ready and motion detect */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)mpu9250_address, reg, 2, 1000) != HAL_OK);

	uint8_t mpu_reg= MPU9250_INT_PIN_CFG;
	/* Clear IRQ flag on any read operation */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)mpu9250_address, &mpu_reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)mpu9250_address, &temp, 14, 1000) != HAL_OK);
	temp |= 0x10;
	reg[0] = MPU9250_INT_PIN_CFG;
	reg[1] = temp;
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)mpu9250_address, reg, 2, 1000) != HAL_OK);

	/* Return OK */
	return MPU9250_Result_Ok;
}

MPU9250_Result MPU9250_DisableInterrupts(I2C_HandleTypeDef* I2Cx,MPU9250* DataStruct)
{
	uint8_t reg[2] = {MPU9250_INT_ENABLE,0x00};
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t mpu9250_address = DataStruct->MPU9250_Address;

	/* Disable interrupts */
	while(HAL_I2C_Master_Transmit(Handle,(uint16_t)mpu9250_address,reg,2,1000)!=HAL_OK);
	/* Return OK */
	return MPU9250_Result_Ok;
}

MPU9250_Result MPU9250_ReadInterrupts(I2C_HandleTypeDef* I2Cx,MPU9250* DataStruct, MPU9250_Interrupt* InterruptsStruct)
{
	uint8_t read;

	/* Reset structure */
	InterruptsStruct->Status = 0;
	uint8_t reg = MPU9250_INT_STATUS;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t mpu9250_address = DataStruct->MPU9250_Address;

	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)mpu9250_address, &reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)mpu9250_address, &read, 14, 1000) != HAL_OK);

	/* Fill value */
	InterruptsStruct->Status = read;
	/* Return OK */
	return MPU9250_Result_Ok;
}
