#ifndef __MPU9250_H
#define __MPU9250_H

/*
 C++ detection
#ifdef __cplusplus
extern "C" {
#endif
*/

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"

/**
 * @defgroup MPU9250_Macros
 * @brief    Library defines
 * @{
 */

/* Default I2C clock */
#ifndef MPU9250_I2C_CLOCK
	#define MPU9250_I2C_CLOCK              400000            /*!< Default I2C clock speed */
#endif

/**
 * @brief  Data rates predefined constants
 * @{
 */
#define MPU9250_DataRate_8KHz       0   /*!< Sample rate set to 8 kHz */
#define MPU9250_DataRate_4KHz       1   /*!< Sample rate set to 4 kHz */
#define MPU9250_DataRate_2KHz       3   /*!< Sample rate set to 2 kHz */
#define MPU9250_DataRate_1KHz       7   /*!< Sample rate set to 1 kHz */
#define MPU9250_DataRate_500Hz      15  /*!< Sample rate set to 500 Hz */
#define MPU9250_DataRate_250Hz      31  /*!< Sample rate set to 250 Hz */
#define MPU9250_DataRate_125Hz      63  /*!< Sample rate set to 125 Hz */
#define MPU9250_DataRate_100Hz      79  /*!< Sample rate set to 100 Hz */
/**
 * @}
 */

/**
 * @}
 */

/**
 * @defgroup MPU9250_Typedefs
 * @brief    Library Typedefs
 * @{
 */

/**
 * @brief  MPU9250 can have 2 different slave addresses, depends on it's input AD0 pin
 *         This feature allows you to use 2 different sensors with this library at the same time
 */
typedef enum  {
	MPU9250_Device_0 = 0x00, /*!< AD0 pin is set to low */
	MPU9250_Device_1 = 0x02  /*!< AD0 pin is set to high */
} MPU9250_Device;

/**
 * @brief  MPU9250 result enumeration
 */
typedef enum  {
	MPU9250_Result_Ok = 0x00,          /*!< Everything OK */
	MPU9250_Result_Error,              /*!< Unknown error */
	MPU9250_Result_DeviceNotConnected, /*!< There is no device with valid slave address */
	MPU9250_Result_DeviceInvalid       /*!< Connected device with address is not MPU9250 */
} MPU9250_Result;

/**
 * @brief  Parameters for accelerometer range
 */
typedef enum  {
	MPU9250_Accelerometer_2G = 0x00, /*!< Range is +- 2G */
	MPU9250_Accelerometer_4G = 0x01, /*!< Range is +- 4G */
	MPU9250_Accelerometer_8G = 0x02, /*!< Range is +- 8G */
	MPU9250_Accelerometer_16G = 0x03 /*!< Range is +- 16G */
} MPU9250_Accelerometer;

/**
 * @brief  Parameters for gyroscope range
 */
typedef enum {
	MPU9250_Gyroscope_250s = 0x00,  /*!< Range is +- 250 degrees/s */
	MPU9250_Gyroscope_500s = 0x01,  /*!< Range is +- 500 degrees/s */
	MPU9250_Gyroscope_1000s = 0x02, /*!< Range is +- 1000 degrees/s */
	MPU9250_Gyroscope_2000s = 0x03  /*!< Range is +- 2000 degrees/s */
} MPU9250_Gyroscope;

/**
 * @brief  Parameters for Magnetometer range
 */
typedef enum {
	MPU9250_Magnetometer_14bit = 0x00,  /*!< Range is +- 0.6 mG per LSB */
	MPU9250_Magnetometer_16bit = 0x01   /*!< Range is +- 0.15 mG per LSB */
} MPU9250_Magnetometer;

/**
 * @brief  Main MPU9250 structure
 */
typedef struct  {
	/* Private */
	uint8_t MPU9250_Address; /*!< I2C address of MPU9250 device. */
	uint8_t AK8963_Address;  /*!< I2C address of AK8963 device. */
	float Gyro_Mult;         /*!< Gyroscope corrector from raw data to "degrees/s". Only for private use */
	float Gyro_biasx;
	float Gyro_biasy;
	float Gyro_biasz;
	float Gyro_testx;
	float Gyro_testy;
	float Gyro_testz;
	float Acce_Mult;         /*!< Accelerometer corrector from raw data to "g". Only for private use */
	float Acce_biasx;
	float Acce_biasy;
	float Acce_biasz;
	float Acce_testx;
	float Acce_testy;
	float Acce_testz;
	float Magn_Mult;         /*!< Magnetometer corrector from raw data to "mG". Only for private use */
	float Magn_biasx;
	float Magn_biasy;
	float Magn_biasz;
	float Magn_Calix;				 /*!< Magnetometer x-axis calibration. Only for private use */
	float Magn_Caliy;				 /*!< Magnetometer y-axis calibration. Only for private use */
	float Magn_Caliz;				 /*!< Magnetometer z-axis calibration. Only for private use */
	/* Public */
	int16_t Accelerometer_X; /*!< Accelerometer value X axis */
	int16_t Accelerometer_Y; /*!< Accelerometer value Y axis */
	int16_t Accelerometer_Z; /*!< Accelerometer value Z axis */
	int16_t Gyroscope_X;     /*!< Gyroscope value X axis */
	int16_t Gyroscope_Y;     /*!< Gyroscope value Y axis */
	int16_t Gyroscope_Z;     /*!< Gyroscope value Z axis */
	int16_t Magnetometer_X;  /*!< Magnetometer value X axis */
	int16_t Magnetometer_Y;  /*!< Magnetometer value Y axis */
	int16_t Magnetometer_Z;  /*!< Magnetometer value Z axis */
	float   Temperature;       /*!< Temperature in degrees */
	//I2C_HandleTypeDef* I2Cx;
} MPU9250;

/**
 * @brief  Interrupts union and structure
 */
typedef union {
	struct {
		uint8_t DataReady:1;       /*!< Data ready interrupt */
		uint8_t reserved2:2;       /*!< Reserved bits */
		uint8_t Master:1;          /*!< Master interrupt. Not enabled with library */
		uint8_t FifoOverflow:1;    /*!< FIFO overflow interrupt. Not enabled with library */
		uint8_t reserved1:1;       /*!< Reserved bit */
		uint8_t MotionDetection:1; /*!< Motion detected interrupt */
		uint8_t reserved0:1;       /*!< Reserved bit */
	} F;
	uint8_t Status;
} MPU9250_Interrupt;


/**
 * @}
 */

/**
 * @defgroup MPU9250_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Detect MPU9250
 * @param  *DataStruct: Pointer to empty @ref MPU9250_t structure
 * @param  DeviceNumber: MPU9250 has one pin, AD0 which can be used to set address of device.
 *          This feature allows you to use 2 different sensors on the same board with same library.
 *          If you set AD0 pin to low, then this parameter should be MPU9250_Device_0,
 *          but if AD0 pin is high, then you should use MPU9250_Device_1
 *
 *          Parameter can be a value of @ref MPU9250_Device_t enumeration
 * @retval Initialization status:
 *            - MPU9250_Result_t: Everything OK
 *            - Other member: in other cases
 */

MPU9250_Result MPU9250_Detect(I2C_HandleTypeDef* I2Cx,
														MPU9250* DataStruct,
														MPU9250_Device DeviceNumber);

/**
 * @brief  Initializes MPU9250 and I2C peripheral
 * @param  *DataStruct: Pointer to empty @ref MPU9250_t structure
 * @param  DeviceNumber: MPU9250 has one pin, AD0 which can be used to set address of device.
 *          This feature allows you to use 2 different sensors on the same board with same library.
 *          If you set AD0 pin to low, then this parameter should be MPU9250_Device_0,
 *          but if AD0 pin is high, then you should use MPU9250_Device_1
 *
 *          Parameter can be a value of @ref MPU9250_Device_t enumeration
 * @param  AccelerometerSensitivity: Set accelerometer sensitivity. This parameter can be a value of @ref MPU9250_Accelerometer_t enumeration
 * @param  GyroscopeSensitivity: Set gyroscope sensitivity. This parameter can be a value of @ref MPU9250_Gyroscope_t enumeration
 * @param  MagnetometerSensitivity: Set magnetometer sensitivity. This parameter can be a value of @ref MPU9250_Magnetometer_t enumeration
 * @retval Initialization status:
 *            - MPU9250_Result_t: Everything OK
 *            - Other member: in other cases
 */
MPU9250_Result MPU9250_Init(I2C_HandleTypeDef* I2Cx,
														MPU9250* DataStruct,
														MPU9250_Device DeviceNumber,
														MPU9250_Accelerometer AccelerometerSensitivity,
														MPU9250_Gyroscope GyroscopeSensitivity,
														MPU9250_Magnetometer MagnetometerSensitivity);

/**
 * @brief  Sets gyroscope sensitivity
 * @param  *DataStruct: Pointer to @ref MPU9250_t structure indicating MPU9250 device
 * @param  GyroscopeSensitivity: Gyro sensitivity value. This parameter can be a value of @ref MPU9250_Gyroscope_t enumeration
 * @retval Member of @ref MPU9250_Result_t enumeration
 */
MPU9250_Result MPU9250_SetGyroscope(I2C_HandleTypeDef* I2Cx, MPU9250* DataStruct,	MPU9250_Gyroscope GyroscopeSensitivity);

/**
 * @brief  Sets accelerometer sensitivity
 * @param  *DataStruct: Pointer to @ref MPU9250_t structure indicating MPU9250 device
 * @param  AccelerometerSensitivity: Gyro sensitivity value. This parameter can be a value of @ref MPU9250_Accelerometer_t enumeration
 * @retval Member of @ref MPU9250_Result_t enumeration
 */
MPU9250_Result MPU9250_SetAccelerometer(I2C_HandleTypeDef* I2Cx, MPU9250* DataStruct,	MPU9250_Accelerometer AccelerometerSensitivity);

/**
 * @brief  Sets magnetometer sensitivity
 * @param  *DataStruct: Pointer to @ref MPU9250_t structure indicating MPU9250 device
 * @param  MagnetometerSensitivity: Magn sensitivity value. This parameter can be a value of @ref MPU9250_Magnetometer_t enumeration
 * @retval Member of @ref MPU9250_Result_t enumeration
 */
MPU9250_Result MPU9250_SetMagnetometer(I2C_HandleTypeDef* I2Cx, MPU9250* DataStruct, MPU9250_Magnetometer MagnetometerSensitivity);

/**
 * @brief  Sets output data rate
 * @param  *DataStruct: Pointer to @ref MPU9250_t structure indicating MPU9250 device
 * @param  rate: Data rate value. An 8-bit value for prescaler value
 * @retval Member of @ref MPU9250_Result_t enumeration
 */
MPU9250_Result MPU9250_SetDataRate(I2C_HandleTypeDef* I2Cx,MPU9250* DataStruct, uint8_t rate);


/**
 * @brief  Enables interrupts
 * @param  *DataStruct: Pointer to @ref MPU9250_t structure indicating MPU9250 device
 * @retval Member of @ref MPU9250_Result_t enumeration
 */
MPU9250_Result MPU9250_EnableInterrupts(I2C_HandleTypeDef* I2Cx,MPU9250* DataStruct);

/**
 * @brief  Disables interrupts
 * @param  *DataStruct: Pointer to @ref MPU9250_t structure indicating MPU9250 device
 * @retval Member of @ref MPU9250_Result_t enumeration
 */
MPU9250_Result MPU9250_DisableInterrupts(I2C_HandleTypeDef* I2Cx,MPU9250* DataStruct);

/**
 * @brief  Reads and clears interrupts
 * @param  *DataStruct: Pointer to @ref MPU9250_t structure indicating MPU9250 device
 * @param  *InterruptsStruct: Pointer to @ref MPU9250_Interrupt_t structure to store status in
 * @retval Member of @ref MPU9250_Result_t enumeration
 */
MPU9250_Result MPU9250_ReadInterrupts(I2C_HandleTypeDef* I2Cx,MPU9250* DataStruct, MPU9250_Interrupt* InterruptsStruct);

/**
 * @brief  Reads accelerometer data from sensor
 * @param  *DataStruct: Pointer to @ref MPU9250_t structure to store data to
 * @retval Member of @ref MPU9250_Result_t:
 *            - MPU9250_Result_Ok: everything is OK
 *            - Other: in other cases
 */
MPU9250_Result MPU9250_ReadAccelerometer(I2C_HandleTypeDef* I2Cx,MPU9250* DataStruct);

/**
 * @brief  Reads gyroscope data from sensor
 * @param  *DataStruct: Pointer to @ref MPU9250_t structure to store data to
 * @retval Member of @ref MPU9250_Result_t:
 *            - MPU9250_Result_Ok: everything is OK
 *            - Other: in other cases
 */
MPU9250_Result MPU9250_ReadGyroscope(I2C_HandleTypeDef* I2Cx,MPU9250* DataStruct);

/**
 * @brief  Reads magnetometer data from sensor
 * @param  *DataStruct: Pointer to @ref MPU9250_t structure to store data to
 * @retval Member of @ref MPU9250_Result_t:
 *            - MPU9250_Result_Ok: everything is OK
 *            - Other: in other cases
 */
MPU9250_Result MPU9250_ReadMagnetometer(I2C_HandleTypeDef* I2Cx,MPU9250* DataStruct);

/**
 * @brief  Reads temperature data from sensor
 * @param  *DataStruct: Pointer to @ref MPU9250_t structure to store data to
 * @retval Member of @ref MPU9250_Result_t:
 *            - MPU9250_Result_Ok: everything is OK
 *            - Other: in other cases
 */
MPU9250_Result MPU9250_ReadTemperature(I2C_HandleTypeDef* I2Cx,MPU9250* DataStruct);

/**
 * @brief  Reads accelerometer, gyroscope, temperature and magnetometer data from sensor
 * @param  *DataStruct: Pointer to @ref MPU9250_t structure to store data to
 * @retval Member of @ref MPU9250_Result_t:
 *            - MPU9250_Result_Ok: everything is OK
 *            - Other: in other cases
 */
MPU9250_Result MPU9250_ReadAll(I2C_HandleTypeDef* I2Cx,MPU9250* DataStruct);

/**
 * @brief  Calibrate accelerometer, gyroscope
 * @param  *DataStruct: Pointer to @ref MPU9250_t structure to store data to
 * @retval Member of @ref MPU9250_Result_t:
 *            - MPU9250_Result_Ok: everything is OK
 *            - Other: in other cases
 */
MPU9250_Result MPU9250_Calibrate(I2C_HandleTypeDef* I2Cx,MPU9250* DataStruct);

/**
 * @brief  Test accelerometer, gyroscope
 * @param  *DataStruct: Pointer to @ref MPU9250_t structure to store data to
 * @retval Member of @ref MPU9250_Result_t:
 *            - MPU9250_Result_Ok: everything is OK
 *            - Other: in other cases
 */
MPU9250_Result MPU9250_Selftest(I2C_HandleTypeDef* I2Cx,MPU9250* DataStruct);

/**
 * @brief  Calibrate magnetometer
 * @param  *DataStruct: Pointer to @ref MPU9250_t structure to store data to
 * @retval Member of @ref MPU9250_Result_t:
 *            - MPU9250_Result_Ok: everything is OK
 *            - Other: in other cases
 */
MPU9250_Result AK8963_Calibration(I2C_HandleTypeDef* I2Cx,MPU9250* DataStruct);

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */


#endif /* __MPU9250_H_ */
