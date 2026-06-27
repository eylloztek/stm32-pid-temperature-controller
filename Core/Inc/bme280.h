#ifndef BME280_H_
#define BME280_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/**
 * @file bme280.h
 * @author Eylül Öztek
 * @brief STM32 HAL based BME280 temperature, pressure, humidity and altitude driver.
 *
 * @details
 * This driver provides a simple STM32 HAL based interface for the Bosch BME280
 * environmental sensor over I2C. It supports:
 *
 * - Sensor initialization with default or user-defined configuration
 * - Normal mode and forced mode operation
 * - Raw ADC data reading
 * - Compensated temperature, pressure, and humidity reading
 * - Altitude calculation from pressure
 *
 * @note
 * This driver expects STM32 HAL shifted I2C addresses.
 * Use BME280_I2C_ADDR_GND or BME280_I2C_ADDR_VDDIO instead of raw 7-bit
 * addresses.
 */

/* -------------------------------------------------------------------------- */
/*                              I2C ADDRESSES                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief BME280 7-bit I2C address when SDO is connected to GND.
 */
#define BME280_I2C_ADDR_GND_7BIT       0x76

/**
 * @brief BME280 7-bit I2C address when SDO is connected to VDDIO.
 */
#define BME280_I2C_ADDR_VDDIO_7BIT     0x77

/**
 * @brief STM32 HAL shifted I2C address when SDO is connected to GND.
 */
#define BME280_I2C_ADDR_GND            (BME280_I2C_ADDR_GND_7BIT << 1)

/**
 * @brief STM32 HAL shifted I2C address when SDO is connected to VDDIO.
 */
#define BME280_I2C_ADDR_VDDIO          (BME280_I2C_ADDR_VDDIO_7BIT << 1)

/* -------------------------------------------------------------------------- */
/*                              REGISTER MAP                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Chip identification register.
 */
#define BME280_REG_ID                  0xD0

/**
 * @brief Soft reset register.
 */
#define BME280_REG_RESET               0xE0

/**
 * @brief Humidity oversampling control register.
 */
#define BME280_REG_CTRL_HUM            0xF2

/**
 * @brief Device status register.
 */
#define BME280_REG_STATUS              0xF3

/**
 * @brief Temperature, pressure oversampling and mode control register.
 */
#define BME280_REG_CTRL_MEAS           0xF4

/**
 * @brief Standby time, IIR filter and SPI mode configuration register.
 */
#define BME280_REG_CONFIG              0xF5

/**
 * @brief Pressure data MSB register.
 */
#define BME280_REG_PRESS_MSB           0xF7

/**
 * @brief Pressure data LSB register.
 */
#define BME280_REG_PRESS_LSB           0xF8

/**
 * @brief Pressure data XLSB register.
 */
#define BME280_REG_PRESS_XLSB          0xF9

/**
 * @brief Temperature data MSB register.
 */
#define BME280_REG_TEMP_MSB            0xFA

/**
 * @brief Temperature data LSB register.
 */
#define BME280_REG_TEMP_LSB            0xFB

/**
 * @brief Temperature data XLSB register.
 */
#define BME280_REG_TEMP_XLSB           0xFC

/**
 * @brief Humidity data MSB register.
 */
#define BME280_REG_HUM_MSB             0xFD

/**
 * @brief Humidity data LSB register.
 */
#define BME280_REG_HUM_LSB             0xFE

/**
 * @brief First calibration data block start address.
 */
#define BME280_REG_CALIB00             0x88

/**
 * @brief Second humidity calibration data block start address.
 */
#define BME280_REG_CALIB26             0xE1

/* -------------------------------------------------------------------------- */
/*                              CONSTANT VALUES                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief Expected BME280 chip ID value.
 */
#define BME280_CHIP_ID                 0x60

/**
 * @brief Soft reset command value.
 */
#define BME280_RESET_VALUE             0xB6

/**
 * @brief I2C communication timeout in milliseconds.
 */
#define BME280_TIMEOUT                 100U

/* -------------------------------------------------------------------------- */
/*                              STATUS REGISTER BITS                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief Status register bit indicating that NVM data is being copied.
 */
#define BME280_STATUS_IM_UPDATE        0x01

/**
 * @brief Status register bit indicating that a measurement is running.
 */
#define BME280_STATUS_MEASURING        0x08

/* -------------------------------------------------------------------------- */
/*                              REGISTER BIT POSITIONS                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief Bit position of temperature oversampling field in ctrl_meas register.
 */
#define BME280_CTRL_MEAS_OSRS_T_POS    5

/**
 * @brief Bit position of pressure oversampling field in ctrl_meas register.
 */
#define BME280_CTRL_MEAS_OSRS_P_POS    2

/**
 * @brief Bit position of mode field in ctrl_meas register.
 */
#define BME280_CTRL_MEAS_MODE_POS      0

/**
 * @brief Bit position of standby time field in config register.
 */
#define BME280_CONFIG_T_SB_POS         5

/**
 * @brief Bit position of IIR filter field in config register.
 */
#define BME280_CONFIG_FILTER_POS       2

/* -------------------------------------------------------------------------- */
/*                              OVERSAMPLING SETTINGS                         */
/* -------------------------------------------------------------------------- */

/**
 * @brief Disable measurement for selected channel.
 */
#define BME280_OVERSAMPLING_SKIPPED    0x00

/**
 * @brief Oversampling x1.
 */
#define BME280_OVERSAMPLING_1          0x01

/**
 * @brief Oversampling x2.
 */
#define BME280_OVERSAMPLING_2          0x02

/**
 * @brief Oversampling x4.
 */
#define BME280_OVERSAMPLING_4          0x03

/**
 * @brief Oversampling x8.
 */
#define BME280_OVERSAMPLING_8          0x04

/**
 * @brief Oversampling x16.
 */
#define BME280_OVERSAMPLING_16         0x05

/* -------------------------------------------------------------------------- */
/*                              SENSOR MODES                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Sleep mode. No measurement is performed.
 */
#define BME280_SLEEP_MODE              0x00

/**
 * @brief Forced mode. Performs one measurement and returns to sleep mode.
 */
#define BME280_FORCED_MODE             0x01

/**
 * @brief Normal mode. Performs cyclic measurements automatically.
 */
#define BME280_NORMAL_MODE             0x03

/* -------------------------------------------------------------------------- */
/*                              IIR FILTER SETTINGS                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Disable IIR filter.
 */
#define BME280_FILTER_OFF              0x00

/**
 * @brief IIR filter coefficient 2.
 */
#define BME280_FILTER_2                0x01

/**
 * @brief IIR filter coefficient 4.
 */
#define BME280_FILTER_4                0x02

/**
 * @brief IIR filter coefficient 8.
 */
#define BME280_FILTER_8                0x03

/**
 * @brief IIR filter coefficient 16.
 */
#define BME280_FILTER_16               0x04

/* -------------------------------------------------------------------------- */
/*                              STANDBY TIME SETTINGS                         */
/* -------------------------------------------------------------------------- */

/**
 * @brief Standby time 0.5 ms.
 */
#define BME280_STANDBY_TIME_0_5        0x00

/**
 * @brief Standby time 62.5 ms.
 */
#define BME280_STANDBY_TIME_62_5       0x01

/**
 * @brief Standby time 125 ms.
 */
#define BME280_STANDBY_TIME_125        0x02

/**
 * @brief Standby time 250 ms.
 */
#define BME280_STANDBY_TIME_250        0x03

/**
 * @brief Standby time 500 ms.
 */
#define BME280_STANDBY_TIME_500        0x04

/**
 * @brief Standby time 1000 ms.
 */
#define BME280_STANDBY_TIME_1000       0x05

/**
 * @brief Standby time 10 ms.
 */
#define BME280_STANDBY_TIME_10         0x06

/**
 * @brief Standby time 20 ms.
 */
#define BME280_STANDBY_TIME_20         0x07

/* -------------------------------------------------------------------------- */
/*                              TYPE DEFINITIONS                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief BME280 driver status codes.
 */
typedef enum {
	BME280_OK = 0, /**< Operation completed successfully. */
	BME280_ERROR, /**< HAL communication or generic driver error. */
	BME280_INVALID_ID, /**< Read chip ID does not match BME280. */
	BME280_INVALID_PARAM, /**< Invalid function parameter. */
	BME280_TIMEOUT_ERROR /**< Device did not become ready in expected time. */
} BME280_Status_t;

/**
 * @brief BME280 factory calibration parameters.
 *
 * @details
 * These values are read from the sensor NVM during initialization and are used
 * in Bosch compensation formulas for temperature, pressure, and humidity.
 *
 * @note
 * The user should not manually modify these values.
 */
typedef struct {
	uint16_t dig_T1; /**< Temperature calibration coefficient T1. */
	int16_t dig_T2; /**< Temperature calibration coefficient T2. */
	int16_t dig_T3; /**< Temperature calibration coefficient T3. */

	uint16_t dig_P1; /**< Pressure calibration coefficient P1. */
	int16_t dig_P2; /**< Pressure calibration coefficient P2. */
	int16_t dig_P3; /**< Pressure calibration coefficient P3. */
	int16_t dig_P4; /**< Pressure calibration coefficient P4. */
	int16_t dig_P5; /**< Pressure calibration coefficient P5. */
	int16_t dig_P6; /**< Pressure calibration coefficient P6. */
	int16_t dig_P7; /**< Pressure calibration coefficient P7. */
	int16_t dig_P8; /**< Pressure calibration coefficient P8. */
	int16_t dig_P9; /**< Pressure calibration coefficient P9. */

	uint8_t dig_H1; /**< Humidity calibration coefficient H1. */
	int16_t dig_H2; /**< Humidity calibration coefficient H2. */
	uint8_t dig_H3; /**< Humidity calibration coefficient H3. */
	int16_t dig_H4; /**< Humidity calibration coefficient H4. */
	int16_t dig_H5; /**< Humidity calibration coefficient H5. */
	int8_t dig_H6; /**< Humidity calibration coefficient H6. */
} BME280_Calibration_Parameters_t;

/**
 * @brief BME280 sensor configuration.
 *
 * @details
 * This structure stores oversampling, mode, standby time, and IIR filter
 * configuration values.
 *
 * @note
 * This driver reads compensated temperature, pressure, and humidity together.
 * Therefore, BME280_OVERSAMPLING_SKIPPED is rejected for osrs_t, osrs_p, and
 * osrs_h in BME280_ValidateConfig().
 */
typedef struct {
	uint8_t osrs_h; /**< Humidity oversampling setting. */
	uint8_t osrs_t; /**< Temperature oversampling setting. */
	uint8_t osrs_p; /**< Pressure oversampling setting. */
	uint8_t mode; /**< Sensor operating mode. */
	uint8_t standby_time; /**< Standby time used in normal mode. */
	uint8_t filter; /**< IIR filter coefficient. */
} BME280_Config_t;

/**
 * @brief Raw uncompensated BME280 ADC data.
 */
typedef struct {
	int32_t adc_temperature; /**< Raw 20-bit temperature ADC value. */
	int32_t adc_pressure; /**< Raw 20-bit pressure ADC value. */
	int32_t adc_humidity; /**< Raw 16-bit humidity ADC value. */
} BME280_RawData_t;

/**
 * @brief Compensated BME280 sensor data.
 */
typedef struct {
	float temperature; /**< Temperature in degrees Celsius. */
	float pressure; /**< Pressure in hPa. */
	float humidity; /**< Relative humidity in percent RH. */
} BME280_Data_t;

/**
 * @brief BME280 device handle.
 *
 * @details
 * This structure stores the STM32 HAL I2C handle, selected device address,
 * calibration parameters, last applied configuration, and internal fine
 * temperature value.
 *
 * @note
 * The user must create one handle per BME280 sensor.
 * Internal fields should not be modified manually after initialization.
 */
typedef struct {
	I2C_HandleTypeDef *hi2c; /**< STM32 HAL I2C handle. */
	uint16_t address; /**< Shifted I2C address for STM32 HAL. */
	BME280_Calibration_Parameters_t calib; /**< Factory calibration data. */
	BME280_Config_t config; /**< Last applied sensor configuration. */
	int32_t t_fine; /**< Fine temperature value used internally. */
} BME280_Handle_t;

/* -------------------------------------------------------------------------- */
/*                              PUBLIC FUNCTIONS                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize the BME280 sensor with default settings.
 *
 * @details
 * This function fills a default configuration structure internally and calls
 * BME280_InitWithConfig().
 *
 * Default settings:
 * - Humidity oversampling: x4
 * - Temperature oversampling: x1
 * - Pressure oversampling: x4
 * - Mode: normal mode
 * - Standby time: 1000 ms
 * - IIR filter coefficient: 4
 *
 * @param dev Pointer to BME280 handle.
 * @param hi2c Pointer to STM32 HAL I2C handle.
 * @param address Shifted I2C address for STM32 HAL.
 *
 * @return BME280_OK if initialization succeeds.
 * @return BME280_INVALID_PARAM if an input parameter is invalid.
 * @return BME280_ERROR if I2C communication fails.
 * @return BME280_INVALID_ID if the chip ID does not match BME280.
 * @return BME280_TIMEOUT_ERROR if the device does not become ready in time.
 */
BME280_Status_t BME280_Init(BME280_Handle_t *dev, I2C_HandleTypeDef *hi2c,
		uint16_t address);

/**
 * @brief Initialize the BME280 sensor with a user-defined configuration.
 *
 * @details
 * This function checks the I2C address, verifies device readiness, performs
 * a soft reset, checks the chip ID, reads calibration data, and applies the
 * user configuration.
 *
 * @param dev Pointer to BME280 handle.
 * @param hi2c Pointer to STM32 HAL I2C handle.
 * @param address Shifted I2C address for STM32 HAL.
 * @param config Pointer to user-defined configuration structure.
 *
 * @return BME280_OK if initialization succeeds.
 * @return BME280_INVALID_PARAM if an input parameter or configuration is invalid.
 * @return BME280_ERROR if I2C communication fails.
 * @return BME280_INVALID_ID if the chip ID does not match BME280.
 * @return BME280_TIMEOUT_ERROR if NVM copy does not complete in time.
 */
BME280_Status_t BME280_InitWithConfig(BME280_Handle_t *dev,
		I2C_HandleTypeDef *hi2c, uint16_t address,
		const BME280_Config_t *config);

/**
 * @brief Apply a new BME280 configuration.
 *
 * @details
 * The sensor is first placed into sleep mode before writing the config register.
 * This prevents ignored config writes while the sensor is in normal mode.
 *
 * @param dev Pointer to BME280 handle.
 * @param config Pointer to configuration structure.
 *
 * @return BME280_OK if configuration is applied successfully.
 * @return BME280_INVALID_PARAM if an input parameter or configuration is invalid.
 * @return BME280_ERROR if I2C communication fails.
 * @return BME280_TIMEOUT_ERROR if a measurement does not finish in time.
 */
BME280_Status_t BME280_SetConfig(BME280_Handle_t *dev,
		const BME280_Config_t *config);

/**
 * @brief Put the BME280 into sleep mode.
 *
 * @details
 * This function preserves the current oversampling settings and only changes
 * the mode bits to sleep mode.
 *
 * @param dev Pointer to BME280 handle.
 *
 * @return BME280_OK if sleep mode is set successfully.
 * @return BME280_INVALID_PARAM if dev or dev->hi2c is NULL.
 * @return BME280_ERROR if I2C communication fails.
 */
BME280_Status_t BME280_Sleep(BME280_Handle_t *dev);

/**
 * @brief Trigger one forced-mode measurement.
 *
 * @details
 * In forced mode, the sensor performs one measurement and then automatically
 * returns to sleep mode. Therefore, each new measurement must be triggered by
 * writing forced mode again.
 *
 * @param dev Pointer to BME280 handle.
 *
 * @return BME280_OK if the forced measurement is triggered successfully.
 * @return BME280_INVALID_PARAM if dev or dev->hi2c is NULL.
 * @return BME280_ERROR if I2C communication fails.
 */
BME280_Status_t BME280_TriggerForcedMeasurement(BME280_Handle_t *dev);

/**
 * @brief Wait until the current BME280 measurement is complete.
 *
 * @details
 * This function polls the measuring bit in the status register until the
 * measurement is complete or the timeout loop expires.
 *
 * @param dev Pointer to BME280 handle.
 *
 * @return BME280_OK if the measurement completes.
 * @return BME280_INVALID_PARAM if dev or dev->hi2c is NULL.
 * @return BME280_ERROR if I2C communication fails.
 * @return BME280_TIMEOUT_ERROR if measurement does not complete in time.
 */
BME280_Status_t BME280_WaitMeasurementComplete(BME280_Handle_t *dev);

/**
 * @brief Read raw uncompensated ADC values from the BME280.
 *
 * @details
 * This function performs a burst read from register 0xF7 to 0xFE and extracts
 * raw pressure, temperature, and humidity ADC values.
 *
 * @param dev Pointer to BME280 handle.
 * @param raw Pointer to raw data output structure.
 *
 * @return BME280_OK if raw data is read successfully.
 * @return BME280_INVALID_PARAM if an input parameter is invalid.
 * @return BME280_ERROR if I2C communication fails.
 */
BME280_Status_t BME280_ReadRawData(BME280_Handle_t *dev, BME280_RawData_t *raw);

/**
 * @brief Read compensated temperature, pressure, and humidity values.
 *
 * @details
 * This function reads raw ADC values, applies Bosch compensation formulas, and
 * returns floating-point temperature, pressure, and humidity values.
 *
 * If the device is configured in forced mode, this function automatically
 * triggers a new forced measurement, waits for completion, and then reads the
 * result.
 *
 * @param dev Pointer to BME280 handle.
 * @param data Pointer to compensated data output structure.
 *
 * @return BME280_OK if compensated data is read successfully.
 * @return BME280_INVALID_PARAM if an input parameter is invalid.
 * @return BME280_ERROR if I2C communication fails.
 * @return BME280_TIMEOUT_ERROR if forced measurement does not finish in time.
 */
BME280_Status_t BME280_ReadSensorData(BME280_Handle_t *dev, BME280_Data_t *data);

/**
 * @brief Read the BME280 chip ID register.
 *
 * @param dev Pointer to BME280 handle.
 * @param device_id Pointer to chip ID output variable.
 *
 * @return BME280_OK if chip ID is read successfully.
 * @return BME280_INVALID_PARAM if an input parameter is invalid.
 * @return BME280_ERROR if I2C communication fails.
 */
BME280_Status_t BME280_ReadDeviceID(BME280_Handle_t *dev, uint8_t *device_id);

/**
 * @brief Perform a BME280 soft reset.
 *
 * @details
 * This function writes the reset command to the reset register and waits until
 * the calibration data copy from NVM is complete.
 *
 * @param dev Pointer to BME280 handle.
 *
 * @return BME280_OK if reset completes successfully.
 * @return BME280_INVALID_PARAM if dev or dev->hi2c is NULL.
 * @return BME280_ERROR if I2C communication fails.
 * @return BME280_TIMEOUT_ERROR if NVM copy does not complete in time.
 */
BME280_Status_t BME280_SoftReset(BME280_Handle_t *dev);

/**
 * @brief Fill a configuration structure with default BME280 settings.
 *
 * @details
 * Default settings:
 * - Humidity oversampling: x4
 * - Temperature oversampling: x1
 * - Pressure oversampling: x4
 * - Mode: normal mode
 * - Standby time: 1000 ms
 * - IIR filter coefficient: 4
 *
 * @param config Pointer to configuration structure.
 *
 * @return BME280_OK if default configuration is written successfully.
 * @return BME280_INVALID_PARAM if config is NULL.
 */
BME280_Status_t BME280_GetDefaultConfig(BME280_Config_t *config);

/**
 * @brief Calculate altitude from pressure and sea-level pressure.
 *
 * @details
 * This function calculates approximate altitude using the barometric formula.
 * The result depends on the provided sea-level pressure value.
 *
 * @param pressure_hpa Measured pressure in hPa.
 * @param sea_level_hpa Sea-level pressure in hPa. Standard value is 1013.25 hPa.
 * @param altitude_m Pointer to altitude output in meters.
 *
 * @return BME280_OK if altitude is calculated successfully.
 * @return BME280_INVALID_PARAM if pressure_hpa, sea_level_hpa, or altitude_m is invalid.
 *
 * @note
 * This function uses powf(), so the math library may need to be linked.
 */
BME280_Status_t BME280_CalculateAltitude(float pressure_hpa,
		float sea_level_hpa, float *altitude_m);

#ifdef __cplusplus
}
#endif

#endif /* BME280_H_ */
