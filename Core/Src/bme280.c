/**
 * @file bme280.c
 * @author Eylül Öztek
 * @brief STM32 HAL based BME280 driver implementation.
 *
 * @details
 * This file implements I2C communication, initialization, configuration,
 * raw data reading, compensation formulas, forced-mode handling, and altitude
 * calculation for the BME280 environmental sensor.
 */

#include "bme280.h"
#include <math.h>

/**
 * @brief Read one or more bytes from a BME280 register.
 *
 * @param dev Pointer to BME280 handle.
 * @param reg Register address to read from.
 * @param data Pointer to output data buffer.
 * @param length Number of bytes to read.
 *
 * @return BME280_OK if read operation succeeds.
 * @return BME280_INVALID_PARAM if an input parameter is invalid.
 * @return BME280_ERROR if I2C read operation fails.
 */
static BME280_Status_t BME280_ReadRegister(BME280_Handle_t *dev, uint8_t reg,
		uint8_t *data, uint16_t length);

/**
 * @brief Write one byte to a BME280 register.
 *
 * @param dev Pointer to BME280 handle.
 * @param reg Register address to write to.
 * @param value Byte value to write.
 *
 * @return BME280_OK if write operation succeeds.
 * @return BME280_INVALID_PARAM if an input parameter is invalid.
 * @return BME280_ERROR if I2C write operation fails.
 */
static BME280_Status_t BME280_WriteRegister(BME280_Handle_t *dev, uint8_t reg,
		uint8_t value);

/**
 * @brief Check whether the BME280 responds on the configured I2C address.
 *
 * @param dev Pointer to BME280 handle.
 *
 * @return BME280_OK if the device acknowledges the I2C address.
 * @return BME280_INVALID_PARAM if dev or dev->hi2c is NULL.
 * @return BME280_ERROR if the device does not respond.
 */
static BME280_Status_t BME280_IsDeviceReady(BME280_Handle_t *dev);

/**
 * @brief Read factory calibration data from the BME280.
 *
 * @param dev Pointer to BME280 handle.
 *
 * @return BME280_OK if calibration data is read successfully.
 * @return BME280_INVALID_PARAM if dev or dev->hi2c is NULL.
 * @return BME280_ERROR if I2C communication fails.
 */
static BME280_Status_t BME280_ReadCalibrationData(BME280_Handle_t *dev);

/**
 * @brief Wait until BME280 NVM data copy is complete.
 *
 * @param dev Pointer to BME280 handle.
 *
 * @return BME280_OK if NVM copy is complete.
 * @return BME280_INVALID_PARAM if dev or dev->hi2c is NULL.
 * @return BME280_ERROR if I2C communication fails.
 * @return BME280_TIMEOUT_ERROR if NVM copy does not complete in time.
 */
static BME280_Status_t BME280_WaitNVMReady(BME280_Handle_t *dev);

/**
 * @brief Validate a BME280 configuration structure.
 *
 * @param config Pointer to configuration structure.
 *
 * @return BME280_OK if configuration is valid.
 * @return BME280_INVALID_PARAM if configuration contains invalid values.
 */
static BME280_Status_t BME280_ValidateConfig(const BME280_Config_t *config);

/**
 * @brief Compensate raw temperature ADC value.
 *
 * @param dev Pointer to BME280 handle.
 * @param adc_temp Raw temperature ADC value.
 * @param fine_temp Pointer to fine temperature output.
 *
 * @return Compensated temperature in 0.01 degrees Celsius.
 */
static int32_t BME280_CompensateTemperature(BME280_Handle_t *dev,
		int32_t adc_temp, int32_t *fine_temp);

/**
 * @brief Compensate raw pressure ADC value.
 *
 * @param dev Pointer to BME280 handle.
 * @param adc_press Raw pressure ADC value.
 * @param fine_temp Fine temperature value from temperature compensation.
 *
 * @return Compensated pressure in Q24.8 Pa format.
 */
static uint32_t BME280_CompensatePressure(BME280_Handle_t *dev,
		int32_t adc_press, int32_t fine_temp);

/**
 * @brief Compensate raw humidity ADC value.
 *
 * @param dev Pointer to BME280 handle.
 * @param adc_hum Raw humidity ADC value.
 * @param fine_temp Fine temperature value from temperature compensation.
 *
 * @return Compensated humidity in Q22.10 percent RH format.
 */
static uint32_t BME280_CompensateHumidity(BME280_Handle_t *dev, int32_t adc_hum,
		int32_t fine_temp);

/**
 * @brief Read compensated sensor values in fixed-point format.
 *
 * @param dev Pointer to BME280 handle.
 * @param temperature Pointer to compensated temperature output in 0.01 degC.
 * @param pressure Pointer to compensated pressure output in Q24.8 Pa.
 * @param humidity Pointer to compensated humidity output in Q22.10 %RH.
 *
 * @return BME280_OK if fixed-point data is read successfully.
 * @return BME280_INVALID_PARAM if an input parameter is invalid.
 * @return BME280_ERROR if I2C communication fails.
 */
static BME280_Status_t BME280_ReadFixedPointData(BME280_Handle_t *dev,
		int32_t *temperature, uint32_t *pressure, uint32_t *humidity);

/**
 * @brief Convert two little-endian bytes to unsigned 16-bit value.
 *
 * @param lsb Least significant byte.
 * @param msb Most significant byte.
 *
 * @return Combined unsigned 16-bit value.
 */
static uint16_t BME280_U16_LE(uint8_t lsb, uint8_t msb) {
	return (uint16_t) (((uint16_t) msb << 8) | lsb);
}

/**
 * @brief Convert two little-endian bytes to signed 16-bit value.
 *
 * @param lsb Least significant byte.
 * @param msb Most significant byte.
 *
 * @return Combined signed 16-bit value.
 */
static int16_t BME280_S16_LE(uint8_t lsb, uint8_t msb) {
	return (int16_t) BME280_U16_LE(lsb, msb);
}

/**
 * @brief Sign-extend a 12-bit signed value to 16-bit signed format.
 *
 * @param value Raw 12-bit value stored in the lower bits.
 *
 * @return Sign-extended signed 16-bit value.
 */
static int16_t BME280_SignExtend12(uint16_t value) {
	if (value & 0x0800U) {
		value |= 0xF000U;
	}

	return (int16_t) value;
}

static BME280_Status_t BME280_ReadRegister(BME280_Handle_t *dev, uint8_t reg,
		uint8_t *data, uint16_t length) {
	if (dev == NULL || dev->hi2c == NULL || data == NULL || length == 0U) {
		return BME280_INVALID_PARAM;
	}

	if (HAL_I2C_Mem_Read(dev->hi2c, dev->address, reg,
	I2C_MEMADD_SIZE_8BIT, data, length,
	BME280_TIMEOUT) != HAL_OK) {
		return BME280_ERROR;
	}

	return BME280_OK;
}

static BME280_Status_t BME280_WriteRegister(BME280_Handle_t *dev, uint8_t reg,
		uint8_t value) {
	if (dev == NULL || dev->hi2c == NULL) {
		return BME280_INVALID_PARAM;
	}

	if (HAL_I2C_Mem_Write(dev->hi2c, dev->address, reg,
	I2C_MEMADD_SIZE_8BIT, &value, 1,
	BME280_TIMEOUT) != HAL_OK) {
		return BME280_ERROR;
	}

	return BME280_OK;
}

static BME280_Status_t BME280_IsDeviceReady(BME280_Handle_t *dev) {
	if (dev == NULL || dev->hi2c == NULL) {
		return BME280_INVALID_PARAM;
	}

	if (HAL_I2C_IsDeviceReady(dev->hi2c, dev->address, 3,
	BME280_TIMEOUT) != HAL_OK) {
		return BME280_ERROR;
	}

	return BME280_OK;
}

/**
 * @brief Read the BME280 chip ID register.
 *
 * @details
 * This function reads the chip identification register located at address
 * BME280_REG_ID. A valid BME280 sensor should return BME280_CHIP_ID.
 *
 * @param dev Pointer to the BME280 device handle.
 * @param device_id Pointer to the variable where the chip ID will be stored.
 *
 * @return BME280_OK if the chip ID is read successfully.
 * @return BME280_INVALID_PARAM if dev, dev->hi2c, or device_id is NULL.
 * @return BME280_ERROR if the I2C register read operation fails.
 */
BME280_Status_t BME280_ReadDeviceID(BME280_Handle_t *dev, uint8_t *device_id) {
	if (dev == NULL || dev->hi2c == NULL || device_id == NULL) {
		return BME280_INVALID_PARAM;
	}

	return BME280_ReadRegister(dev, BME280_REG_ID, device_id, 1);
}

/**
 * @brief Perform a software reset on the BME280 sensor.
 *
 * @details
 * This function writes the BME280 soft reset command to the reset register.
 * After the reset command, the function waits until the sensor finishes copying
 * calibration parameters from NVM to internal registers.
 *
 * @param dev Pointer to the BME280 device handle.
 *
 * @return BME280_OK if the reset operation completes successfully.
 * @return BME280_INVALID_PARAM if dev or dev->hi2c is NULL.
 * @return BME280_ERROR if the reset command cannot be written.
 * @return BME280_TIMEOUT_ERROR if NVM copy does not finish in time.
 *
 * @note After a software reset, the sensor returns to sleep mode.
 */
BME280_Status_t BME280_SoftReset(BME280_Handle_t *dev) {
	BME280_Status_t status;

	if (dev == NULL || dev->hi2c == NULL) {
		return BME280_INVALID_PARAM;
	}

	status = BME280_WriteRegister(dev, BME280_REG_RESET, BME280_RESET_VALUE);
	if (status != BME280_OK) {
		return status;
	}

	/*
	 * After soft reset, calibration data is copied from NVM.
	 * A short delay avoids immediate communication after reset.
	 */
	HAL_Delay(2);

	return BME280_WaitNVMReady(dev);
}

static BME280_Status_t BME280_WaitNVMReady(BME280_Handle_t *dev) {
	BME280_Status_t status;
	uint8_t status_reg = 0;

	if (dev == NULL || dev->hi2c == NULL) {
		return BME280_INVALID_PARAM;
	}

	for (uint32_t i = 0; i < 100U; i++) {
		status = BME280_ReadRegister(dev, BME280_REG_STATUS, &status_reg, 1);
		if (status != BME280_OK) {
			return status;
		}

		if ((status_reg & BME280_STATUS_IM_UPDATE) == 0U) {
			return BME280_OK;
		}

		HAL_Delay(1);
	}

	return BME280_TIMEOUT_ERROR;
}

/**
 * @brief Wait until the current BME280 measurement is complete.
 *
 * @details
 * This function polls the measuring bit in the BME280 status register. The bit
 * remains set while a temperature, pressure, or humidity measurement is running.
 * The function returns when the measurement is complete or when the timeout
 * loop expires.
 *
 * @param dev Pointer to the BME280 device handle.
 *
 * @return BME280_OK if the measurement is complete.
 * @return BME280_INVALID_PARAM if dev or dev->hi2c is NULL.
 * @return BME280_ERROR if reading the status register fails.
 * @return BME280_TIMEOUT_ERROR if the measurement does not finish in time.
 */
BME280_Status_t BME280_WaitMeasurementComplete(BME280_Handle_t *dev) {
	BME280_Status_t status;
	uint8_t status_reg = 0;

	if (dev == NULL || dev->hi2c == NULL) {
		return BME280_INVALID_PARAM;
	}

	for (uint32_t i = 0; i < 100U; i++) {
		status = BME280_ReadRegister(dev, BME280_REG_STATUS, &status_reg, 1);
		if (status != BME280_OK) {
			return status;
		}

		if ((status_reg & BME280_STATUS_MEASURING) == 0U) {
			return BME280_OK;
		}

		HAL_Delay(1);
	}

	return BME280_TIMEOUT_ERROR;
}

static BME280_Status_t BME280_ReadCalibrationData(BME280_Handle_t *dev) {
	BME280_Status_t status;
	uint8_t calib1[26] = { 0 };
	uint8_t calib2[7] = { 0 };
	uint16_t raw_H4;
	uint16_t raw_H5;

	if (dev == NULL || dev->hi2c == NULL) {
		return BME280_INVALID_PARAM;
	}

	/*
	 * Calibration block 1:
	 * 0x88 ... 0xA1
	 */
	status = BME280_ReadRegister(dev, BME280_REG_CALIB00, calib1, 26);
	if (status != BME280_OK) {
		return status;
	}

	/*
	 * Calibration block 2:
	 * 0xE1 ... 0xE7
	 */
	status = BME280_ReadRegister(dev, BME280_REG_CALIB26, calib2, 7);
	if (status != BME280_OK) {
		return status;
	}

	dev->calib.dig_T1 = BME280_U16_LE(calib1[0], calib1[1]);
	dev->calib.dig_T2 = BME280_S16_LE(calib1[2], calib1[3]);
	dev->calib.dig_T3 = BME280_S16_LE(calib1[4], calib1[5]);

	dev->calib.dig_P1 = BME280_U16_LE(calib1[6], calib1[7]);
	dev->calib.dig_P2 = BME280_S16_LE(calib1[8], calib1[9]);
	dev->calib.dig_P3 = BME280_S16_LE(calib1[10], calib1[11]);
	dev->calib.dig_P4 = BME280_S16_LE(calib1[12], calib1[13]);
	dev->calib.dig_P5 = BME280_S16_LE(calib1[14], calib1[15]);
	dev->calib.dig_P6 = BME280_S16_LE(calib1[16], calib1[17]);
	dev->calib.dig_P7 = BME280_S16_LE(calib1[18], calib1[19]);
	dev->calib.dig_P8 = BME280_S16_LE(calib1[20], calib1[21]);
	dev->calib.dig_P9 = BME280_S16_LE(calib1[22], calib1[23]);

	dev->calib.dig_H1 = calib1[25];

	dev->calib.dig_H2 = BME280_S16_LE(calib2[0], calib2[1]);
	dev->calib.dig_H3 = calib2[2];

	/*
	 * dig_H4:
	 * 0xE4 contains dig_H4[11:4]
	 * 0xE5[3:0] contains dig_H4[3:0]
	 */
	raw_H4 = ((uint16_t) calib2[3] << 4) | ((uint16_t) calib2[4] & 0x0FU);

	/*
	 * dig_H5:
	 * 0xE5[7:4] contains dig_H5[3:0]
	 * 0xE6 contains dig_H5[11:4]
	 */
	raw_H5 = ((uint16_t) calib2[5] << 4) | ((uint16_t) calib2[4] >> 4);

	dev->calib.dig_H4 = BME280_SignExtend12(raw_H4);
	dev->calib.dig_H5 = BME280_SignExtend12(raw_H5);

	dev->calib.dig_H6 = (int8_t) calib2[6];

	return BME280_OK;
}

static BME280_Status_t BME280_ValidateConfig(const BME280_Config_t *config) {
	if (config == NULL) {
		return BME280_INVALID_PARAM;
	}

	if (config->osrs_h > BME280_OVERSAMPLING_16
			|| config->osrs_t > BME280_OVERSAMPLING_16
			|| config->osrs_p > BME280_OVERSAMPLING_16) {
		return BME280_INVALID_PARAM;
	}

	/*
	 * This driver reads compensated temperature, pressure, and humidity together.
	 * Therefore, all three measurement channels must be enabled.
	 */
	if (config->osrs_h == BME280_OVERSAMPLING_SKIPPED
			|| config->osrs_t == BME280_OVERSAMPLING_SKIPPED
			|| config->osrs_p == BME280_OVERSAMPLING_SKIPPED) {
		return BME280_INVALID_PARAM;
	}

	if (config->mode != BME280_SLEEP_MODE && config->mode != BME280_FORCED_MODE
			&& config->mode != BME280_NORMAL_MODE) {
		return BME280_INVALID_PARAM;
	}

	if (config->standby_time > BME280_STANDBY_TIME_20) {
		return BME280_INVALID_PARAM;
	}

	if (config->filter > BME280_FILTER_16) {
		return BME280_INVALID_PARAM;
	}

	return BME280_OK;
}

/**
 * @brief Initialize the BME280 sensor with default configuration.
 *
 * @details
 * This function creates a default configuration by calling
 * BME280_GetDefaultConfig() and then initializes the sensor using
 * BME280_InitWithConfig().
 *
 * Default configuration:
 * - Humidity oversampling: x4
 * - Temperature oversampling: x1
 * - Pressure oversampling: x4
 * - Mode: normal mode
 * - Standby time: 1000 ms
 * - IIR filter coefficient: 4
 *
 * @param dev Pointer to the BME280 device handle.
 * @param hi2c Pointer to the STM32 HAL I2C handle.
 * @param address Shifted I2C address used by STM32 HAL.
 *
 * @return BME280_OK if initialization is successful.
 * @return BME280_INVALID_PARAM if any input parameter is invalid.
 * @return BME280_ERROR if I2C communication fails.
 * @return BME280_INVALID_ID if the detected chip ID is not valid.
 * @return BME280_TIMEOUT_ERROR if the sensor does not become ready in time.
 */
BME280_Status_t BME280_Init(BME280_Handle_t *dev, I2C_HandleTypeDef *hi2c,
		uint16_t address) {
	BME280_Config_t default_config;

	BME280_GetDefaultConfig(&default_config);
	return BME280_InitWithConfig(dev, hi2c, address, &default_config);
}

/**
 * @brief Initialize the BME280 sensor with user-defined configuration.
 *
 * @details
 * This function performs the complete BME280 initialization sequence:
 * - Validate input parameters and configuration
 * - Store the I2C handle and device address
 * - Check if the device is ready on the I2C bus
 * - Perform a software reset
 * - Read and verify the chip ID
 * - Read factory calibration parameters
 * - Apply the requested sensor configuration
 *
 * @param dev Pointer to the BME280 device handle.
 * @param hi2c Pointer to the STM32 HAL I2C handle.
 * @param address Shifted I2C address used by STM32 HAL.
 * @param config Pointer to the user-defined configuration structure.
 *
 * @return BME280_OK if initialization is successful.
 * @return BME280_INVALID_PARAM if any input parameter or configuration is invalid.
 * @return BME280_ERROR if I2C communication fails.
 * @return BME280_INVALID_ID if the chip ID does not match BME280_CHIP_ID.
 * @return BME280_TIMEOUT_ERROR if reset or NVM copy does not finish in time.
 */
BME280_Status_t BME280_InitWithConfig(BME280_Handle_t *dev,
		I2C_HandleTypeDef *hi2c, uint16_t address,
		const BME280_Config_t *config) {
	BME280_Status_t status;
	uint8_t device_id = 0;

	if (dev == NULL || hi2c == NULL || config == NULL) {
		return BME280_INVALID_PARAM;
	}

	if (address != BME280_I2C_ADDR_GND && address != BME280_I2C_ADDR_VDDIO) {
		return BME280_INVALID_PARAM;
	}

	status = BME280_ValidateConfig(config);
	if (status != BME280_OK) {
		return status;
	}

	dev->hi2c = hi2c;
	dev->address = address;
	dev->t_fine = 0;

	status = BME280_IsDeviceReady(dev);
	if (status != BME280_OK) {
		return status;
	}

	status = BME280_SoftReset(dev);
	if (status != BME280_OK) {
		return status;
	}

	status = BME280_ReadDeviceID(dev, &device_id);
	if (status != BME280_OK) {
		return status;
	}

	if (device_id != BME280_CHIP_ID) {
		return BME280_INVALID_ID;
	}

	status = BME280_ReadCalibrationData(dev);
	if (status != BME280_OK) {
		return status;
	}

	status = BME280_SetConfig(dev, config);
	if (status != BME280_OK) {
		return status;
	}

	return BME280_OK;
}

/**
 * @brief Fill a configuration structure with default BME280 settings.
 *
 * @details
 * This function writes a known working default configuration into the provided
 * BME280_Config_t structure. The default configuration is suitable for general
 * environmental monitoring applications.
 *
 * Default values:
 * - Humidity oversampling: x4
 * - Temperature oversampling: x1
 * - Pressure oversampling: x4
 * - Mode: normal mode
 * - Standby time: 1000 ms
 * - IIR filter coefficient: 4
 *
 * @param config Pointer to the configuration structure to be filled.
 *
 * @return BME280_OK if default values are written successfully.
 * @return BME280_INVALID_PARAM if config is NULL.
 */
BME280_Status_t BME280_GetDefaultConfig(BME280_Config_t *config) {
	if (config == NULL) {
		return BME280_INVALID_PARAM;
	}

	config->osrs_h = BME280_OVERSAMPLING_4;
	config->osrs_t = BME280_OVERSAMPLING_1;
	config->osrs_p = BME280_OVERSAMPLING_4;
	config->mode = BME280_NORMAL_MODE;
	config->standby_time = BME280_STANDBY_TIME_1000;
	config->filter = BME280_FILTER_4;

	return BME280_OK;
}

/**
 * @brief Apply a new configuration to the BME280 sensor.
 *
 * @details
 * This function validates the provided configuration and writes it to the
 * BME280 control registers.
 *
 * The sensor is first placed into sleep mode before writing the config register,
 * because config register writes may be ignored while the sensor is operating
 * in normal mode.
 *
 * Register write order:
 * - ctrl_meas is written with sleep mode
 * - ctrl_hum is written with humidity oversampling
 * - config is written with standby time and IIR filter settings
 * - ctrl_meas is written with temperature/pressure oversampling and final mode
 *
 * @param dev Pointer to the BME280 device handle.
 * @param config Pointer to the configuration structure to apply.
 *
 * @return BME280_OK if the configuration is applied successfully.
 * @return BME280_INVALID_PARAM if dev, dev->hi2c, config, or config values are invalid.
 * @return BME280_ERROR if any I2C register write operation fails.
 * @return BME280_TIMEOUT_ERROR if an ongoing measurement does not finish in time.
 *
 * @note The applied configuration is stored in dev->config.
 */
BME280_Status_t BME280_SetConfig(BME280_Handle_t *dev,
		const BME280_Config_t *config) {
	BME280_Status_t status;
	uint8_t ctrl_hum;
	uint8_t ctrl_meas_sleep;
	uint8_t ctrl_meas_final;
	uint8_t config_reg;

	if (dev == NULL || dev->hi2c == NULL || config == NULL) {
		return BME280_INVALID_PARAM;
	}

	status = BME280_ValidateConfig(config);
	if (status != BME280_OK) {
		return status;
	}

	/*
	 * Put the sensor into sleep mode before writing the config register.
	 * Config writes may be ignored while the device is in normal mode.
	 */
	ctrl_meas_sleep = ((config->osrs_t & 0x07U) << BME280_CTRL_MEAS_OSRS_T_POS)
			| ((config->osrs_p & 0x07U) << BME280_CTRL_MEAS_OSRS_P_POS) |
			BME280_SLEEP_MODE;

	status = BME280_WriteRegister(dev, BME280_REG_CTRL_MEAS, ctrl_meas_sleep);
	if (status != BME280_OK) {
		return status;
	}

	status = BME280_WaitMeasurementComplete(dev);
	if (status != BME280_OK) {
		return status;
	}

	/*
	 * ctrl_hum register:
	 * bit 2:0 -> osrs_h
	 */
	ctrl_hum = config->osrs_h & 0x07U;

	/*
	 * config register:
	 * bit 7:5 -> t_sb
	 * bit 4:2 -> filter
	 * bit 0   -> spi3w_en, kept disabled for I2C
	 */
	config_reg = ((config->standby_time & 0x07U) << BME280_CONFIG_T_SB_POS)
			| ((config->filter & 0x07U) << BME280_CONFIG_FILTER_POS);

	/*
	 * ctrl_meas register:
	 * bit 7:5 -> osrs_t
	 * bit 4:2 -> osrs_p
	 * bit 1:0 -> mode
	 */
	ctrl_meas_final = ((config->osrs_t & 0x07U) << BME280_CTRL_MEAS_OSRS_T_POS)
			| ((config->osrs_p & 0x07U) << BME280_CTRL_MEAS_OSRS_P_POS)
			| ((config->mode & 0x03U) << BME280_CTRL_MEAS_MODE_POS);

	/*
	 * Humidity oversampling changes become effective after writing ctrl_meas.
	 * Therefore, ctrl_hum must be written before ctrl_meas.
	 */
	status = BME280_WriteRegister(dev, BME280_REG_CTRL_HUM, ctrl_hum);
	if (status != BME280_OK) {
		return status;
	}

	status = BME280_WriteRegister(dev, BME280_REG_CONFIG, config_reg);
	if (status != BME280_OK) {
		return status;
	}

	status = BME280_WriteRegister(dev, BME280_REG_CTRL_MEAS, ctrl_meas_final);
	if (status != BME280_OK) {
		return status;
	}

	dev->config = *config;

	return BME280_OK;
}

/**
 * @brief Put the BME280 sensor into sleep mode.
 *
 * @details
 * This function preserves the current temperature and pressure oversampling
 * settings and only clears the mode bits in the ctrl_meas register to enter
 * sleep mode.
 *
 * @param dev Pointer to the BME280 device handle.
 *
 * @return BME280_OK if the sensor is placed into sleep mode successfully.
 * @return BME280_INVALID_PARAM if dev or dev->hi2c is NULL.
 * @return BME280_ERROR if writing to the ctrl_meas register fails.
 *
 * @note dev->config.mode is updated to BME280_SLEEP_MODE after a successful write.
 */
BME280_Status_t BME280_Sleep(BME280_Handle_t *dev) {
	BME280_Status_t status;
	uint8_t ctrl_meas_sleep;

	if (dev == NULL || dev->hi2c == NULL) {
		return BME280_INVALID_PARAM;
	}

	ctrl_meas_sleep = ((dev->config.osrs_t & 0x07U)
			<< BME280_CTRL_MEAS_OSRS_T_POS)
			| ((dev->config.osrs_p & 0x07U) << BME280_CTRL_MEAS_OSRS_P_POS) |
			BME280_SLEEP_MODE;

	status = BME280_WriteRegister(dev, BME280_REG_CTRL_MEAS, ctrl_meas_sleep);
	if (status != BME280_OK) {
		return status;
	}

	dev->config.mode = BME280_SLEEP_MODE;

	return BME280_OK;
}

/**
 * @brief Trigger a single forced-mode measurement.
 *
 * @details
 * In forced mode, the BME280 performs one measurement cycle and then
 * automatically returns to sleep mode. Therefore, each new forced measurement
 * must be triggered by writing forced mode to the ctrl_meas register again.
 *
 * This function uses the oversampling settings stored in dev->config.
 *
 * @param dev Pointer to the BME280 device handle.
 *
 * @return BME280_OK if the forced measurement is triggered successfully.
 * @return BME280_INVALID_PARAM if dev or dev->hi2c is NULL.
 * @return BME280_ERROR if writing to the ctrl_meas register fails.
 */
BME280_Status_t BME280_TriggerForcedMeasurement(BME280_Handle_t *dev) {
	uint8_t ctrl_meas;

	if (dev == NULL || dev->hi2c == NULL) {
		return BME280_INVALID_PARAM;
	}

	/*
	 * In forced mode, every new measurement must be triggered by writing
	 * mode[1:0] = 01 again.
	 */
	ctrl_meas = ((dev->config.osrs_t & 0x07U) << BME280_CTRL_MEAS_OSRS_T_POS)
			| ((dev->config.osrs_p & 0x07U) << BME280_CTRL_MEAS_OSRS_P_POS) |
			BME280_FORCED_MODE;

	return BME280_WriteRegister(dev, BME280_REG_CTRL_MEAS, ctrl_meas);
}

/**
 * @brief Read raw uncompensated ADC values from the BME280.
 *
 * @details
 * This function performs a burst read starting from BME280_REG_PRESS_MSB.
 * It reads 8 consecutive bytes containing:
 * - 20-bit raw pressure ADC value
 * - 20-bit raw temperature ADC value
 * - 16-bit raw humidity ADC value
 *
 * The raw values are not temperature-compensated and should not be used directly
 * as physical sensor values.
 *
 * @param dev Pointer to the BME280 device handle.
 * @param raw Pointer to the raw data structure where ADC values will be stored.
 *
 * @return BME280_OK if raw data is read successfully.
 * @return BME280_INVALID_PARAM if dev, dev->hi2c, or raw is NULL.
 * @return BME280_ERROR if the burst read operation fails.
 */
BME280_Status_t BME280_ReadRawData(BME280_Handle_t *dev, BME280_RawData_t *raw) {
	BME280_Status_t status;
	uint8_t data[8] = { 0 };

	if (dev == NULL || dev->hi2c == NULL || raw == NULL) {
		return BME280_INVALID_PARAM;
	}

	/*
	 * Burst read from 0xF7 to 0xFE:
	 * pressure[19:0], temperature[19:0], humidity[15:0]
	 */
	status = BME280_ReadRegister(dev, BME280_REG_PRESS_MSB, data, 8);
	if (status != BME280_OK) {
		return status;
	}

	raw->adc_pressure = ((int32_t) data[0] << 12) | ((int32_t) data[1] << 4)
			| ((int32_t) data[2] >> 4);

	raw->adc_temperature = ((int32_t) data[3] << 12) | ((int32_t) data[4] << 4)
			| ((int32_t) data[5] >> 4);

	raw->adc_humidity = ((int32_t) data[6] << 8) | ((int32_t) data[7]);

	return BME280_OK;
}

static BME280_Status_t BME280_ReadFixedPointData(BME280_Handle_t *dev,
		int32_t *temperature, uint32_t *pressure, uint32_t *humidity) {
	BME280_Status_t status;
	BME280_RawData_t raw;
	int32_t fine_temp = 0;

	if (dev == NULL || dev->hi2c == NULL || temperature == NULL
			|| pressure == NULL || humidity == NULL) {
		return BME280_INVALID_PARAM;
	}

	status = BME280_ReadRawData(dev, &raw);
	if (status != BME280_OK) {
		return status;
	}

	*temperature = BME280_CompensateTemperature(dev, raw.adc_temperature,
			&fine_temp);

	dev->t_fine = fine_temp;

	*pressure = BME280_CompensatePressure(dev, raw.adc_pressure, fine_temp);

	*humidity = BME280_CompensateHumidity(dev, raw.adc_humidity, fine_temp);

	return BME280_OK;
}

/**
 * @brief Read compensated temperature, pressure, and humidity values.
 *
 * @details
 * This function reads BME280 sensor data and returns physical values as
 * floating-point numbers:
 *
 * - Temperature in degrees Celsius
 * - Pressure in hPa
 * - Humidity in percent RH
 *
 * If the sensor is configured in forced mode, this function automatically
 * triggers a new forced measurement, waits for the measurement to finish, and
 * then reads the data.
 *
 * @param dev Pointer to the BME280 device handle.
 * @param data Pointer to the compensated sensor data output structure.
 *
 * @return BME280_OK if sensor data is read and compensated successfully.
 * @return BME280_INVALID_PARAM if dev, dev->hi2c, or data is NULL.
 * @return BME280_ERROR if I2C communication fails.
 * @return BME280_TIMEOUT_ERROR if forced measurement does not finish in time.
 */
BME280_Status_t BME280_ReadSensorData(BME280_Handle_t *dev, BME280_Data_t *data) {
	BME280_Status_t status;
	int32_t fixed_temperature = 0;
	uint32_t fixed_pressure = 0;
	uint32_t fixed_humidity = 0;

	if (dev == NULL || dev->hi2c == NULL || data == NULL) {
		return BME280_INVALID_PARAM;
	}

	/*
	 * In forced mode, the device returns to sleep after one measurement.
	 * Therefore, trigger a new measurement before each read.
	 */
	if (dev->config.mode == BME280_FORCED_MODE) {
		status = BME280_TriggerForcedMeasurement(dev);
		if (status != BME280_OK) {
			return status;
		}

		status = BME280_WaitMeasurementComplete(dev);
		if (status != BME280_OK) {
			return status;
		}
	}

	status = BME280_ReadFixedPointData(dev, &fixed_temperature, &fixed_pressure,
			&fixed_humidity);
	if (status != BME280_OK) {
		return status;
	}

	/*
	 * Temperature: 0.01 degC
	 * Pressure: Q24.8 Pa
	 * Humidity: Q22.10 %RH
	 */
	data->temperature = (float) fixed_temperature / 100.0f;
	data->pressure = ((float) fixed_pressure / 256.0f) / 100.0f;
	data->humidity = (float) fixed_humidity / 1024.0f;

	return BME280_OK;
}

static int32_t BME280_CompensateTemperature(BME280_Handle_t *dev,
		int32_t adc_temp, int32_t *fine_temp) {
	int32_t var1;
	int32_t var2;
	int32_t temperature;

	if (dev == NULL || fine_temp == NULL) {
		return 0;
	}

	var1 = ((((adc_temp >> 3) - ((int32_t) dev->calib.dig_T1 << 1)))
			* ((int32_t) dev->calib.dig_T2)) >> 11;

	var2 = (((((adc_temp >> 4) - ((int32_t) dev->calib.dig_T1))
			* ((adc_temp >> 4) - ((int32_t) dev->calib.dig_T1))) >> 12)
			* ((int32_t) dev->calib.dig_T3)) >> 14;

	*fine_temp = var1 + var2;

	temperature = (*fine_temp * 5 + 128) >> 8;

	return temperature;
}

static uint32_t BME280_CompensatePressure(BME280_Handle_t *dev,
		int32_t adc_press, int32_t fine_temp) {
	int64_t var1;
	int64_t var2;
	int64_t pressure;

	if (dev == NULL) {
		return 0;
	}

	var1 = ((int64_t) fine_temp) - 128000;
	var2 = var1 * var1 * (int64_t) dev->calib.dig_P6;
	var2 = var2 + ((var1 * (int64_t) dev->calib.dig_P5) << 17);
	var2 = var2 + (((int64_t) dev->calib.dig_P4) << 35);

	var1 = ((var1 * var1 * (int64_t) dev->calib.dig_P3) >> 8)
			+ ((var1 * (int64_t) dev->calib.dig_P2) << 12);

	var1 = (((((int64_t) 1) << 47) + var1) * ((int64_t) dev->calib.dig_P1))
			>> 33;

	if (var1 == 0) {
		return 0;
	}

	pressure = 1048576 - adc_press;
	pressure = (((pressure << 31) - var2) * 3125) / var1;

	var1 = (((int64_t) dev->calib.dig_P9) * (pressure >> 13) * (pressure >> 13))
			>> 25;

	var2 = (((int64_t) dev->calib.dig_P8) * pressure) >> 19;

	pressure = ((pressure + var1 + var2) >> 8)
			+ (((int64_t) dev->calib.dig_P7) << 4);

	return (uint32_t) pressure;
}

static uint32_t BME280_CompensateHumidity(BME280_Handle_t *dev, int32_t adc_hum,
		int32_t fine_temp) {
	int32_t v_x1_u32r;

	if (dev == NULL) {
		return 0;
	}

	v_x1_u32r = fine_temp - ((int32_t) 76800);

	v_x1_u32r = (((((adc_hum << 14) - (((int32_t) dev->calib.dig_H4) << 20)
			- (((int32_t) dev->calib.dig_H5) * v_x1_u32r)) + ((int32_t) 16384))
			>> 15)
			* (((((((v_x1_u32r * ((int32_t) dev->calib.dig_H6)) >> 10)
					* (((v_x1_u32r * ((int32_t) dev->calib.dig_H3)) >> 11)
							+ ((int32_t) 32768))) >> 10) + ((int32_t) 2097152))
					* ((int32_t) dev->calib.dig_H2) + 8192) >> 14));

	v_x1_u32r = v_x1_u32r
			- (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7)
					* ((int32_t) dev->calib.dig_H1)) >> 4);

	v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
	v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;

	return (uint32_t) (v_x1_u32r >> 12);
}

/**
 * @brief Calculate altitude from pressure and sea-level pressure.
 *
 * @details
 * This function estimates altitude using the barometric formula. The result
 * depends on the measured pressure and the provided sea-level reference
 * pressure.
 *
 * A commonly used standard sea-level pressure value is 1013.25 hPa, but for
 * better accuracy this value should be adjusted according to local weather
 * conditions.
 *
 * @param pressure_hpa Measured pressure in hPa.
 * @param sea_level_hpa Reference sea-level pressure in hPa.
 * @param altitude_m Pointer to the calculated altitude value in meters.
 *
 * @return BME280_OK if altitude is calculated successfully.
 * @return BME280_INVALID_PARAM if altitude_m is NULL or pressure values are invalid.
 *
 * @note This function uses powf(). Depending on the toolchain, linking the math
 *       library with -lm may be required.
 */
BME280_Status_t BME280_CalculateAltitude(float pressure_hpa,
		float sea_level_hpa, float *altitude_m) {
	if (altitude_m == NULL || pressure_hpa <= 0.0f || sea_level_hpa <= 0.0f) {
		return BME280_INVALID_PARAM;
	}

	*altitude_m = 44330.0f
			* (1.0f - powf(pressure_hpa / sea_level_hpa, 0.1903f));

	return BME280_OK;
}
