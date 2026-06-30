/*
 * pid_flash_storage.h
 *
 *  Created on: Jun 30, 2026
 *      Author: Eylül Öztek
 */

#ifndef INC_PID_FLASH_STORAGE_H_
#define INC_PID_FLASH_STORAGE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/**
 * @file pid_flash_storage.h
 * @brief Flash storage module for saving and loading PID controller settings.
 *
 * @details
 * This module stores the latest PID configuration in the last Flash sector of
 * STM32F446RE. The linker script must reserve this sector so that application
 * code is never linked into the same Flash area.
 *
 * Reserved Flash area for STM32F446RE:
 * - Sector 7
 * - Start address: 0x08060000
 * - Size: 128 KB
 */

#define PID_FLASH_STORAGE_ADDRESS      0x08060000UL
#define PID_FLASH_STORAGE_SECTOR       FLASH_SECTOR_7
#define PID_FLASH_STORAGE_SIZE_BYTES   (128UL * 1024UL)

#define PID_FLASH_STORAGE_MAGIC        0x50494453UL /* 'PIDS' */
#define PID_FLASH_STORAGE_VERSION      1UL

/**
 * @brief Flash storage operation status.
 */
typedef enum {
	PID_FLASH_STORAGE_OK = 0,
	PID_FLASH_STORAGE_EMPTY,
	PID_FLASH_STORAGE_INVALID,
	PID_FLASH_STORAGE_NULL_PARAM,
	PID_FLASH_STORAGE_ERASE_ERROR,
	PID_FLASH_STORAGE_WRITE_ERROR,
	PID_FLASH_STORAGE_VERIFY_ERROR
} PIDFlashStorage_Status_t;

/**
 * @brief Stored control mode value.
 *
 * @note These values are intentionally aligned with the project control mode:
 * HEATING = 0, COOLING = 1.
 */
typedef enum {
	PID_FLASH_STORAGE_MODE_HEATING = 0,
	PID_FLASH_STORAGE_MODE_COOLING = 1
} PIDFlashStorage_Mode_t;

/**
 * @brief User-facing PID configuration stored in Flash.
 */
typedef struct {
	float kp;       /**< Proportional gain. */
	float ki;       /**< Integral gain. */
	float kd;       /**< Derivative gain. */
	float setpoint; /**< Temperature setpoint in degrees Celsius. */
	uint32_t mode;  /**< Control mode: PID_FLASH_STORAGE_MODE_HEATING or PID_FLASH_STORAGE_MODE_COOLING. */
} PIDFlashStorage_Config_t;

/**
 * @brief Save a PID configuration to STM32 Flash.
 *
 * @param config Pointer to the PID configuration to save.
 * @return Operation status.
 */
PIDFlashStorage_Status_t PIDFlashStorage_Save(
		const PIDFlashStorage_Config_t *config);

/**
 * @brief Load a PID configuration from STM32 Flash.
 *
 * @param config Pointer to the output configuration structure.
 * @return PID_FLASH_STORAGE_OK if a valid configuration is loaded.
 * @return PID_FLASH_STORAGE_EMPTY if no saved configuration exists.
 * @return PID_FLASH_STORAGE_INVALID if the saved data fails validation.
 */
PIDFlashStorage_Status_t PIDFlashStorage_Load(PIDFlashStorage_Config_t *config);

/**
 * @brief Clear the Flash sector used for PID storage.
 *
 * @return Operation status.
 */
PIDFlashStorage_Status_t PIDFlashStorage_Clear(void);

/**
 * @brief Check whether a valid PID configuration exists in Flash.
 *
 * @return 1 if a valid configuration exists, otherwise 0.
 */
uint8_t PIDFlashStorage_HasValidConfig(void);

/**
 * @brief Convert storage status to a string for UART/debug messages.
 *
 * @param status Storage status value.
 * @return Constant string representation.
 */
const char* PIDFlashStorage_StatusToString(PIDFlashStorage_Status_t status);

#ifdef __cplusplus
}
#endif

#endif /* INC_PID_FLASH_STORAGE_H_ */
