/*
 * pid_flash_storage.c
 *
 *  Created on: Jun 30, 2026
 *      Author: Eylül Öztek
 */

#include "pid_flash_storage.h"
#include <string.h>

/**
 * @brief Raw Flash record layout.
 */
typedef struct {
	uint32_t magic;
	uint32_t version;
	uint32_t size;
	PIDFlashStorage_Config_t config;
	uint32_t checksum;
} PIDFlashStorage_Record_t;

/**
 * @brief Calculate a simple FNV-1a checksum.
 *
 * @param data Pointer to byte buffer.
 * @param length Buffer length in bytes.
 * @return 32-bit checksum value.
 */
static uint32_t PIDFlashStorage_CalculateChecksum(const uint8_t *data,
		uint32_t length) {
	uint32_t hash = 2166136261UL;

	for (uint32_t i = 0; i < length; i++) {
		hash ^= data[i];
		hash *= 16777619UL;
	}

	return hash;
}

/**
 * @brief Return the raw record pointer mapped to Flash memory.
 *
 * @return Pointer to stored Flash record.
 */
static const PIDFlashStorage_Record_t* PIDFlashStorage_GetStoredRecord(void) {
	return (const PIDFlashStorage_Record_t*) PID_FLASH_STORAGE_ADDRESS;
}

/**
 * @brief Check whether the storage sector looks erased.
 *
 * @return 1 if erased, otherwise 0.
 */
static uint8_t PIDFlashStorage_IsErased(void) {
	const uint32_t *flashWords = (const uint32_t*) PID_FLASH_STORAGE_ADDRESS;
	uint32_t wordCount = sizeof(PIDFlashStorage_Record_t) / sizeof(uint32_t);

	for (uint32_t i = 0; i < wordCount; i++) {
		if (flashWords[i] != 0xFFFFFFFFUL) {
			return 0U;
		}
	}

	return 1U;
}

/**
 * @brief Validate a raw Flash record.
 *
 * @param record Pointer to raw record.
 * @return Operation status.
 */
static PIDFlashStorage_Status_t PIDFlashStorage_ValidateRecord(
		const PIDFlashStorage_Record_t *record) {
	uint32_t calculatedChecksum;

	if (record == 0) {
		return PID_FLASH_STORAGE_NULL_PARAM;
	}

	if (PIDFlashStorage_IsErased()) {
		return PID_FLASH_STORAGE_EMPTY;
	}

	if (record->magic != PID_FLASH_STORAGE_MAGIC) {
		return PID_FLASH_STORAGE_INVALID;
	}

	if (record->version != PID_FLASH_STORAGE_VERSION) {
		return PID_FLASH_STORAGE_INVALID;
	}

	if (record->size != sizeof(PIDFlashStorage_Record_t)) {
		return PID_FLASH_STORAGE_INVALID;
	}

	if ((record->config.mode != PID_FLASH_STORAGE_MODE_HEATING)
			&& (record->config.mode != PID_FLASH_STORAGE_MODE_COOLING)) {
		return PID_FLASH_STORAGE_INVALID;
	}

	calculatedChecksum = PIDFlashStorage_CalculateChecksum(
			(const uint8_t*) record,
			(uint32_t) sizeof(PIDFlashStorage_Record_t) - sizeof(uint32_t));

	if (calculatedChecksum != record->checksum) {
		return PID_FLASH_STORAGE_INVALID;
	}

	return PID_FLASH_STORAGE_OK;
}

/**
 * @brief Erase the Flash sector used for PID storage.
 *
 * @return Operation status.
 */
static PIDFlashStorage_Status_t PIDFlashStorage_EraseSector(void) {
	FLASH_EraseInitTypeDef eraseInit;
	uint32_t sectorError = 0U;
	HAL_StatusTypeDef halStatus;

	eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
	eraseInit.Sector = PID_FLASH_STORAGE_SECTOR;
	eraseInit.NbSectors = 1U;
	eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	halStatus = HAL_FLASHEx_Erase(&eraseInit, &sectorError);

	if (halStatus != HAL_OK || sectorError != 0xFFFFFFFFUL) {
		return PID_FLASH_STORAGE_ERASE_ERROR;
	}

	return PID_FLASH_STORAGE_OK;
}

PIDFlashStorage_Status_t PIDFlashStorage_Save(
		const PIDFlashStorage_Config_t *config) {
	PIDFlashStorage_Record_t record;
	PIDFlashStorage_Status_t status;
	HAL_StatusTypeDef halStatus;
	const uint32_t *sourceWords;
	uint32_t address;
	uint32_t wordCount;

	if (config == 0) {
		return PID_FLASH_STORAGE_NULL_PARAM;
	}

	if ((config->mode != PID_FLASH_STORAGE_MODE_HEATING)
			&& (config->mode != PID_FLASH_STORAGE_MODE_COOLING)) {
		return PID_FLASH_STORAGE_INVALID;
	}

	memset(&record, 0, sizeof(record));
	record.magic = PID_FLASH_STORAGE_MAGIC;
	record.version = PID_FLASH_STORAGE_VERSION;
	record.size = sizeof(PIDFlashStorage_Record_t);
	record.config = *config;
	record.checksum = PIDFlashStorage_CalculateChecksum((const uint8_t*) &record,
			(uint32_t) sizeof(PIDFlashStorage_Record_t) - sizeof(uint32_t));

	halStatus = HAL_FLASH_Unlock();
	if (halStatus != HAL_OK) {
		return PID_FLASH_STORAGE_WRITE_ERROR;
	}

	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR
			| FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	status = PIDFlashStorage_EraseSector();
	if (status != PID_FLASH_STORAGE_OK) {
		HAL_FLASH_Lock();
		return status;
	}

	sourceWords = (const uint32_t*) &record;
	wordCount = sizeof(PIDFlashStorage_Record_t) / sizeof(uint32_t);
	address = PID_FLASH_STORAGE_ADDRESS;

	for (uint32_t i = 0; i < wordCount; i++) {
		halStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address,
				sourceWords[i]);

		if (halStatus != HAL_OK) {
			HAL_FLASH_Lock();
			return PID_FLASH_STORAGE_WRITE_ERROR;
		}

		address += sizeof(uint32_t);
	}

	HAL_FLASH_Lock();

	status = PIDFlashStorage_Load(&record.config);
	if (status != PID_FLASH_STORAGE_OK) {
		return PID_FLASH_STORAGE_VERIFY_ERROR;
	}

	return PID_FLASH_STORAGE_OK;
}

PIDFlashStorage_Status_t PIDFlashStorage_Load(PIDFlashStorage_Config_t *config) {
	const PIDFlashStorage_Record_t *record = PIDFlashStorage_GetStoredRecord();
	PIDFlashStorage_Status_t status;

	if (config == 0) {
		return PID_FLASH_STORAGE_NULL_PARAM;
	}

	status = PIDFlashStorage_ValidateRecord(record);
	if (status != PID_FLASH_STORAGE_OK) {
		return status;
	}

	*config = record->config;

	return PID_FLASH_STORAGE_OK;
}

PIDFlashStorage_Status_t PIDFlashStorage_Clear(void) {
	PIDFlashStorage_Status_t status;
	HAL_StatusTypeDef halStatus;

	halStatus = HAL_FLASH_Unlock();
	if (halStatus != HAL_OK) {
		return PID_FLASH_STORAGE_ERASE_ERROR;
	}

	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR
			| FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	status = PIDFlashStorage_EraseSector();

	HAL_FLASH_Lock();

	return status;
}

uint8_t PIDFlashStorage_HasValidConfig(void) {
	PIDFlashStorage_Config_t config;
	PIDFlashStorage_Status_t status = PIDFlashStorage_Load(&config);

	return (status == PID_FLASH_STORAGE_OK) ? 1U : 0U;
}

const char* PIDFlashStorage_StatusToString(PIDFlashStorage_Status_t status) {
	switch (status) {
	case PID_FLASH_STORAGE_OK:
		return "OK";

	case PID_FLASH_STORAGE_EMPTY:
		return "EMPTY";

	case PID_FLASH_STORAGE_INVALID:
		return "INVALID";

	case PID_FLASH_STORAGE_NULL_PARAM:
		return "NULL_PARAM";

	case PID_FLASH_STORAGE_ERASE_ERROR:
		return "ERASE_ERROR";

	case PID_FLASH_STORAGE_WRITE_ERROR:
		return "WRITE_ERROR";

	case PID_FLASH_STORAGE_VERIFY_ERROR:
		return "VERIFY_ERROR";

	default:
		return "UNKNOWN";
	}
}
