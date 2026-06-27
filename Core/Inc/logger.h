/*
 * logger.h
 *
 *  Created on: Jul 20, 2025
 *      Author: Arif Mandal
 */

#ifndef INC_LOGGER_H_
#define INC_LOGGER_H_

#include "main.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>


typedef enum {
    LOG_LEVEL_NONE = 0,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_WARNING,
    LOG_LEVEL_INFO,
    LOG_LEVEL_DEBUG,
} LogLevel_t;

typedef struct {
    LogLevel_t currentLevel;
} Logger_t;


extern Logger_t logger;


void loggerInit(LogLevel_t level);
void loggerLog(LogLevel_t level, const char *format, ...);
void loggerSetLevel(LogLevel_t level);
int _write(int file, char *ptr, int len);


#define LOG_ERROR(...)   loggerLog(LOG_LEVEL_ERROR, __VA_ARGS__)
#define LOG_WARNING(...) loggerLog(LOG_LEVEL_WARNING, __VA_ARGS__)
#define LOG_INFO(...)    loggerLog(LOG_LEVEL_INFO, __VA_ARGS__)
#define LOG_DEBUG(...)   loggerLog(LOG_LEVEL_DEBUG, __VA_ARGS__)

#endif /* INC_LOGGER_H_ */
