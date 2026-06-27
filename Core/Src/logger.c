/*
 * logger.c
 *
 *  Created on: Jul 20, 2025
 *      Author: Arif Mandal
 */

#include "logger.h"

Logger_t logger;

int _write(int file, char *ptr, int len) {
	/* Implement your write code here, this is used by puts and printf for example */
	int i = 0;
	for (i = 0; i < len; i++)
		ITM_SendChar((*ptr++));
	return len;
}


void loggerInit(LogLevel_t level)
{
    logger.currentLevel = level;

    setvbuf(stdout, NULL, _IONBF, 0);
}

void loggerSetLevel(LogLevel_t level)
{
    logger.currentLevel = level;
}

static const char* loggerLevelToString(LogLevel_t level)
{
    switch (level) {
        case LOG_LEVEL_ERROR:   return "ERROR";
        case LOG_LEVEL_WARNING: return "WARN";
        case LOG_LEVEL_INFO:    return "INFO";
        case LOG_LEVEL_DEBUG:   return "DEBUG";
        default:                return "UNKNOWN";
    }
}

static void loggerPrintTimestamp(void)
{
    uint32_t ms = HAL_GetTick();
    uint32_t seconds = ms / 1000;
    uint32_t millis = ms % 1000;

   printf("[%5lu.%03lu] ", seconds, millis);
}

void loggerLog(LogLevel_t level, const char *format, ...)
{
    if (level > logger.currentLevel) {
        return;
    }

    loggerPrintTimestamp(); //optional 
	printf("[%s] ", loggerLevelToString(level));

    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);

    printf("\r\n");
}
