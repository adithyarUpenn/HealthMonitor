/**************************************************************************//**
* @file      SerialConsole.h
* @brief     Serial Console driver
* @author    Adi
* @date      2023-12-14

******************************************************************************/

#ifndef SERIAL_CONSOLE_H
#define SERIAL_CONSOLE_H

/******************************************************************************
* Includes
******************************************************************************/
#include <asf.h>
#include "string.h"
#include "circular_buffer.h"
#include <stdarg.h>

/******************************************************************************
* Variables
******************************************************************************/
extern cbuf_handle_t cbufRx;

/******************************************************************************
* Defines
******************************************************************************/

/******************************************************************************
* Structures and Enumerations
******************************************************************************/
enum eDebugLogLevels {
	LOG_INFO_LVL = 0,	//Logs an INFO message
	LOG_DEBUG_LVL = 1,	//Logs a DEBUG message
	LOG_WARNING_LVL = 2,	//Logs a WARNING MSG
	LOG_ERROR_LVL = 3,	//Logs an Error message
	LOG_FATAL_LVL = 4,	//Logs a FATAL message (a non-recoverable error)
	LOG_OFF_LVL = 5,	//Enum to indicate levels are off
	N_DEBUG_LEVELS = 6	//Max number of log levels
};

/******************************************************************************
* Global Function Declaration
******************************************************************************/
void InitializeSerialConsole(void);
void DeinitializeSerialConsole(void);
void SerialConsoleWriteString(char * string);
int SerialConsoleReadCharacter(uint8_t *rxChar);
void LogMessage(enum eDebugLogLevels level, const char *format, ...);
void setLogLevel(enum eDebugLogLevels debugLevel);
enum eDebugLogLevels getLogLevel(void);

#endif /* SENSOR_H_ */