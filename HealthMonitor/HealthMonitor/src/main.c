/**************************************************************************//**
* @file      main.c
* @brief     Main application file
* @author    Adi
* @date      2023-12-14

******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/
#include <asf.h>
#include "Sensor/Sensor.h"
#include "SerialConsole/SerialConsole.h"

/******************************************************************************
* Defines
******************************************************************************/

/******************************************************************************
* Variables
******************************************************************************/

/******************************************************************************
* Forward Declarations
******************************************************************************/
void vApplicationDaemonTaskStartupHook (void *ucParameterToPass);
void vApplicationMallocFailedHook(void);
/******************************************************************************
* Callback Functions
******************************************************************************/


/**************************************************************************//**
* @fn		int main (void)
* @brief	Main function. Program starts here
* @details 	Initialization and scheduling implemented
                				
* @param[in]	N/A
* @param[out]	N/A
* @return		Return 0, code shouldn't ideally exit this function
* @note         
*****************************************************************************/
int main (void)
{
	system_init();
	/* Insert application code here, after the board has been initialized. */
	Sensor_Initialize();
	
	InitializeSerialConsole();
	/* This skeleton code simply sets the LED to the state of the button. */
	while (1) {
		
		SerialConsoleWriteString("ESE516 - CLI and Debug Logger\r\n");
		/* Is button pressed? */
		port_pin_set_output_level(PIN_PA06, LED_0_ACTIVE);
		delay_ms(500);
		port_pin_set_output_level(PIN_PA06, !LED_0_ACTIVE);
		delay_ms(500);
	}
	
	return 0;
}

/**************************************************************************//**
* function          StartTasks
* @brief            Initialize application tasks in this function
* @details
* @param[in]        None
* @return           None
*****************************************************************************/
static void StartTasks(void)
{


// snprintf(bufferPrint, 64, "Heap before starting tasks: %d\r\n", xPortGetFreeHeapSize());
// SerialConsoleWriteString(bufferPrint);

// //Initialize Tasks here

// if (xTaskCreate(vCommandConsoleTask, "CLI_TASK", CLI_TASK_SIZE, NULL, CLI_PRIORITY, &cliTaskHandle) != pdPASS) {
// SerialConsoleWriteString("ERR: CLI task could not be initialized!\r\n");
// }

// snprintf(bufferPrint, 64, "Heap after starting CLI: %d\r\n", xPortGetFreeHeapSize());
// SerialConsoleWriteString(bufferPrint);

}


/**************************************************************************/ /**
* function          DaemonTask
* @brief            Initialization code for all subsystems that require FreeRToS
* @details			This function is called from the FreeRToS timer task. Any code
*					here will be called before other tasks are initialized.
* @param[in]        None
* @return           None
*****************************************************************************/

void vApplicationDaemonTaskStartupHook (void *ucParameterToPass) //vApplicationDaemonTaskStartupHook()
{

// //CODE HERE: Initialize any HW here


// //Initialize tasks
// StartTasks();


}



void vApplicationMallocFailedHook(void)
{
// SerialConsoleWriteString("Error on memory allocation on FREERTOS!\r\n");
// while(1);
// }

// void vApplicationStackOverflowHook(void)
// {
// SerialConsoleWriteString("Error on stack overflow on FREERTOS!\r\n");
// while(1);
}