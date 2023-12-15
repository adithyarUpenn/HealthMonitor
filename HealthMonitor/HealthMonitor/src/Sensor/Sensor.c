/**************************************************************************//**
* @file      Sensor.c
* @brief     File with common sensor functions
* @author    Adi
* @date      2023-12-14

******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/
#include <asf.h>
#include "Sensor.h"
#include "./Buzzer/Buzzer.h"

/******************************************************************************
* Defines
******************************************************************************/

/******************************************************************************
* Variables
******************************************************************************/

/******************************************************************************
* Forward Declarations
******************************************************************************/

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
bool Sensor_Initialize(void) {
	/* Buzzer Initialize */
	Buzzer_Initialize();	
	return true;
}