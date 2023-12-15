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
int main (void)
{
	system_init();
	/* Insert application code here, after the board has been initialized. */
	Sensor_Initialize();
	/* This skeleton code simply sets the LED to the state of the button. */
	while (1) {
		/* Is button pressed? */
		port_pin_set_output_level(BUZZER_PIN, LED_0_ACTIVE);
		delay_ms(500);
		port_pin_set_output_level(BUZZER_PIN, !LED_0_ACTIVE);
		delay_ms(500);
	}
	
	return 0;
}