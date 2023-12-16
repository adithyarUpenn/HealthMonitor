/**************************************************************************//**
* @file      HRM.c
* @brief     Heart Rate Monitor Control functions
* @author    Adi
* @date      2023-12-14

******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/
#include <asf.h>
#include "./Sensor/Sensor.h"
#include "./HRM/HRM.h"

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
* @fn		bool HRM_Initialize(void)
* @brief	Function to initialize HRM inputs
                				
* @param[in]	N/A
* @param[out]	N/A
* @return		Return true
* @note         
*****************************************************************************/
bool HRM_Initialize(void) {
	/* Initialize HRM pins */
	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);

	pin_conf.direction  = PORT_PIN_DIR_INPUT;
    pin_conf.input_pull  = 	SYSTEM_PINMUX_PIN_PULL_NONE;
	port_pin_set_config(HRM_LO_POS_CONNECT, &pin_conf);

    pin_conf.direction  = PORT_PIN_DIR_INPUT;
    pin_conf.input_pull  = 	SYSTEM_PINMUX_PIN_PULL_NONE;
	port_pin_set_config(HRM_INPUT_CONNECT, &pin_conf);

    pin_conf.direction  = PORT_PIN_DIR_INPUT;
    pin_conf.input_pull  = 	SYSTEM_PINMUX_PIN_PULL_NONE;
	port_pin_set_config(HRM_LO_NEG_CONNECT, &pin_conf);	
	return true;
}