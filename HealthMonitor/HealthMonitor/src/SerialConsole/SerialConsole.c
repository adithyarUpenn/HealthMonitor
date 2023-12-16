/**************************************************************************//**
* @file      SerialConsole.c
* @brief     Serial Console driver
* @author    Adi
* @date      2023-12-14

******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/
#include "SerialConsole.h"
#include "asf.h"
#include "./CLI/CliThread.h"
// #include "semphr.h" // Include this at the top for semaphore functions
#include <stdarg.h>

/******************************************************************************
* Defines
******************************************************************************/
#define RX_BUFFER_SIZE 512	///<Size of character buffer for RX, in bytes
#define TX_BUFFER_SIZE 512	///<Size of character buffers for TX, in bytes

/******************************************************************************
* Variables
******************************************************************************/
cbuf_handle_t cbufRx;	///<Circular buffer handler for receiving characters from the Serial Interface
cbuf_handle_t cbufTx;	///<Circular buffer handler for transmitting characters from the Serial Interface

char latestRx;	///< Holds the latest character that was received
char latestTx;	///< Holds the latest character to be transmitted.

struct usart_module usart_instance;
char rxCharacterBuffer[RX_BUFFER_SIZE]; ///<Buffer to store received characters
char txCharacterBuffer[TX_BUFFER_SIZE]; ///<Buffer to store characters to be sent
enum eDebugLogLevels currentDebugLevel = LOG_INFO_LVL; ///<Variable that holds the level of debug log messages to show. Defaults to showing all debug values

/******************************************************************************
* Forward Declarations
******************************************************************************/
static void configure_usart(void);
static void configure_usart_callbacks(void);

/******************************************************************************
* Callback Functions
******************************************************************************/
void usart_write_callback(struct usart_module *const usart_module);	//Callback for when we finish writing characters to UART
void usart_read_callback(struct usart_module *const usart_module);	//Callback for when we finis reading characters from UART

/******************************************************************************
* Functions Implementation
******************************************************************************/

/**************************************************************************//**
* @fn			void InitializeSerialConsole(void)
* @brief		Initializes the UART - sets up the SERCOM to act as UART and registers the callbacks for
*				asynchronous reads and writes.
* @details		Initializes the UART - sets up the SERCOM to act as UART and registers the callbacks for
*				asynchronous reads and writes. 
* @note			Call from main once to initialize Hardware.
*****************************************************************************/

void InitializeSerialConsole(void)
{

	//Initialize circular buffers for RX and TX
	cbufRx = circular_buf_init((uint8_t*)rxCharacterBuffer, RX_BUFFER_SIZE);
	cbufTx = circular_buf_init((uint8_t*)txCharacterBuffer, RX_BUFFER_SIZE);

	//Configure USART and Callbacks
	configure_usart();
	configure_usart_callbacks();
	NVIC_SetPriority(SERCOM4_IRQn, 10);

	usart_read_buffer_job(&usart_instance, (uint8_t*) &latestRx, 1);	//Kicks off constant reading of characters
}

/**************************************************************************//**
* @fn			void DeinitializeSerialConsole(void)
* @brief		Deinitlaises the UART
* @note			
*****************************************************************************/
void DeinitializeSerialConsole(void)
{
	usart_disable(&usart_instance);
}

/**************************************************************************//**
* @fn			void SerialConsoleWriteString(char * string)
* @brief		Writes a string to be written to the uart. Copies the string to a ring buffer that is used to hold the text send to the uart
* @details		Uses the ringbuffer 'cbufTx', which in turn uses the array 'txCharacterBuffer'
* @note			Use to send a string of characters to the user via UART
*****************************************************************************/
void SerialConsoleWriteString(char * string)
{
	if(string != NULL)
	{
		for (size_t iter = 0; iter < strlen(string); iter++)
		{
			circular_buf_put(cbufTx, string[iter]);
		}

		if(usart_get_job_status(&usart_instance, USART_TRANSCEIVER_TX) == STATUS_OK)
		{
			circular_buf_get(cbufTx, (uint8_t*) &latestTx); //Perform only if the SERCOM TX is free (not busy)
			usart_write_buffer_job(&usart_instance, (uint8_t*) &latestTx, 1);
		}
	}
}

/**************************************************************************//**
* @fn			int SerialConsoleReadCharacter(uint8_t *rxChar)
* @brief		Reads a character from the RX ring buffer and stores it on the pointer given as an argument.
*				Also, returns -1 if there is no characters on the buffer
*				This buffer has values added to it when the UART receives ASCII characters from the terminal
* @details		Uses the ringbuffer 'cbufTx', which in turn uses the array 'txCharacterBuffer'
* @param[in]	Pointer to a character. This function will return the character from the RX buffer into this pointer
* @return		Returns -1 if there are no characters in the buffer
* @note			Use to receive characters from the RX buffer (FIFO)
*****************************************************************************/
int SerialConsoleReadCharacter(uint8_t *rxChar)
{
	int a = circular_buf_get(cbufRx, (uint8_t*) rxChar);
	return a;
}


/*
DEBUG LOGGER FUNCTIONS
*/

/**************************************************************************//**
* @fn			eDebugLogLevels getLogLevel(void)
* @brief		Gets the level of debug to print to the console to the given argument.
*				Debug logs below the given level will not be allowed to be printed on the system
* @return		Returns the current debug level of the system.
* @note
*****************************************************************************/

enum eDebugLogLevels getLogLevel(void)
{
return currentDebugLevel;
}

/**************************************************************************//**
* @fn			eDebugLogLevels getLogLevel(void)
* @brief		Sets the level of debug to print to the console to the given argument.
*				Debug logs below the given level will not be allowed to be printed on the system
* @param[in]   debugLevel The debug level to be set for the debug logger
* @note
*****************************************************************************/
void setLogLevel(enum eDebugLogLevels debugLevel)
{
currentDebugLevel = debugLevel;
}


/**************************************************************************//**
* @fn		void LogMessage(enum eDebugLogLevelslevel, const char *format, ...)
* @brief	Function to print log messages
                				
* @param[in]	enum eDebugLogLevelslevel - Determines the log levels of the
				message to output. If the level is smaller than the current
				“logLevel” it is not printed. 
* @param[in]	const char *format - Pointer to a array of characters to be printed.
* @param[in]	... A variable no. of variables depending on the debug message
* @return		N/A
* @note         
*****************************************************************************/
void LogMessage(enum eDebugLogLevels level, const char *format, ...)
{
	
	if(level >= currentDebugLevel) {
		char buffer[50];
		va_list arguments;
		va_start(arguments, format);
		vsprintf(buffer, format, arguments);
		va_end ( arguments );
		SerialConsoleWriteString(buffer);
	}
	return;
};

/*
COMMAND LINE INTERFACE COMMANDS
*/

/******************************************************************************
* Local Functions
******************************************************************************/

/**************************************************************************//**
* @fn			static void configure_usart(void)
* @brief		Code to configure the SERCOM "EDBG_CDC_MODULE" to be a UART channel running at 115200 8N1
* @note			
*****************************************************************************/
static void configure_usart(void)
{
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);

	config_usart.baudrate    = 115200;
	config_usart.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
	config_usart.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
	config_usart.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
	config_usart.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
	config_usart.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;
	while (usart_init(&usart_instance,
					  EDBG_CDC_MODULE, 
					  &config_usart) != STATUS_OK) 
	{

	}
	
	usart_enable(&usart_instance);
}


/**************************************************************************//**
* @fn			static void configure_usart_callbacks(void)
* @brief		Code to register callbacks
* @note
*****************************************************************************/
static void configure_usart_callbacks(void)
{
	usart_register_callback(&usart_instance,
							usart_write_callback, 
							USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_register_callback(&usart_instance,
							usart_read_callback, 
							USART_CALLBACK_BUFFER_RECEIVED);
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
}

/******************************************************************************
* Callback Functions
******************************************************************************/

/**************************************************************************//**
* @fn			void usart_read_callback(struct usart_module *const usart_module)
* @brief		Callback called when the system finishes receives all the bytes requested from a UART read job
		Students to fill out. Please note that the code here is dummy code. It is only used to show you how some functions work.
* @note
*****************************************************************************/
void usart_read_callback(struct usart_module *const usart_module)
{
    // Add the received character to the ring buffer.
    circular_buf_put(cbufRx, (uint8_t) latestRx); 

    // Continue reading characters.
    usart_read_buffer_job(&usart_instance, (uint8_t*) &latestRx, 1);
}





/**************************************************************************//**
* @fn			void usart_write_callback(struct usart_module *const usart_module)
* @brief		Callback called when the system finishes sending all the bytes requested from a UART read job
* @note
*****************************************************************************/
void usart_write_callback(struct usart_module *const usart_module)
{
	if(circular_buf_get(cbufTx, (uint8_t*) &latestTx) != -1) //Only continue if there are more characters to send
	{
		usart_write_buffer_job(&usart_instance, (uint8_t*) &latestTx, 1);
	}
	
}

// #define F_CPU       48000000UL  // SAMD21 operates at 48 MHz
// #define UART_BAUD   115200        // Desired baud rate

// void uart_init(void) {
//     // Configure PA10 (TX) and PA11 (RX) as peripheral functions
//     PORT->Group[0].PINCFG[10].bit.PMUXEN = 1;   // Enable peripheral multiplexer for PA10
//     PORT->Group[0].PMUX[5].reg |= PORT_PMUX_PMUXE(0x1); // Set PA10 as peripheral function E (SERCOM)
//     PORT->Group[0].PINCFG[11].bit.PMUXEN = 1;   // Enable peripheral multiplexer for PA11
//     PORT->Group[0].PMUX[5].reg |= PORT_PMUX_PMUXO(0x1); // Set PA11 as peripheral function E (SERCOM)

//     // Set the GCLK generator 0 (Main clock) as source for the SERCOM module
//     GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_SERCOM2_CORE;
//     while (GCLK->STATUS.bit.SYNCBUSY);

//     // Reset SERCOM2
//     SERCOM2->USART.CTRLA.bit.ENABLE = 0;
//     while (SERCOM2->USART.SYNCBUSY.bit.ENABLE);

//     // Configure UART settings
//     SERCOM2->USART.CTRLA.reg =
//         SERCOM_USART_CTRLA_DORD |   // LSB first
//         SERCOM_USART_CTRLA_MODE_USART_INT_CLK |   // Internal clock
//         SERCOM_USART_CTRLA_RXPO(1) |   // RX on PAD[1] = PA11
//         SERCOM_USART_CTRLA_TXPO(0);    // TX on PAD[0] = PA10

//     SERCOM2->USART.CTRLB.reg =
//         SERCOM_USART_CTRLB_CHSIZE(0x0) |   // 8-bit character size
//         SERCOM_USART_CTRLB_SBMODE |   // 2 stop bits
//         SERCOM_USART_CTRLB_SFDE;    // Start of frame detection enable

//     // Calculate BAUD register value
//     uint64_t br = (uint64_t)(65536UL * (F_CPU - 8UL * UART_BAUD)) / F_CPU;
//     SERCOM2->USART.BAUD.reg = (uint16_t)br;

//     // Enable UART
//     SERCOM2->USART.CTRLA.bit.ENABLE = 1;
//     while (SERCOM2->USART.SYNCBUSY.bit.ENABLE);
// }

// void uart_send_byte(uint8_t data) {
//     // Wait for the data register to be empty
//     while (!SERCOM2->USART.INTFLAG.bit.DRE);
    
//     // Send the data
//     SERCOM2->USART.DATA.reg = data;
// }

// uint8_t uart_receive_byte(void) {
//     // Wait for data to be received
//     while (!SERCOM2->USART.INTFLAG.bit.RXC);
    
//     // Read and return the received data
//     return SERCOM2->USART.DATA.reg;
// }

// int main(void) {
//     SystemInit();

//     // Initialize UART
//     uart_init();

//     while (1) {
//         // Example: Send 'A' and receive data
//         uart_send_byte('A');

//         // Wait for a while to observe the sent character
//         for (volatile uint32_t i = 0; i < 1000000; i++) {
//             __asm__("nop");
//         }

//         // Receive the character and send it back
//         uint8_t received_data = uart_receive_byte();
//         uart_send_byte(received_data);
//     }
// }