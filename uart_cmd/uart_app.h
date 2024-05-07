/****************************************************************************
* Title                 :   UART app
* Filename              :   uart_app.h
* Origin Date           :   2024/04/04
* Version               :   v0.0.0
* Compiler              :   
* Target                :   
* Notes                 :   None
*****************************************************************************/


/** \file uart_app.h
 *  \brief This module contains .
 *
 *  This is the header file for 
 */
#ifndef UART_APP_H_
#define UART_APP_H_

/******************************************************************************
* Includes
*******************************************************************************/
#include "main.h"

/******************************************************************************
* Preprocessor Constants
*******************************************************************************/
#define UART_APP_DBG_MSG_EN                     (0)


/******************************************************************************
* Configuration Constants
*******************************************************************************/
#define UART_RTO_IT								(0) // Enable UART RTO interrupt
#define UART_IDLE_IT							(1) // Enable UART IDLE interrupt

#define UART_RX_DMABUFFER_SIZE					(100)


/******************************************************************************
* Macros
*******************************************************************************/

/******************************************************************************
* Typedefs
*******************************************************************************/


/******************************************************************************
* Variables
*******************************************************************************/


/******************************************************************************
* Function Prototypes
*******************************************************************************/

#ifdef __cplusplus
extern "C"{
#endif

int uart_app_init(void);
int uart_app_start(void);
int uart_app_send(uint8_t* p_data, uint16_t size);
int uart_fifo_send(uint8_t* p_data, uint16_t size);
int uart_fifo_receive(uint8_t* p_data, uint16_t* p_size);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // UART_APP_H_

/*** End of File **************************************************************/