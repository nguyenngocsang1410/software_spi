/*******************************************************************************
 * Title                 :
 * Filename              :   uart_app.c
 * Origin Date           :   2024/04/04
 * Version               :   0.0.0
 * Compiler              :
 * Target                :
 * Notes                 :   None
 *******************************************************************************/

/** \file uart_app.c
 *  \brief This module contains the
 */
/******************************************************************************
 * Includes
 *******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "uart_app.h"
#include "fifo.h"
/******************************************************************************
 * Module Preprocessor Constants
 *******************************************************************************/

/******************************************************************************
 * Module Preprocessor Macros
 *******************************************************************************/

/******************************************************************************
 * Module Typedefs
 *******************************************************************************/
typedef struct uart_data
{
	uint8_t* p_data;
	uint16_t size;
} uart_data_t;

/******************************************************************************
 * Module Variable Definitions
 *******************************************************************************/
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

uint8_t g_rxdmabuffer[UART_RX_DMABUFFER_SIZE]={0}; // Buffer for DMA
fifo_t uart_data_fifo;

/******************************************************************************
 * Function Prototypes
 *******************************************************************************/

/******************************************************************************
 * Static Function Definitions
 *******************************************************************************/

/**
 * @brief Extract data from DMA buffer
 * @param pui8buffer: Pointer to buffer to store data
 * @param pui16size: Pointer to store size of data
 * @return 0 if successful, -1 if no data
 */
static int uart_extract_data(uint8_t* pui8buffer, uint16_t* pui16size)
{
	static uint32_t old_pos=0;
	/* Calculate current position in buffer */
	uint32_t pos = UART_RX_DMABUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
	if(pos == old_pos)
	{
		*pui16size = 0;
		return -1;
	}
	/* Check if buffer is in "linear" mode or "overflow" mode */
	if (pos > old_pos)	/* Current position is over previous one */
	{
		/* We are in "linear" mode */
		/* Process data directly by subtracting "pointers" */
		*pui16size = pos - old_pos;
		memcpy(pui8buffer, &g_rxdmabuffer[old_pos], pos - old_pos);
	}
	else
	{
		/* We are in "overflow" mode */
		/* First process data to the end of buffer */
		memcpy(pui8buffer, &g_rxdmabuffer[old_pos], UART_RX_DMABUFFER_SIZE - old_pos);
		/* Check and continue with beginning of buffer */
		if (pos > 0)
		{
			memcpy((pui8buffer+(UART_RX_DMABUFFER_SIZE - old_pos)), &g_rxdmabuffer[0], pos);
		}
		*pui16size = UART_RX_DMABUFFER_SIZE - old_pos + pos;
	}
#if (UART_APP_DBG_MSG_EN != 0)
	/* Print debug message */
	printf("pos=%d, old_pos=%d, size=%d\n", (int)pos, (int)old_pos, *pui16size);
	printf("Data: %.*s\n", *pui16size, pui8buffer);
#endif /* End of (UART_APP_DBG_MSG_EN != 0)) */
	old_pos = pos;	/* Save current position as old */
	return 0;
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart == &huart2)
	{
		/* Extract data from DMA buffer */
		uint16_t data_size;
		uint8_t uart_rx_buffer[UART_RX_DMABUFFER_SIZE];
		if (uart_extract_data(uart_rx_buffer, &data_size) == 0)
		{
			/* Send data to FIFO */
			uart_fifo_send(uart_rx_buffer, data_size);
		}
		/* start the DMA again */
		uart_app_start();
	}
}

/******************************************************************************
 * Function Definitions
 *******************************************************************************/
int uart_app_init(void)
{
	fifo_init(&uart_data_fifo);
	uart_app_start();
	return 0;
}

int uart_app_send(uint8_t* p_data, uint16_t size)
{
	if (HAL_UART_Transmit(&huart2, p_data, size, 1000) != HAL_OK)
	{
		return -1;
	}
	return 0;
}

int uart_app_start(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, g_rxdmabuffer, UART_RX_DMABUFFER_SIZE);
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT); // Disable Half-Transfer interrupt
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_TC); // Disable Transfer-Complete interrupt
	return 0;
}

int uart_fifo_send(uint8_t* p_data, uint16_t size)
{
	uart_data_t* p_uart_data = (uart_data_t*)malloc(sizeof(uart_data_t));
	if(p_uart_data == NULL)
	{
		return -1;
	}
	p_uart_data->p_data = (uint8_t*)malloc(size);
	if(p_uart_data->p_data == NULL)
	{
		free(p_uart_data);
		return -1;
	}
	memcpy(p_uart_data->p_data, p_data, size);
	p_uart_data->size = size;
	fifo_put(&uart_data_fifo, p_uart_data);
	return 0;
}

int uart_fifo_receive(uint8_t* p_data, uint16_t* p_size)
{
	if(fifo_is_empty(&uart_data_fifo) == true)
	{
		return -1;
	}

	uart_data_t* p_uart_data = (uart_data_t*)fifo_get(&uart_data_fifo);
	if(p_uart_data == NULL)
	{
		return -1;
	}
	memcpy(p_data, p_uart_data->p_data, p_uart_data->size);
	*p_size = p_uart_data->size;
	free(p_uart_data->p_data);
	free(p_uart_data);
	return 0;
}
