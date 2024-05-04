/****************************************************************************
* Title                 :   software SPI header file
* Filename              :   soft_spi.h
* Author                :   ItachiVN
* Origin Date           :   2024/03/05
* Version               :   v0.0.0
* Compiler              :   STM32CubeIDE
* Target                :   STM32F407-Disco
* Notes                 :   None
*****************************************************************************/


/** \file soft_spi.h
 *  \brief This module contains .
 *
 *  This is the header file for 
 */
#ifndef _SPI_SOFT_SPI_H_
#define _SPI_SOFT_SPI_H_

/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>

/******************************************************************************
* Preprocessor Constants
*******************************************************************************/
// Pin Mapping Table //
/*
*  Function     | Pin
* ---------------------------------
*   SPI_SEL     | PE2 (0 -> ARM, 1 -> STM32) //0
*   SPI_MISO    | PE4	// 1
*   SPI_MOSI    | PE5	// 2
*   SPI_CLK     | PE6   // 3
*   SPI_CSN0    | PE7
*   SPI_CSN1    | PE8
*   SPI_CSN2    | PE9
*   SPI_CSN3    | PE10
*   SPI_CSN4    | PE11
*   SPI_CSN5    | PE12
*   SPI_CSN6    | PE13
*   SPI_CSN7    | PE14
*/
#define SPI_SEL_PORT        GPIOE
#define SPI_SEL_PIN         GPIO_PIN_2

#define SPI_MISO_PORT       GPIOE
#define SPI_MISO_PIN        GPIO_PIN_4

#define SPI_MOSI_PORT       GPIOE
#define SPI_MOSI_PIN        GPIO_PIN_5

#define SPI_CLK_PORT        GPIOE
#define SPI_CLK_PIN         GPIO_PIN_6

#define SPI_CSNX_PORT       GPIOE // Assuming all CSN pins are on the same port

#define SPI_CSN0_PORT       GPIOE
#define SPI_CSN0_PIN        GPIO_PIN_7

#define SPI_CSN1_PORT       GPIOE
#define SPI_CSN1_PIN        GPIO_PIN_8

#define SPI_CSN2_PORT       GPIOE
#define SPI_CSN2_PIN        GPIO_PIN_9

#define SPI_CSN3_PORT       GPIOE
#define SPI_CSN3_PIN        GPIO_PIN_10

#define SPI_CSN4_PORT       GPIOE
#define SPI_CSN4_PIN        GPIO_PIN_11

#define SPI_CSN5_PORT       GPIOE
#define SPI_CSN5_PIN        GPIO_PIN_12

#define SPI_CSN6_PORT       GPIOE
#define SPI_CSN6_PIN        GPIO_PIN_13

#define SPI_CSN7_PORT       GPIOE
#define SPI_CSN7_PIN        GPIO_PIN_14

/******************************************************************************
* PORTING
// TODO: PORTING
*******************************************************************************/
#ifdef STM32F407xx

#define TIM1_SRC_CLK             (168000000)    // TIM1 clock frequency
#define TIM1_PRES				 (168)		    // TIM1 default prescaler
#define TIM1_CLK                 (TIM1_SRC_CLK / TIM1_PRES)

#define soft_spi_delay()         delay_us(10)
#define soft_spi_clk_high()      HAL_GPIO_WritePin(SPI_CLK_PORT, SPI_CLK_PIN, GPIO_PIN_SET)
#define soft_spi_clk_low()       HAL_GPIO_WritePin(SPI_CLK_PORT, SPI_CLK_PIN, GPIO_PIN_RESET)
#define soft_spi_mosi_high()     HAL_GPIO_WritePin(SPI_MOSI_PORT, SPI_MOSI_PIN, GPIO_PIN_SET)
#define soft_spi_mosi_low()      HAL_GPIO_WritePin(SPI_MOSI_PORT, SPI_MOSI_PIN, GPIO_PIN_RESET)
#define soft_spi_miso_read()     HAL_GPIO_ReadPin(SPI_MISO_PORT, SPI_MISO_PIN)
#endif /* End of #ifdef STM32F407xx */


/******************************************************************************
* Configuration Constants
*******************************************************************************/



/******************************************************************************
* Macros
*******************************************************************************/

/******************************************************************************
* Typedefs
*******************************************************************************/
enum softspi_firstbit
{
    SOFTSPI_FIRST_BIT_LSB = 0,
    SOFTSPI_FIRST_BIT_MSB = 1,
};
/******************************************************************************
* Variables
*******************************************************************************/


/******************************************************************************
* Function Prototypes
*******************************************************************************/
/**
 * @brief: Initialize the software SPI IO
 * 
 */
void softspi_slave_select(uint8_t slave_id);
void softspi_slave_deselect(void);
void spi_io_init(void);
int spi_timer_init();
int softspi_transmit(uint8_t first_bit, const uint8_t *p_data, uint16_t numb_bits);
int softspi_receive(uint8_t first_bit, uint8_t *p_data, uint16_t numb_bits);


#ifdef __cplusplus
extern "C"{
#endif


#ifdef __cplusplus
} // extern "C"
#endif

#endif // _SPI_SOFT_SPI_H_

/*** End of File **************************************************************/
