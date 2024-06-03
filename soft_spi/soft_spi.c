/*******************************************************************************
* Title                 :   software SPI
* Filename              :   soft_spi.c
* Author                :   ItachiVN
* Origin Date           :   2024/03/05
* Version               :   0.0.0
* Compiler              :   STM32CubeIDE
* Target                :   STM32F407-Disco
* Notes                 :   None
*******************************************************************************/


/** \file soft_spi.c
 *  \brief This module contains the software SPI implementation for STM32F407-Disco
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include <stdbool.h>
#include <stdio.h>

#include "main.h"
#include "soft_spi.h"

/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
extern TIM_HandleTypeDef htim1;

/******************************************************************************
* Function Prototypes
*******************************************************************************/

/******************************************************************************
* Function Definitions
*******************************************************************************/
void spi_io_init(void)
{
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitTypeDef gpio_output_pins = {0};

    /* Configure SPI output pins */
    
    gpio_output_pins.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_output_pins.Pull = GPIO_NOPULL;
    gpio_output_pins.Speed = GPIO_SPEED_FREQ_HIGH;
    // MOSI
    gpio_output_pins.Pin =  SPI_MOSI_PIN;
    HAL_GPIO_Init(SPI_MOSI_PORT, &gpio_output_pins);
    // Set MOSI initial to low
    HAL_GPIO_WritePin(SPI_MOSI_PORT, SPI_MOSI_PIN, GPIO_PIN_RESET);

    // CLK
    gpio_output_pins.Pin =  SPI_CLK_PIN;
    HAL_GPIO_Init(SPI_CLK_PORT, &gpio_output_pins);
    // Set CLK initial to low
    HAL_GPIO_WritePin(SPI_CLK_PORT, SPI_CLK_PIN, GPIO_PIN_RESET);

    // CSNx pins (assuming they are all on the same port)
    gpio_output_pins.Pin =  SPI_CSN0_PIN | SPI_CSN1_PIN | SPI_CSN2_PIN | SPI_CSN3_PIN |
                            SPI_CSN4_PIN | SPI_CSN5_PIN | SPI_CSN6_PIN | SPI_CSN7_PIN;
    HAL_GPIO_Init(SPI_CSNX_PORT, &gpio_output_pins);
    HAL_GPIO_WritePin(SPI_CSNX_PORT, gpio_output_pins.Pin, GPIO_PIN_SET);

    // SPI source select pin
    gpio_output_pins.Pin =  SPI_SEL_PIN;
    HAL_GPIO_Init(SPI_SEL_PORT, &gpio_output_pins);
    HAL_GPIO_WritePin(SPI_SEL_PORT, gpio_output_pins.Pin, GPIO_PIN_RESET);

    /* Configure the input pins */
    GPIO_InitTypeDef gpio_input_pins = {0};
    gpio_input_pins.Mode = GPIO_MODE_INPUT;
    gpio_input_pins.Pull = GPIO_PULLUP;
    gpio_input_pins.Speed = GPIO_SPEED_FREQ_HIGH;
    // MISO
    gpio_input_pins.Pin =  SPI_MISO_PIN;
    HAL_GPIO_Init(SPI_MISO_PORT, &gpio_input_pins);
}

/**
 * @brief: TIM1 init
 * @return 0 on success, -1 on failure
 */

int spi_timer_init()
{
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = TIM1_PRES - 1;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 0xFFFF-1;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
        return -1;
    }
    return 0;
}


/**
 * @brief: Send data via software SPI
 *
 * @param p_data: pointer to the data to be sent
 * @param numb_bits: size of the data in bits
 * @return int: 0 if success, negative if failed
 * @note: Currently only support SPI mode 0, LSB first
 */
int softspi_transmit(uint8_t first_bit, const uint8_t *p_data, uint16_t numb_bits)
{
	if (p_data == NULL)
	{
		printf("[ERR]: Invalid data pointer\n");
		return -1;
	}

	if (numb_bits == 0)
			{
		printf("[ERR]: Invalid data size\n");
		return -1;
	}

    if(first_bit == SOFTSPI_FIRST_BIT_MSB)
			{
        for(uint8_t idx = numb_bits; idx > 0; idx--)
        {
            /* MODE 0: CPOL = 0, CPHA = 0
             * sample on the leading edge of the clock
             * latch data on the trailing edge of the clock
             */
            soft_spi_clk_low();
            uint8_t bit = (p_data[(idx-1)/8] & (1 << ((idx-1) % 8)));
            if(bit)
            {
                soft_spi_mosi_high();
            }
            else
            {
                soft_spi_mosi_low();
            }
            soft_spi_delay();

            soft_spi_clk_high();
            soft_spi_delay();
        }
    }
    else
    {
        for(uint8_t idx = 0; idx < numb_bits; idx++)
        {
            /* MODE 0: CPOL = 0, CPHA = 0
             * sample on the leading edge of the clock
             * latch data on the trailing edge of the clock
             */
            soft_spi_clk_low();
            uint8_t bit = (p_data[idx/8] & (1 << (idx % 8)));
            if(bit)
            {
                soft_spi_mosi_high();
            }
            else
            {
                soft_spi_mosi_low();
            }
            soft_spi_delay();

            soft_spi_clk_high();
            soft_spi_delay();
        }
    }

    soft_spi_clk_low();
	return 0;
}

/**
 * @brief: Receive number of bits via software SPI
 * @param first_bit: 0 for LSB first, 1 for MSB first
 * @param p_data: pointer to the data to be received
 * @param numb_bits: size of the data in bits
 * @return int: 0 if success, negative if failed
 * @note: Currently only support SPI mode 0, LSB first
 */
int softspi_receive(uint8_t first_bit, uint8_t *p_data, uint16_t numb_bits)
{
    if (p_data == NULL)
    {
        printf("[ERR]: Invalid data pointer\n");
        return -1;
    }

    if (numb_bits == 0)
    {
        printf("[ERR]: Invalid data size\n");
        return -1;
    }

    for (uint8_t i = 0; i < numb_bits; i++)
    {
        /* MODE 0: CPOL = 0, CPHA = 0
         * sample on the leading edge of the clock
         * latch data on the trailing edge of the clock
         */
        soft_spi_clk_low();
        soft_spi_delay();

        soft_spi_clk_high();
        // Sample MISO
        if(first_bit == SOFTSPI_FIRST_BIT_MSB)
        {
            if(soft_spi_miso_read())
            {
                p_data[(numb_bits - i - 1)/8] |= (1 << ((numb_bits - i - 1) % 8));
            }
            else
            {
                p_data[(numb_bits - i - 1)/8] &= ~(1 << ((numb_bits - i - 1) % 8));
            }
        }
        else
        {
            if(soft_spi_miso_read())
            {
                p_data[i/8] |= (1 << (i % 8));
            }
            else
            {
                p_data[i/8] &= ~(1 << (i % 8));
            }
        }
        soft_spi_delay();
    }
    soft_spi_clk_low();
    return 0;

}


uint16_t softspi_csn_pins[]={ SPI_CSN0_PIN, SPI_CSN1_PIN, SPI_CSN2_PIN, SPI_CSN3_PIN,
                             SPI_CSN4_PIN, SPI_CSN5_PIN, SPI_CSN6_PIN, SPI_CSN7_PIN};

/**
 * @brief: Control the CSNx pins

 * @param slave_id: 0-255 for the slave id

 * @note: Application specific, set the CSNx pins to high corresponding to the slave_id
          ie.   slave_id = 0x01, set CSN0 to high
                slave_id = 0x02, set CSN1 to high
                slave_id = 0x03, set CSN0 and CSN1 to high
 */
void softspi_slave_select(uint8_t slave_id)
{
    // Set the selected CSNx pin to low
    for(uint8_t i = 0; i < 8; i++)
    {
        HAL_GPIO_WritePin(SPI_CSNX_PORT, softspi_csn_pins[i], slave_id & (1 << i) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

/**
 * @brief: Release the CSNx pins by setting them all to high
 */
void softspi_slave_deselect(void)
{
    softspi_slave_select(0xFF);
}