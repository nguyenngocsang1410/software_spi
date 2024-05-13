/*******************************************************************************
* Title                 :   SPI Module Test
* Filename              :   spi_test.c
* Origin Date           :   2024/03/12
* Version               :   0.0.0
* Compiler              :   STM32CubeIDE
* Target                :   STM32F407-Disco
* Notes                 :   None
*******************************************************************************/


/** \file spi_test.c
 *  \brief This module contains the
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "main.h"
#include "spi_test.h"
#include "spi_regs_data.h"
/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/

/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/
#define ARRAY_SIZE(arr)   (sizeof(arr) / sizeof(arr[0]))
/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
extern UART_HandleTypeDef huart2;

/******************************************************************************
* Function Prototypes
*******************************************************************************/
int spi_write_slave(uint8_t slave_addr, const spi_test_reg_t *regs, uint32_t num_regs);
int spi_validate_slave(uint8_t slave_addr, const spi_test_reg_t *regs, uint32_t num_regs);

/******************************************************************************
* Function Definitions
*******************************************************************************/
/**
 * @brief: Transfer data via software SPI
 * @param read/write: 1 for write, 0 for read
 * @param first_bit: 0 for LSB first, 1 for MSB first
 * @param reg_addr: destination register address
 * @param reg_addr_size: size of the register address in bits 
 * @param p_data: pointer to the data to be sent/received
 * @param data_size: size of the data in bits
 * @return int: 0 if success, negative if failed
 */
int rf_spi_transfer(uint8_t write_read, uint8_t first_bit,
                     uint32_t reg_addr, uint8_t reg_addr_size,
                     uint32_t* p_data, uint8_t data_size)
{
    if (p_data == NULL)
    {
        printf("[ERR]: Invalid data pointer\n");
        return -1;
    }
    uint8_t transfer_cmd;
    if(write_read == SOFTSPI_WRITE)
    {
        transfer_cmd = SPI_RF_WRITE;
    }
    else
    {
        transfer_cmd = SPI_RF_READ;
    }
    int status;
    uint32_t spi_transfer_data = 0;
    // Combine write_read and reg_addr
    uint32_t addr = reg_addr;
    if(first_bit != SOFTSPI_FIRST_BIT_LSB)
    {
        addr = reverse_bit(addr, reg_addr_size);
    }
    spi_transfer_data = transfer_cmd | (addr << 1);
    status = softspi_transmit(SOFTSPI_FIRST_BIT_LSB, (uint8_t*) &spi_transfer_data, reg_addr_size + 1); // 1bit CMD
    if (status != 0)
    {
        printf("[ERR]: Failed to transmit command\n");
        return -1;
    }
    if(transfer_cmd == SPI_RF_WRITE) // write
    {
        uint32_t spi_send_data = *p_data;  
        status = softspi_transmit(first_bit, (uint8_t*) &spi_send_data, data_size);
        if (status != 0)
        {
            printf("[ERR]: Failed to transmit data\n");
            return -1;
        }
    }
    else // read
    {
        uint32_t spi_recv_data = 0;     // Data width max 32 bits

        status = softspi_receive(first_bit, (uint8_t*) &spi_recv_data, data_size);
        if (status != 0)
        {
            printf("[ERR]: Failed to receive data\n");
            return -1;
        }
        *p_data = spi_recv_data;
    }
    return 0;
}

int dac_spi_transfer(uint8_t write_read, uint8_t first_bit,
                     uint32_t reg_addr, uint8_t reg_addr_size,
                     uint32_t* p_data, uint8_t data_size)
{
    if (p_data == NULL)
    {
        printf("[ERR]: Invalid data pointer\n");
        return -1;
    }
    uint8_t transfer_cmd;
    if(write_read == SOFTSPI_WRITE)
    {
        transfer_cmd = SPI_DAC_WRITE;
    }
    else
    {
        transfer_cmd = SPI_DAC_READ;
    }
    int status;
    uint32_t spi_transfer_data = 0;
    // Combine write_read and reg_addr
    uint32_t addr = reg_addr;
    if(first_bit != SOFTSPI_FIRST_BIT_LSB)
    {
        addr = reverse_bit(addr, reg_addr_size);
    }
    spi_transfer_data = transfer_cmd | (addr << 1);
    status = softspi_transmit(SOFTSPI_FIRST_BIT_LSB, (uint8_t*) &spi_transfer_data, reg_addr_size + 1); // 1bit CMD
    if (status != 0)
    {
        printf("[ERR]: Failed to transmit command\n");
        return -1;
    }
    if(transfer_cmd == SPI_DAC_WRITE) // write
    {
        uint32_t spi_send_data = *p_data;  
        status = softspi_transmit(first_bit, (uint8_t*) &spi_send_data, data_size);
        if (status != 0)
        {
            printf("[ERR]: Failed to transmit data\n");
            return -1;
        }
    }
    else // read
    {
        uint32_t spi_recv_data = 0;     // Data width max 32 bits

        status = softspi_receive(first_bit, (uint8_t*) &spi_recv_data, data_size);
        if (status != 0)
        {
            printf("[ERR]: Failed to receive data\n");
            return -1;
        }
        *p_data = spi_recv_data;
    }
    return 0;
}

int adc_spi_transfer(uint8_t write_read, uint8_t first_bit,
                     uint32_t reg_addr, uint8_t reg_addr_size,
                     uint32_t* p_data, uint8_t data_size)
{
    if (p_data == NULL)
    {
        printf("[ERR]: Invalid data pointer\n");
        return -1;
    }
    uint8_t transfer_cmd;
    reg_addr_size += 1; // ADC slaves have 2 bit CMD
    if(write_read == SOFTSPI_WRITE)
    {
        transfer_cmd = SPI_ADC_WRITE;
    }
    else
    {
        transfer_cmd = SPI_ADC_READ;
        reg_addr_size += 2; // ADC slaves have 2 bits latency in read
    }
    first_bit = SOFTSPI_FIRST_BIT_LSB; // ADC slaves are LSB first

    int status;
    uint32_t spi_transfer_data = 0;
    // Combine write_read and reg_addr
    uint32_t addr = reg_addr;
    spi_transfer_data = transfer_cmd |(addr << 2); // CMD is 2 bit with ADC slaves
    status = softspi_transmit(SOFTSPI_FIRST_BIT_LSB, (uint8_t*) &spi_transfer_data, reg_addr_size + 1);
    if (status != 0)
    {
        printf("[ERR]: Failed to transmit command\n");
        return -1;
    }
    if(transfer_cmd == SPI_ADC_WRITE) // write
    {
        uint32_t spi_send_data = *p_data;  
        status = softspi_transmit(SOFTSPI_FIRST_BIT_LSB, (uint8_t*) &spi_send_data, data_size);
        if (status != 0)
        {
            printf("[ERR]: Failed to transmit data\n");
            return -1;
        }
    }
    else // read
    {
        uint32_t spi_recv_data = 0;     // Data width max 32 bits

        status = softspi_receive(first_bit, (uint8_t*) &spi_recv_data, data_size);
        if (status != 0)
        {
            printf("[ERR]: Failed to receive data\n");
            return -1;
        }
        *p_data = spi_recv_data;
    }
    return 0;
}

/*
 * @brief   This function writes the given register values to the slave device
 * @param   slave_addr: Slave device address (0-255)
 * @param   regs: Pointer to the array of register values to be written
 * @param   num_regs: Number of register values to be written
 * @return  0 on success, -1 on failure
 */
int spi_write_slave(uint8_t slave_addr, const spi_test_reg_t *regs, uint32_t num_regs)
{
    // Check inputs
    if(regs == NULL)
    {
        printf("[ERR]: Invalid regs\n");
        return -1;
    }

    if(num_regs == 0)
    {
        printf("[ERR]: Invalid num_regs\n");
        return -1;
    }

    int ret = 0;
    // Write the register values
    for(uint32_t i = 0; i < num_regs; i++)
    {
        // Set corresponding slave addr
        softspi_slave_select(slave_addr);
#if (SPI_TEST_DBG_MSG_EN == 1)
//        printf("Setting slave addr: %d\n", slave_addr);
        printf("Writing to reg addr: 0x%X, val: 0x%X reg addr width: %db, reg val width: %db\n",
                (unsigned int)regs[i].reg_addr, (unsigned int) regs[i].reg_val, regs[i].reg_addr_size, regs[i].reg_val_size);
#endif /* End of (SPI_TEST_DBG_MSG_EN == 1) */


        switch (slave_addr)
        {
            case SPI_RX1_SLAVE_ADDR:
            case SPI_RX2_SLAVE_ADDR:
            case SPI_ORX1_SLAVE_ADDR:
            case SPI_ORX2_SLAVE_ADDR:
            case SPI_TX1_SLAVE_ADDR:
            case SPI_TX2_SLAVE_ADDR:
            case SPI_PLL_SLAVE_ADDR:
            case SPI_RX1_RSSI_SLAVE_ADDR:
            case SPI_RX2_RSSI_SLAVE_ADDR:
            case SPI_JESD204_TX1_SLAVE_ADDR:
            case SPI_JESD204_TX2_SLAVE_ADDR:
            case SPI_JESD204_RX_SLAVE_ADDR:
            case SPI_PHY_SLAVE_ADDR:
            case SPI_QEC_RX1_SLAVE_ADDR:
            case SPI_QEC_RX2_SLAVE_ADDR:
            case SPI_QEC_ORX1_SLAVE_ADDR:
            case SPI_QEC_ORX2_SLAVE_ADDR:
            case SPI_QEC_TX1_SLAVE_ADDR:
            case SPI_QEC_TX2_SLAVE_ADDR:
            case SPI_FIR_TX1_SLAVE_ADDR:
            case SPI_FIR_TX2_SLAVE_ADDR:
            case SPI_FIR_RX1_SLAVE_ADDR:
            case SPI_FIR_RX2_SLAVE_ADDR:
            case SPI_FIR_ORX1_SLAVE_ADDR:
            case SPI_FIR_ORX2_SLAVE_ADDR:
                ret = rf_spi_transfer(SOFTSPI_WRITE, SOFTSPI_FIRST_BIT_MSB,
                                        regs[i].reg_addr, regs[i].reg_addr_size,
                                        (uint32_t*)&regs[i].reg_val, regs[i].reg_val_size);
                break;

            case SPI_DAC1_SLAVE_ADDR:
            case SPI_DAC2_SLAVE_ADDR:
                ret = dac_spi_transfer(SOFTSPI_WRITE, SOFTSPI_FIRST_BIT_MSB,
                                        regs[i].reg_addr, regs[i].reg_addr_size,
                                        (uint32_t*)&regs[i].reg_val, regs[i].reg_val_size);
                break;

            case SPI_ADC1_SLAVE_ADDR:
            case SPI_ADC2_SLAVE_ADDR:
            case SPI_ADC3_SLAVE_ADDR:
            case SPI_ADC4_SLAVE_ADDR:
                ret = adc_spi_transfer(SOFTSPI_WRITE, SOFTSPI_FIRST_BIT_MSB,
                                        regs[i].reg_addr, regs[i].reg_addr_size,
                                        (uint32_t*)&regs[i].reg_val, regs[i].reg_val_size);
                break;

            
            default:
                printf("[ERR] spi_write_slave() Not supported slave address: %d\n", slave_addr);
                break;
        }
        
        if(ret != 0)
        {
            printf("[ERR] Failed to write to reg addr: 0x%X, write val 0x%X \n", (unsigned int) regs[i].reg_addr,
            																	 (unsigned int) regs[i].reg_val);
            softspi_slave_deselect();
			return -1;
        }
        // Release the slave
        softspi_slave_deselect();
    }
    return ret;
}

/**
 * @brief Read the register values from the slave device
 * @param slave_addr: Slave device address (0-255)
 * @param regs: Pointer to the array of register values to be read
 * @param num_regs: Number of register values to be read
 * @return int: 0 on success, -1 on failure
 */
int spi_read_slave(uint8_t slave_addr, const spi_test_reg_t *regs, uint32_t num_regs)
{
    // Check inputs
    if(regs == NULL)
    {
        printf("[ERR]: Invalid regs\n");
        return -1;
    }

    if(num_regs == 0)
    {
        printf("[ERR]: Invalid num_regs\n");
        return -1;
    }

    int ret = 0;
    // Read the register values
    for(uint32_t i = 0; i < num_regs; i++)
    {
        // Set corresponding slave addr
        softspi_slave_select(slave_addr);
#if (SPI_TEST_DBG_MSG_EN == 1)
//        printf("Setting slave addr: %d\n", slave_addr);
#endif /* End of (SPI_TEST_DBG_MSG_EN == 1) */


        switch (slave_addr)
        {
            case SPI_RX1_SLAVE_ADDR:
            case SPI_RX2_SLAVE_ADDR:
            case SPI_ORX1_SLAVE_ADDR:
            case SPI_ORX2_SLAVE_ADDR:
            case SPI_TX1_SLAVE_ADDR:
            case SPI_TX2_SLAVE_ADDR:
            case SPI_PLL_SLAVE_ADDR:
            case SPI_RX1_RSSI_SLAVE_ADDR:
            case SPI_RX2_RSSI_SLAVE_ADDR:
            case SPI_JESD204_TX1_SLAVE_ADDR:
            case SPI_JESD204_TX2_SLAVE_ADDR:
            case SPI_JESD204_RX_SLAVE_ADDR:
            case SPI_PHY_SLAVE_ADDR:
            case SPI_QEC_RX1_SLAVE_ADDR:
            case SPI_QEC_RX2_SLAVE_ADDR:
            case SPI_QEC_ORX1_SLAVE_ADDR:
            case SPI_QEC_ORX2_SLAVE_ADDR:
            case SPI_QEC_TX1_SLAVE_ADDR:
            case SPI_QEC_TX2_SLAVE_ADDR:
            case SPI_FIR_TX1_SLAVE_ADDR:
            case SPI_FIR_TX2_SLAVE_ADDR:
            case SPI_FIR_RX1_SLAVE_ADDR:
            case SPI_FIR_RX2_SLAVE_ADDR:
            case SPI_FIR_ORX1_SLAVE_ADDR:
            case SPI_FIR_ORX2_SLAVE_ADDR:
                ret = rf_spi_transfer(SOFTSPI_READ, SOFTSPI_FIRST_BIT_MSB,
                                        regs[i].reg_addr, regs[i].reg_addr_size,
                                        (uint32_t*)&regs[i].reg_val, regs[i].reg_val_size);
                break;

            case SPI_DAC1_SLAVE_ADDR:
            case SPI_DAC2_SLAVE_ADDR:
                ret = dac_spi_transfer(SOFTSPI_READ, SOFTSPI_FIRST_BIT_MSB,
                                        regs[i].reg_addr, regs[i].reg_addr_size,
                                        (uint32_t*)&regs[i].reg_val, regs[i].reg_val_size);
                break;

            case SPI_ADC1_SLAVE_ADDR:
            case SPI_ADC2_SLAVE_ADDR:
            case SPI_ADC3_SLAVE_ADDR:
            case SPI_ADC4_SLAVE_ADDR:
                ret = adc_spi_transfer(SOFTSPI_READ, SOFTSPI_FIRST_BIT_MSB,
                                        regs[i].reg_addr, regs[i].reg_addr_size,
                                        (uint32_t*)&regs[i].reg_val, regs[i].reg_val_size);
                break;

            
            default:
                printf("[ERR] spi_read_slave() Not supported slave address: %d\n", slave_addr);
                break;
        }
        if (ret != 0)
        {
            printf("[ERR] Failed to read from reg addr: 0x%X, addr width: %d\n", (unsigned int)regs[i].reg_addr, regs[i].reg_addr_size);
            // Release the slave
            softspi_slave_deselect();
            return -1;
        }
        
        // Release the slave
        softspi_slave_deselect();
#if (SPI_TEST_DBG_MSG_EN == 1)
        printf("Read from reg addr: 0x%X, val: 0x%X, reg addr width: %d, reg val width: %d\n",
                   (unsigned int)regs[i].reg_addr, (unsigned int)regs[i].reg_val, regs[i].reg_addr_size, regs[i].reg_val_size);
#endif /* End of (SPI_TEST_DBG_MSG_EN == 1) */
    }
    return ret;
}

/**
 * @brief Extract the least significant bits from the data
 * @param data input
 * @param num_bits
 * @return uint32_t
 * @example: 
 *  extract_lsb_bit(0x195 , 8) -> 0x95
 *  extract_lsb_bit(0x195 , 4) -> 0x5
 */
uint32_t extract_lsb_bit(uint32_t data, uint8_t num_bits)
{
    uint32_t mask = (1 << num_bits) - 1;
    return data & mask;
}

/**
 * @brief Reading back and validating the register values from the slave device
 * @param slave_addr: Slave device address (0-255) 
 * @param regs: Pointer to the array of register values to be validated
 * @param num_regs: Number of register values to be validated
 * @return 0 on success, -1 on failure
 */
int spi_validate_slave(uint8_t slave_addr, const spi_test_reg_t *regs, uint32_t num_regs)
{
     // Check inputs
    if(regs == NULL)
    {
        printf("[ERR]: Invalid regs\n");
        return -1;
    }

    if(num_regs == 0)
    {
        printf("[ERR]: Invalid num_regs\n");
        return -1;
    }

    // Create a buffer to store the read back register values
    uint32_t read_buffer[num_regs];
    memset(read_buffer, 0, sizeof(read_buffer));

    int ret = 0;

    // Read the register values
    for (uint32_t i = 0; i < num_regs; i++)
    {
        // Set corresponding slave addr
        softspi_slave_select(slave_addr);

#if (SPI_TEST_DBG_MSG_EN == 1)
//        printf("Setting slave addr: %d\n", slave_addr);
#endif /* End of (SPI_TEST_DBG_MSG_EN == 1) */

        switch (slave_addr)
        {
            case SPI_RX1_SLAVE_ADDR:
            case SPI_RX2_SLAVE_ADDR:
            case SPI_ORX1_SLAVE_ADDR:
            case SPI_ORX2_SLAVE_ADDR:
            case SPI_TX1_SLAVE_ADDR:
            case SPI_TX2_SLAVE_ADDR:
            case SPI_PLL_SLAVE_ADDR:
            case SPI_RX1_RSSI_SLAVE_ADDR:
            case SPI_RX2_RSSI_SLAVE_ADDR:
            case SPI_JESD204_TX1_SLAVE_ADDR:
            case SPI_JESD204_TX2_SLAVE_ADDR:
            case SPI_JESD204_RX_SLAVE_ADDR:
            case SPI_PHY_SLAVE_ADDR:
            case SPI_QEC_RX1_SLAVE_ADDR:
            case SPI_QEC_RX2_SLAVE_ADDR:
            case SPI_QEC_ORX1_SLAVE_ADDR:
            case SPI_QEC_ORX2_SLAVE_ADDR:
            case SPI_QEC_TX1_SLAVE_ADDR:
            case SPI_QEC_TX2_SLAVE_ADDR:
            case SPI_FIR_TX1_SLAVE_ADDR:
            case SPI_FIR_TX2_SLAVE_ADDR:
            case SPI_FIR_RX1_SLAVE_ADDR:
            case SPI_FIR_RX2_SLAVE_ADDR:
            case SPI_FIR_ORX1_SLAVE_ADDR:
            case SPI_FIR_ORX2_SLAVE_ADDR:
                ret = rf_spi_transfer(SOFTSPI_READ, SOFTSPI_FIRST_BIT_MSB,
                                        regs[i].reg_addr, regs[i].reg_addr_size,
                                        &read_buffer[i], regs[i].reg_val_size);
                break;

            case SPI_DAC1_SLAVE_ADDR:
            case SPI_DAC2_SLAVE_ADDR:
                ret = dac_spi_transfer(SOFTSPI_READ, SOFTSPI_FIRST_BIT_MSB,
                                        regs[i].reg_addr, regs[i].reg_addr_size,
                                        &read_buffer[i], regs[i].reg_val_size);
                break;

            case SPI_ADC1_SLAVE_ADDR:
            case SPI_ADC2_SLAVE_ADDR:
            case SPI_ADC3_SLAVE_ADDR:
            case SPI_ADC4_SLAVE_ADDR:
                ret = adc_spi_transfer(SOFTSPI_READ, SOFTSPI_FIRST_BIT_MSB,
                                        regs[i].reg_addr, regs[i].reg_addr_size,
                                        &read_buffer[i], regs[i].reg_val_size);
                break;

            
            default:
                printf("[ERR] spi_validate_slave() Not supported slave address: %d\n", slave_addr);
                break;
        }

        if (ret != 0)
        {
            printf("[ERR] Failed to read from reg addr: 0x%X, addr width: %d\n", (unsigned int)regs[i].reg_addr, regs[i].reg_addr_size);
            // Release the slave
            softspi_slave_deselect();
            return -1;
        }
        // Release the slave
        softspi_slave_deselect();
#if (SPI_TEST_DBG_MSG_EN == 1)
		printf("Read from reg addr: 0x%X, val: 0x%X, reg addr width: %d, reg val width: %d\n",
                   (unsigned int)regs[i].reg_addr, (unsigned int)read_buffer[i], regs[i].reg_addr_size, regs[i].reg_val_size);
#endif /* End of (SPI_TEST_DBG_MSG_EN == 1) */
    }

    // Validate the read back register values
    for (uint32_t i = 0; i < num_regs; i++)
    {
        //note: in case of data overload, only the least significant bits are sent (either LSB first or MSB first)
        uint32_t written_value = extract_lsb_bit(regs[i].reg_val, regs[i].reg_val_size); 
        if (read_buffer[i] != written_value)
        {
            printf("[ERR] Failed to validate reg addr: 0x%X, write val: 0x%X != read val 0x%X \n", (unsigned int)regs[i].reg_addr,
            																					   (unsigned int)written_value,
																								   (unsigned int)read_buffer[i]);
            ret = -1;
        }
    }

    return ret;
}
    

int spi_validate_rssi(uint8_t slave_addr, const spi_test_reg_t *regs,
		uint32_t num_regs) {
	// Check inputs
	if (regs == NULL) {
		printf("[ERR]: Invalid regs\n");
		return -1;
	}

	if (num_regs == 0) {
		printf("[ERR]: Invalid num_regs\n");
		return -1;
	}

	// Create a buffer to store the read back register values
	uint32_t read_buffer[num_regs];
	memset(read_buffer, 0, sizeof(read_buffer));

	int ret = 0;

	// Read the register values
	for (uint32_t i = 0; i < num_regs; i++) {
		// Set corresponding slave addr
		softspi_slave_select(slave_addr);
		switch (slave_addr) {
		case SPI_RX1_SLAVE_ADDR:
		case SPI_RX2_SLAVE_ADDR:
		case SPI_ORX1_SLAVE_ADDR:
		case SPI_ORX2_SLAVE_ADDR:
		case SPI_TX1_SLAVE_ADDR:
		case SPI_TX2_SLAVE_ADDR:
		case SPI_PLL_SLAVE_ADDR:
		case SPI_RX1_RSSI_SLAVE_ADDR:
		case SPI_RX2_RSSI_SLAVE_ADDR:
		case SPI_JESD204_TX1_SLAVE_ADDR:
		case SPI_JESD204_TX2_SLAVE_ADDR:
		case SPI_JESD204_RX_SLAVE_ADDR:
		case SPI_PHY_SLAVE_ADDR:
		case SPI_QEC_RX1_SLAVE_ADDR:
		case SPI_QEC_RX2_SLAVE_ADDR:
		case SPI_QEC_ORX1_SLAVE_ADDR:
		case SPI_QEC_ORX2_SLAVE_ADDR:
		case SPI_QEC_TX1_SLAVE_ADDR:
		case SPI_QEC_TX2_SLAVE_ADDR:
		case SPI_FIR_TX1_SLAVE_ADDR:
		case SPI_FIR_TX2_SLAVE_ADDR:
		case SPI_FIR_RX1_SLAVE_ADDR:
		case SPI_FIR_RX2_SLAVE_ADDR:
		case SPI_FIR_ORX1_SLAVE_ADDR:
		case SPI_FIR_ORX2_SLAVE_ADDR:
			ret = rf_spi_transfer(SOFTSPI_READ, SOFTSPI_FIRST_BIT_MSB,
					regs[i].reg_addr, regs[i].reg_addr_size, &read_buffer[i],
					regs[i].reg_val_size);
			break;

		case SPI_DAC1_SLAVE_ADDR:
		case SPI_DAC2_SLAVE_ADDR:
			ret = dac_spi_transfer(SOFTSPI_READ, SOFTSPI_FIRST_BIT_MSB,
					regs[i].reg_addr, regs[i].reg_addr_size, &read_buffer[i],
					regs[i].reg_val_size);
			break;

		case SPI_ADC1_SLAVE_ADDR:
		case SPI_ADC2_SLAVE_ADDR:
		case SPI_ADC3_SLAVE_ADDR:
		case SPI_ADC4_SLAVE_ADDR:
			ret = adc_spi_transfer(SOFTSPI_READ, SOFTSPI_FIRST_BIT_MSB,
					regs[i].reg_addr, regs[i].reg_addr_size, &read_buffer[i],
					regs[i].reg_val_size);
			break;

		default:
			printf("[ERR] spi_validate_slave() Not supported slave address: %d\n",
					slave_addr);
			break;
		}

		if (ret != 0) {
			printf("[ERR] Failed to read from reg addr: 0x%X, addr width: %d\n",
					(unsigned int) regs[i].reg_addr, regs[i].reg_addr_size);
			// Release the slave
			softspi_slave_deselect();
			return -1;
		}
		// Release the slave
		softspi_slave_deselect();
		if (regs[i].reg_addr == 0x09)
			printf(
					"Read from reg addr: 0x%X, val: 0x%X, reg addr width: %d, reg val width: %d\n",
					(unsigned int) regs[i].reg_addr, (unsigned int) read_buffer[i],
					regs[i].reg_addr_size, regs[i].reg_val_size);
	}
	return ret;
}


int spi_master_test()
{
    int ret = 0;
    printf("\n================== Starting SPI slaves test ======================\n");
    printf("Writing to SPI slaves...\n");
    printf("==================================================================\n");
    for(uint8_t i = 0; i < ARRAY_SIZE(spi_slave_devices); i++)
    {
        printf("Writing to %s \n", spi_slave_devices[i].slave_name);
        ret = spi_write_slave(spi_slave_devices[i].slave_addr, spi_slave_devices[i].reg, spi_slave_devices[i].slave_numb_reg);
        if(ret != 0)
        {
            printf("[ERR] Failed to write to %s\n", spi_slave_devices[i].slave_name);
        }
    }
    printf("\n==================================================================\n");
    printf("Validating SPI slaves...\n");
    printf("==================================================================\n");
    for(uint8_t i = 0; i < ARRAY_SIZE(spi_slave_devices); i++)
    {
        printf("\n\n\n========================= Validating %s =========================\n\n", spi_slave_devices[i].slave_name);
        ret = spi_validate_slave(spi_slave_devices[i].slave_addr, spi_slave_devices[i].reg, spi_slave_devices[i].slave_numb_reg);
        if(ret != 0)
        {
            printf("[ERR] Failed to validate %s\n", spi_slave_devices[i].slave_name);
        }
    }
    delay_us(500);
    printf("\n==================================================================\n\n\n");
    return ret;
}


//------------------------------------------------------------------------------------
/**
 * 0: RSSI1
 * 1: RSSI2
 *
 * 4: RSSI2 chkpt1
 * 5: PLL chkpt1
 *
 * 6: PLL chkpt2
 */
int cfg_counter = 6;

/**
 * 0: Write
 * 1: Read
 */
int ctrl_token = 3;
unsigned int run_cnt = 0;
int read_delay = 10;

bool AUTO_READ_FLAG = false;
uint32_t _last_auto_read = 0;
uint32_t autoReadDelay = 100;

spi_test_reg_t rssiReg9 = { 0x09, 0x0, 7, 10 };
int32_t _elapse_time = 0;
char buffer[10] = {0};

spi_test_reg_t spi_auto_read_reg = {
		.reg_addr = 0,
		.reg_addr_size = 0,
		.reg_val = 0,
		.reg_val_size = 0};

int spi_master_cfg() {
	int ret = -1;
	/**
	 * 0: Write to regs
	 * 1: Read from regs
	 * 2: Auto read from reg
	 * 3: Write and verify reg
	 *
	 */
	switch (ctrl_token) {
		case 0:
		{
			ret = spi_write_slave(
					spi_slave_devices_for_cfg[cfg_counter].slave_addr,
					spi_slave_devices_for_cfg[cfg_counter].reg,
					spi_slave_devices_for_cfg[cfg_counter].slave_numb_reg);
			if (ret != 0)
			{
				printf("[ERR] Failed to write to %s\n",
						spi_slave_devices_for_cfg[cfg_counter].slave_name);
			}
			HAL_Delay(3000); // ms
			break;
		}
		case 1:
		{
			run_cnt++;
			printf("%u\n", run_cnt);
			ret = spi_validate_slave(
					spi_slave_devices_for_cfg[cfg_counter].slave_addr,
					spi_slave_devices_for_cfg[cfg_counter].reg,
					spi_slave_devices_for_cfg[cfg_counter].slave_numb_reg);
			if (ret != 0)
			{
				printf("[ERR] Failed to write to %s\n",
						spi_slave_devices_for_cfg[cfg_counter].slave_name);
			}
			HAL_Delay(read_delay); // ms
			break;
		}
		case 2:
		{
			// Auto read
			if (AUTO_READ_FLAG)
			{
				uint32_t currentTime = HAL_GetTick();
				if (currentTime - _last_auto_read >= autoReadDelay)
				{
					_elapse_time = currentTime - _last_auto_read;
					_last_auto_read = currentTime;

					softspi_slave_select(SPI_RX2_RSSI_SLAVE_ADDR);

					rf_spi_transfer(SOFTSPI_READ, SOFTSPI_FIRST_BIT_MSB,
							rssiReg9.reg_addr,
							rssiReg9.reg_addr_size,
							(uint32_t*) &rssiReg9.reg_val,
							rssiReg9.reg_val_size);

					sprintf(buffer, "%lu\r\n", rssiReg9.reg_val);
					HAL_UART_Transmit(&huart2,
							(uint8_t*) buffer,
							strlen(buffer),
							1000);

					softspi_slave_deselect();
				}
			}
			break;
		}
		case 3:
		{
			// Single shot
			if (ret != 0)
				ret = spi_write_slave(
						spi_slave_devices_for_cfg[cfg_counter].slave_addr,
						spi_slave_devices_for_cfg[cfg_counter].reg,
						spi_slave_devices_for_cfg[cfg_counter].slave_numb_reg);
			if (ret != 0)
			{
				printf("[ERR] Failed to write to %s\n",
						spi_slave_devices_for_cfg[cfg_counter].slave_name);
			}
			HAL_Delay(100); // 10_000 ms
			ret = spi_validate_slave(
					spi_slave_devices_for_cfg[cfg_counter].slave_addr,
					spi_slave_devices_for_cfg[cfg_counter].reg,
					spi_slave_devices_for_cfg[cfg_counter].slave_numb_reg);
			if (ret != 0)
			{
				printf("[ERR] Failed to write to %s\n",
						spi_slave_devices_for_cfg[cfg_counter].slave_name);
			}
			HAL_Delay(100); // 10_000 ms

			break;
		}
		default:
			break;
	}

	return ret;
}
//------------------------------------------------------------------------------------

//TODO: TESTING ONLY
#if (SPI_SLAVE_TEST_EN == 1)
volatile uint32_t reg_addr = 0;
volatile uint32_t write_reg_value = 0xa0;
volatile uint8_t reg_addr_size =7;
volatile uint8_t reg_val_size = 8;
volatile uint8_t slave_addr  = SPI_RX2_SLAVE_ADDR;
volatile spi_test_reg_t spi_write_reg;
volatile spi_test_reg_t spi_read_reg;


int spi_test_slave()
{
    spi_write_reg.reg_addr = reg_addr;
    spi_write_reg.reg_val = write_reg_value;
    spi_write_reg.reg_addr_size = reg_addr_size;
    spi_write_reg.reg_val_size = reg_val_size;

    spi_read_reg.reg_addr = reg_addr;
    spi_read_reg.reg_val = 0;
    spi_read_reg.reg_addr_size = reg_addr_size;
    spi_read_reg.reg_val_size = reg_val_size;

	if (0 != spi_write_slave(slave_addr, (spi_test_reg_t*) &spi_write_reg, 1))
	{
		printf("\r\n [ERR] CMD_SPI_WRITE_SLAVE_ACK failed to write slave \r\n");
	}

	if (0 != spi_read_slave(slave_addr, (spi_test_reg_t*) &spi_read_reg, 1))
	{
		printf("\r\n [ERR] CMD_SPI_WRITE_SLAVE_ACK failed to read slave \r\n");
	}

	if(spi_read_reg.reg_val != spi_write_reg.reg_val)
	{
		printf("\r\n [ERR] Mis-match data: Read/Write %d/%d reg value \r\n", (int)spi_read_reg.reg_val, (int)spi_write_reg.reg_val);
		return -1;
	}

	return 0;
}
#endif /* End of (SPI_SLAVE_TEST_EN == 1) */
