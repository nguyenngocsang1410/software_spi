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
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))
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
int spi_write_slave(uint8_t slave_addr, const spi_test_reg_t *regs,
					uint32_t num_regs);
int spi_validate_slave(uint8_t slave_addr, const spi_test_reg_t *regs,
					   uint32_t num_regs);

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
					uint32_t *p_data, uint8_t data_size)
{
	if (p_data == NULL)
	{
		printf("[ERR]: Invalid data pointer\n");
		return -1;
	}
	uint8_t transfer_cmd;
	if (write_read == SOFTSPI_WRITE)
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
	if (first_bit != SOFTSPI_FIRST_BIT_LSB)
	{
		addr = reverse_bit(addr, reg_addr_size);
	}
	spi_transfer_data = transfer_cmd | (addr << 1);
	status =
		softspi_transmit(SOFTSPI_FIRST_BIT_LSB, (uint8_t *)&spi_transfer_data, reg_addr_size + 1); // 1bit CMD
	if (status != 0)
	{
		printf("[ERR]: Failed to transmit command\n");
		return -1;
	}
	if (transfer_cmd == SPI_RF_WRITE) // write
	{
		uint32_t spi_send_data = *p_data;
		status = softspi_transmit(first_bit, (uint8_t *)&spi_send_data, data_size);
		if (status != 0)
		{
			printf("[ERR]: Failed to transmit data\n");
			return -1;
		}
	}
	else // read
	{
		uint32_t spi_recv_data = 0; // Data width max 32 bits

		status = softspi_receive(first_bit, (uint8_t *)&spi_recv_data, data_size);
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
					 uint32_t *p_data, uint8_t data_size)
{
	if (p_data == NULL)
	{
		printf("[ERR]: Invalid data pointer\n");
		return -1;
	}
	uint8_t transfer_cmd;
	if (write_read == SOFTSPI_WRITE)
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
	if (first_bit != SOFTSPI_FIRST_BIT_LSB)
	{
		addr = reverse_bit(addr, reg_addr_size);
	}
	spi_transfer_data = transfer_cmd | (addr << 1);
	status =
		softspi_transmit(SOFTSPI_FIRST_BIT_LSB, (uint8_t *)&spi_transfer_data, reg_addr_size + 1); // 1bit CMD
	if (status != 0)
	{
		printf("[ERR]: Failed to transmit command\n");
		return -1;
	}
	if (transfer_cmd == SPI_DAC_WRITE) // write
	{
		uint32_t spi_send_data = *p_data;
		status = softspi_transmit(first_bit, (uint8_t *)&spi_send_data, data_size);
		if (status != 0)
		{
			printf("[ERR]: Failed to transmit data\n");
			return -1;
		}
	}
	else // read
	{
		uint32_t spi_recv_data = 0; // Data width max 32 bits

		status = softspi_receive(first_bit, (uint8_t *)&spi_recv_data, data_size);
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
					 uint32_t *p_data, uint8_t data_size)
{
	if (p_data == NULL)
	{
		printf("[ERR]: Invalid data pointer\n");
		return -1;
	}
	uint8_t transfer_cmd;
	reg_addr_size += 1; // ADC slaves have 2 bit CMD
	if (write_read == SOFTSPI_WRITE)
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
	spi_transfer_data = transfer_cmd | (addr << 2); // CMD is 2 bit with ADC slaves
	status =
		softspi_transmit(SOFTSPI_FIRST_BIT_LSB, (uint8_t *)&spi_transfer_data, reg_addr_size + 1);
	if (status != 0)
	{
		printf("[ERR]: Failed to transmit command\n");
		return -1;
	}
	if (transfer_cmd == SPI_ADC_WRITE) // write
	{
		uint32_t spi_send_data = *p_data;
		status =
			softspi_transmit(SOFTSPI_FIRST_BIT_LSB, (uint8_t *)&spi_send_data, data_size);
		if (status != 0)
		{
			printf("[ERR]: Failed to transmit data\n");
			return -1;
		}
	}
	else // read
	{
		uint32_t spi_recv_data = 0; // Data width max 32 bits

		status = softspi_receive(first_bit, (uint8_t *)&spi_recv_data, data_size);
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
int spi_write_slave(uint8_t slave_addr, const spi_test_reg_t *regs,
					uint32_t num_regs)
{
	// Check inputs
	if (regs == NULL)
	{
		printf("[ERR]: Invalid regs\n");
		return -1;
	}

	if (num_regs == 0)
	{
		printf("[ERR]: Invalid num_regs\n");
		return -1;
	}

	int ret = 0;
	// Write the register values
	for (uint32_t i = 0; i < num_regs; i++)
	{
		// Set corresponding slave addr
		softspi_slave_select(slave_addr);
#if (SPI_TEST_DBG_MSG_EN == 1)
		//        printf("Setting slave addr: %d\n", slave_addr);
		printf("Writing to reg addr: 0x%X, val: 0x%X reg addr width: %db, reg val width: %db\n",
			   (unsigned int)regs[i].reg_addr, (unsigned int)regs[i].reg_val, regs[i].reg_addr_size, regs[i].reg_val_size);
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
								  (uint32_t *)&regs[i].reg_val, regs[i].reg_val_size);
			break;

		case SPI_DAC1_SLAVE_ADDR:
		case SPI_DAC2_SLAVE_ADDR:
			ret = dac_spi_transfer(SOFTSPI_WRITE, SOFTSPI_FIRST_BIT_MSB,
								   regs[i].reg_addr, regs[i].reg_addr_size,
								   (uint32_t *)&regs[i].reg_val, regs[i].reg_val_size);
			break;

		case SPI_ADC1_SLAVE_ADDR:
		case SPI_ADC2_SLAVE_ADDR:
		case SPI_ADC3_SLAVE_ADDR:
		case SPI_ADC4_SLAVE_ADDR:
			ret = adc_spi_transfer(SOFTSPI_WRITE, SOFTSPI_FIRST_BIT_MSB,
								   regs[i].reg_addr, regs[i].reg_addr_size,
								   (uint32_t *)&regs[i].reg_val, regs[i].reg_val_size);
			break;

		default:
			printf("[ERR] spi_write_slave() Not supported slave address: %d\n", slave_addr);
			break;
		}

		if (ret != 0)
		{
			printf("[ERR] Failed to write to reg addr: 0x%X, write val 0x%X \n", (unsigned int)regs[i].reg_addr,
				   (unsigned int)regs[i].reg_val);
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
int spi_read_slave(uint8_t slave_addr, const spi_test_reg_t *regs,
				   uint32_t num_regs)
{
	// Check inputs
	if (regs == NULL)
	{
		printf("[ERR]: Invalid regs\n");
		return -1;
	}

	if (num_regs == 0)
	{
		printf("[ERR]: Invalid num_regs\n");
		return -1;
	}

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
								  (uint32_t *)&regs[i].reg_val, regs[i].reg_val_size);
			break;

		case SPI_DAC1_SLAVE_ADDR:
		case SPI_DAC2_SLAVE_ADDR:
			ret = dac_spi_transfer(SOFTSPI_READ, SOFTSPI_FIRST_BIT_MSB,
								   regs[i].reg_addr, regs[i].reg_addr_size,
								   (uint32_t *)&regs[i].reg_val, regs[i].reg_val_size);
			break;

		case SPI_ADC1_SLAVE_ADDR:
		case SPI_ADC2_SLAVE_ADDR:
		case SPI_ADC3_SLAVE_ADDR:
		case SPI_ADC4_SLAVE_ADDR:
			ret = adc_spi_transfer(SOFTSPI_READ, SOFTSPI_FIRST_BIT_MSB,
								   regs[i].reg_addr, regs[i].reg_addr_size,
								   (uint32_t *)&regs[i].reg_val, regs[i].reg_val_size);
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
int spi_validate_slave(uint8_t slave_addr, const spi_test_reg_t *regs,
					   uint32_t num_regs)
{
	// Check inputs
	if (regs == NULL)
	{
		printf("[ERR]: Invalid regs\n");
		return -1;
	}

	if (num_regs == 0)
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
		// note: in case of data overload, only the least significant bits are sent (either LSB first or MSB first)
		uint32_t written_value =
			extract_lsb_bit(regs[i].reg_val, regs[i].reg_val_size);
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

int spi_master_test()
{
	int ret = 0;
	printf("\n================== Starting SPI slaves test ======================\n");
	printf("Writing to SPI slaves...\n");
	printf("==================================================================\n");
	for (uint8_t i = 0; i < ARRAY_SIZE(spi_slave_devices); i++)
	{
		printf("Writing to %s \n", spi_slave_devices[i].slave_name);
		ret =
			spi_write_slave(spi_slave_devices[i].slave_addr, spi_slave_devices[i].reg, spi_slave_devices[i].slave_numb_reg);
		if (ret != 0)
		{
			printf("[ERR] Failed to write to %s\n", spi_slave_devices[i].slave_name);
		}
	}
	printf("\n==================================================================\n");
	printf("Validating SPI slaves...\n");
	printf("==================================================================\n");
	for (uint8_t i = 0; i < ARRAY_SIZE(spi_slave_devices); i++)
	{
		printf("\n\n\n========================= Validating %s =========================\n\n", spi_slave_devices[i].slave_name);
		ret =
			spi_validate_slave(spi_slave_devices[i].slave_addr, spi_slave_devices[i].reg, spi_slave_devices[i].slave_numb_reg);
		if (ret != 0)
		{
			printf("[ERR] Failed to validate %s\n", spi_slave_devices[i].slave_name);
		}
	}
	delay_us(1000);
	printf("\n==================================================================\n\n\n");
	return ret;
}

int spi_slaves_config_dc()
{
	int ret = 0;
	printf("\n================== Starting SPI slaves configs ======================\n");
	printf("Writing to SPI slaves...\n");
	printf("==================================================================\n");
	for (uint8_t i = 0; i < ARRAY_SIZE(spi_slaves_conf); i++)
	{
		printf("Writing to %s \n", spi_slaves_conf[i].slave_name);
		ret =
			spi_write_slave(spi_slaves_conf[i].slave_addr, spi_slaves_conf[i].reg, spi_slaves_conf[i].slave_numb_reg);
		if (ret != 0)
		{
			printf("[ERR] Failed to write to %s\n", spi_slaves_conf[i].slave_name);
		}
	}
	printf("\n==================================================================\n");
	printf("Validating SPI slaves...\n");
	printf("==================================================================\n");
	for (uint8_t i = 0; i < ARRAY_SIZE(spi_slaves_conf); i++)
	{
		printf("\n\n\n========================= Validating %s =========================\n\n", spi_slaves_conf[i].slave_name);
		ret =
			spi_validate_slave(spi_slaves_conf[i].slave_addr, spi_slaves_conf[i].reg, spi_slaves_conf[i].slave_numb_reg);
		if (ret != 0)
		{
			printf("[ERR] Failed to validate %s\n", spi_slaves_conf[i].slave_name);
		}
	}
	delay_us(500);
	printf("\n==================================================================\n\n\n");
	return ret;
}

/* -------------------------------------------------------------------------- */
// Sang test

// Common vars

/**
 * 0: RSSI1
 * 1: RSSI2
 * 2: PLL
 * 3:
 * 4: RSSI2 - RSSI test: performance AUXADC
 * 5: PLL - RSSI test: performance AUXADC
 * 6: PLL closed-loop check
 * 7: PLL Open loop HF
 * 8: PLL Closed loop 2.6GHz
 * 9: PLL Closed loop 3.7GHz
 * 10: PLL Open loop LF
 * 11: PLL Closed loop 2.7GHz
 * 12: Disable PLL
 */
int cfg_slave = 12;

typedef spi_test_reg_t rssi_regs_t[8];
typedef spi_test_reg_t pll_regs_t[28];

volatile rssi_regs_t *currentRssiSlave;
volatile pll_regs_t *currentPllSlave;
/**
 * -1: Do nothing
 * 0: Write regs to selected cfg_slave
 * 1: Validate regs in selected cfg_slave
 * 2: Read continously register 9 of RX2_RSSI. Print to UART
 * 3: Write then read
 * 4: Script 1
 * 5: Script 2
 * 6: Script 3
 */
int test_case = -1;

char uartBuffer[10] = {0};
spi_test_reg_t rssiReg9 = {0x09, 0x0, 7, 10};
spi_test_reg_t pllReg18 = {0x18, 0x0, 7, 8};


// Case 1 vars
int _case1_read_delay = 10;


// Case 2 vars
bool _case2_read_enable = false;
uint32_t _case2_last_read = 0;
uint32_t _case2_read_delay = 832;
int32_t _case2_elapse_time = 0;


// Case 3 vars
uint8_t _case3_run = 0;


// Case 4 vars
uint32_t _case4_last_run = 0;

spi_test_reg_t _case4_rx2_rssi_reg5 = { 0x05, 0, 7, 8 };


// Case 5 vars
uint8_t _case5_is_freq_locked = 0;
uint8_t _case5_read_delay = 1;

spi_test_reg_t _case5_targetRegs_lower[3] = {
	{ 0xE, 0x0, 7, 8 },
	{ 0xF, 0x0, 7, 8 },
	{ 0x10, 0x0, 7, 8 }
};
spi_test_reg_t _case5_targetRegs_upper[3] = {
	{ 0xE, 36, 7, 8 },
	{ 0xF, 6, 7, 8 },
	{ 0x10, 1, 7, 8 }
};


// Case 6 vars
uint8_t _case6_is_freq_locked = 0;
uint8_t _case6_run_enable = 1;
uint8_t _case6_max_range = 0xff;
uint8_t _case6_step_size = 16;

spi_test_reg_t _case6_targetRegs = { 0x10, 0, 7, 8 };


int spi_master_cfg()
{
  currentRssiSlave = (rssi_regs_t*) spi_slave_devices_for_cfg[cfg_slave].reg;
  currentPllSlave = (pll_regs_t*) spi_slave_devices_for_cfg[cfg_slave].reg;

  int ret = 0;

  switch (test_case)
  {
	// Write regs to selected cfg_slave
	case 0:
	{
	  ret = spi_write_slave(
		  spi_slave_devices_for_cfg[cfg_slave].slave_addr,
		  spi_slave_devices_for_cfg[cfg_slave].reg,
		  spi_slave_devices_for_cfg[cfg_slave].slave_numb_reg);
	  if (ret != 0)
	  {
		printf("[ERR] Failed to write to %s\n",
			spi_slave_devices_for_cfg[cfg_slave].slave_name);
	  }
	  HAL_Delay(3000); // ms
	  break;
	}

	// Validate regs in selected cfg_slave (read & verify)
	case 1:
	{
	  ret = spi_validate_slave(
		  spi_slave_devices_for_cfg[cfg_slave].slave_addr,
		  spi_slave_devices_for_cfg[cfg_slave].reg,
		  spi_slave_devices_for_cfg[cfg_slave].slave_numb_reg);
	  if (ret != 0)
	  {
		printf("[ERR] Failed to write to %s\n",
			spi_slave_devices_for_cfg[cfg_slave].slave_name);
	  }
	  HAL_Delay(_case1_read_delay); // ms
	  break;
	}

	// Read continously register 9 of RX2_RSSI. Print to UART
	case 2:
	{
	  if (_case2_read_enable)
	  {
		uint32_t currentTime = TIM2_GetTick_us();
		if (currentTime - _case2_last_read >= _case2_read_delay)
		{
		  _case2_elapse_time = currentTime - _case2_last_read;
		  _case2_last_read = currentTime;

		  softspi_slave_select(SPI_RX2_RSSI_SLAVE_ADDR);

		  rf_spi_transfer(SOFTSPI_READ, SOFTSPI_FIRST_BIT_MSB,
			  rssiReg9.reg_addr,
			  rssiReg9.reg_addr_size,
			  (uint32_t*) &rssiReg9.reg_val,
			  rssiReg9.reg_val_size);

		  sprintf(uartBuffer, "%lu\r\n", rssiReg9.reg_val);
		  HAL_UART_Transmit(&huart2,
			  (uint8_t*) uartBuffer,
			  strlen(uartBuffer),
			  1000);

		  softspi_slave_deselect();
		}
	  }
	  break;
	}

	// Write then read
	case 3:
	{
	  if (_case3_run)
	  {
		_case3_run = 0;

		ret = spi_write_slave(
			spi_slave_devices_for_cfg[cfg_slave].slave_addr,
			spi_slave_devices_for_cfg[cfg_slave].reg,
			spi_slave_devices_for_cfg[cfg_slave].slave_numb_reg);
		if (ret != 0)
		{
		  printf("[ERR] Failed to write to %s\n",
			  spi_slave_devices_for_cfg[cfg_slave].slave_name);
		}
		HAL_Delay(100); // 10_000 ms

		ret = spi_validate_slave(
			spi_slave_devices_for_cfg[cfg_slave].slave_addr,
			spi_slave_devices_for_cfg[cfg_slave].reg,
			spi_slave_devices_for_cfg[cfg_slave].slave_numb_reg);
		if (ret != 0)
		{
		  printf("[ERR] Failed to write to %s\n",
			  spi_slave_devices_for_cfg[cfg_slave].slave_name);
		}
		HAL_Delay(100); // 10_000 ms
	  }
	  break;
	}

	/**
	 * Script 1:
	 * - Write 1 to register 5 RX2_RSSI
	 * - Write 0 to register 5 RX2_RSSI
	 * - Read register 9 of RX2_RSSI
	 * - Print to UART (optional)
	 * - Restart
	 */
	case 4:
	{
	  uint32_t currentTime = TIM2_GetTick_us();

	  // Time span 1kHz ~ 1ms
	  if (currentTime - _case4_last_run >= 1000)
	  {
		// For measure runtime
		// printf("Tic: %lu\n", HAL_GetTick());

		_case4_last_run = currentTime;
		uint32_t _one_p = 1;
		uint32_t _zero_p = 0;
		/**
		 * STEP 1: Write 1 to reg 5
		 */
		softspi_slave_select(SPI_RX2_RSSI_SLAVE_ADDR);

		printf(
			"Writing to reg addr: 0x%X, val: 0x%lX reg addr width: %db, reg val width: %db\n",
			(unsigned int) _case4_rx2_rssi_reg5.reg_addr,
			_one_p,
			_case4_rx2_rssi_reg5.reg_addr_size,
			_case4_rx2_rssi_reg5.reg_val_size);

		ret = rf_spi_transfer(SOFTSPI_WRITE, SOFTSPI_FIRST_BIT_MSB,
			_case4_rx2_rssi_reg5.reg_addr,
			_case4_rx2_rssi_reg5.reg_addr_size,
			&_one_p,
			_case4_rx2_rssi_reg5.reg_val_size);

		if (ret != 0)
		{
		  printf(
			  "[ERR] Failed to write to reg addr: 0x%X, write val 0x%lX \n",
			  (unsigned int) _case4_rx2_rssi_reg5.reg_addr, _one_p);

		  softspi_slave_deselect();
		  return -1;
		}
		// Release the slave
		softspi_slave_deselect();

		/**
		 * STEP 2: Write 0 to reg 5
		 */
		softspi_slave_select(SPI_RX2_RSSI_SLAVE_ADDR);

		printf(
			"Writing to reg addr: 0x%X, val: 0x%lX reg addr width: %db, reg val width: %db\n",
			(unsigned int) _case4_rx2_rssi_reg5.reg_addr,
			_zero_p,
			_case4_rx2_rssi_reg5.reg_addr_size,
			_case4_rx2_rssi_reg5.reg_val_size);

		ret = rf_spi_transfer(SOFTSPI_WRITE, SOFTSPI_FIRST_BIT_MSB,
			_case4_rx2_rssi_reg5.reg_addr,
			_case4_rx2_rssi_reg5.reg_addr_size,
			&_zero_p,
			_case4_rx2_rssi_reg5.reg_val_size);

		if (ret != 0)
		{
		  printf(
			  "[ERR] Failed to write to reg addr: 0x%X, write val 0x%lX \n",
			  (unsigned int) _case4_rx2_rssi_reg5.reg_addr, _zero_p);

		  softspi_slave_deselect();
		  return -1;
		}
		// Release the slave
		softspi_slave_deselect();

		/**
		 * STEP 3: Read reg9
		 */

		softspi_slave_select(SPI_RX2_RSSI_SLAVE_ADDR);

		rf_spi_transfer(SOFTSPI_READ, SOFTSPI_FIRST_BIT_MSB,
			rssiReg9.reg_addr,
			rssiReg9.reg_addr_size,
			(uint32_t*) &rssiReg9.reg_val,
			rssiReg9.reg_val_size);

		sprintf(uartBuffer, "%lu\r\n", rssiReg9.reg_val);
		HAL_StatusTypeDef retUART = HAL_UART_Transmit(&huart2,
			(uint8_t*) uartBuffer,
			strlen(uartBuffer),
			1000);

		if (retUART != HAL_OK)
		{
		  printf("[ERROR] UART Fail. Code: 0x%X", (uint8_t) retUART);
		}
		softspi_slave_deselect();

		// printf("Toc: %lu\n", HAL_GetTick());
	  }
	  break;
	}

	/**
	 * Script 2: PLL frequency jump
	 * - Write upper value to 3 regs 0xE 0xF 0x10
	 * - Read 0x18, monitor bit 2 0x18
	 * - Write lower value to 3 regs 0xE 0xF 0x10
	 * - Read 0x18, monitor bit 2 0x18
	 * - Restart
	 *
	 * => Indicator by 2 pins: INDICATOR1,INDICATOR2
	 */
	case 5:
	{
	  /**
	   * STEP 1: Write upper value to 3 regs 0xE 0xF 0x10
	   */

	  spi_test_reg_t *_case5_configReg_p;

	  _case5_configReg_p = (spi_test_reg_t*) _case5_targetRegs_upper;

	  // Start to write
	  HAL_GPIO_WritePin(INDICATOR2_GPIO_Port, INDICATOR2_Pin, GPIO_PIN_SET);

	  for (int i = 0; i < 3; i++)
	  {
		softspi_slave_select(SPI_PLL_SLAVE_ADDR);
		rf_spi_transfer(SOFTSPI_WRITE, SOFTSPI_FIRST_BIT_MSB,
			_case5_configReg_p[i].reg_addr,
			_case5_configReg_p[i].reg_addr_size,
			(uint32_t*) &_case5_configReg_p[i].reg_val,
			_case5_configReg_p[i].reg_val_size);
		softspi_slave_select(SPI_PLL_SLAVE_ADDR);
		delay_us(100);
	  }

	  // Write completed
	  HAL_GPIO_WritePin(INDICATOR2_GPIO_Port, INDICATOR2_Pin, GPIO_PIN_RESET);

	  /**
	   * STEP 2: Read 0x18, monitor bit 2 0x18
	   */
	  _case5_is_freq_locked = 0;
	  while (!_case5_is_freq_locked)
	  {
		softspi_slave_select(SPI_PLL_SLAVE_ADDR);

		rf_spi_transfer(SOFTSPI_READ, SOFTSPI_FIRST_BIT_MSB,
			pllReg18.reg_addr,
			pllReg18.reg_addr_size,
			(uint32_t*) &pllReg18.reg_val,
			pllReg18.reg_val_size);

		softspi_slave_deselect();

		if ((pllReg18.reg_val & (uint32_t) 0b10) != 0)
		{
		  HAL_GPIO_WritePin(INDICATOR1_GPIO_Port, INDICATOR1_Pin, GPIO_PIN_SET);
		  HAL_Delay(1000);
		  HAL_GPIO_WritePin(INDICATOR1_GPIO_Port, INDICATOR1_Pin, GPIO_PIN_RESET);
		  _case5_is_freq_locked = 1;
		  break;
		}

		// Indicate read completed
		HAL_GPIO_WritePin(INDICATOR1_GPIO_Port, INDICATOR1_Pin, GPIO_PIN_SET);
		delay_us(1);
		HAL_GPIO_WritePin(INDICATOR1_GPIO_Port, INDICATOR1_Pin, GPIO_PIN_RESET);
	  }

	  /**
	   * STEP 3: Write lower value to 3 regs 0xE 0xF 0x10
	   */

	  _case5_configReg_p = (spi_test_reg_t*) _case5_targetRegs_lower;

	  // Start to write
	  HAL_GPIO_WritePin(INDICATOR2_GPIO_Port, INDICATOR2_Pin, GPIO_PIN_SET);

	  for (int i = 0; i < 3; i++)
	  {
		softspi_slave_select(SPI_PLL_SLAVE_ADDR);
		rf_spi_transfer(SOFTSPI_WRITE, SOFTSPI_FIRST_BIT_MSB,
			_case5_configReg_p[i].reg_addr,
			_case5_configReg_p[i].reg_addr_size,
			(uint32_t*) &_case5_configReg_p[i].reg_val,
			_case5_configReg_p[i].reg_val_size);
		softspi_slave_select(SPI_PLL_SLAVE_ADDR);
		delay_us(100);
	  }

	  // Write completed
	  HAL_GPIO_WritePin(INDICATOR2_GPIO_Port, INDICATOR2_Pin, GPIO_PIN_RESET);

	  /**
	   * STEP 4: Read 0x18, monitor bit 2 0x18
	   */
	  _case5_is_freq_locked = 0;
	  while (!_case5_is_freq_locked)
	  {
		softspi_slave_select(SPI_PLL_SLAVE_ADDR);

		rf_spi_transfer(SOFTSPI_READ, SOFTSPI_FIRST_BIT_MSB,
			pllReg18.reg_addr,
			pllReg18.reg_addr_size,
			(uint32_t*) &pllReg18.reg_val,
			pllReg18.reg_val_size);

		softspi_slave_deselect();

		if ((pllReg18.reg_val & (uint32_t) 0b10) != 0)
		{
		  HAL_GPIO_WritePin(INDICATOR1_GPIO_Port, INDICATOR1_Pin, GPIO_PIN_SET);
		  HAL_Delay(1000);
		  HAL_GPIO_WritePin(INDICATOR1_GPIO_Port, INDICATOR1_Pin, GPIO_PIN_RESET);
		  _case5_is_freq_locked = 1;
		  break;
		}

		// Indicate read completed
		HAL_GPIO_WritePin(INDICATOR1_GPIO_Port, INDICATOR1_Pin, GPIO_PIN_SET);
		delay_us(1);
		HAL_GPIO_WritePin(INDICATOR1_GPIO_Port, INDICATOR1_Pin, GPIO_PIN_RESET);
	  }

	  // Test case completed
	  HAL_GPIO_WritePin(INDICATOR2_GPIO_Port, INDICATOR2_Pin, GPIO_PIN_SET);
	  HAL_Delay(50);
	  HAL_GPIO_WritePin(INDICATOR2_GPIO_Port, INDICATOR2_Pin, GPIO_PIN_RESET);
	  HAL_Delay(50);

	  break;
	}

	/**
	 * Script 3: PLL frequency jump. Configure only 1 reg for reduce latency
	 * - Write upper value to reg 0x10
	 * - Read 0x18, monitor bit 2 0x18
	 * - Decrease value by step size
	 * - Restart
	 *
	 * => Indicator by 2 pins: INDICATOR1,INDICATOR2
	 */
	case 6:
	{
	  while (_case6_run_enable)
	  {
		/**
		 * STEP 1: Write upper value to reg 0x10
		 */

		HAL_GPIO_WritePin(INDICATOR2_GPIO_Port, INDICATOR2_Pin, GPIO_PIN_SET);

		softspi_slave_select(SPI_PLL_SLAVE_ADDR);
		rf_spi_transfer(SOFTSPI_WRITE, SOFTSPI_FIRST_BIT_MSB,
			_case6_targetRegs.reg_addr,
			_case6_targetRegs.reg_addr_size,
			(uint32_t*) &_case6_targetRegs.reg_val,
			_case6_targetRegs.reg_val_size);
		softspi_slave_deselect();

		HAL_GPIO_WritePin(INDICATOR2_GPIO_Port, INDICATOR2_Pin, GPIO_PIN_RESET);

		/**
		 * STEP 2: Read 0x18, monitor bit 2 0x18
		 */
		_case6_is_freq_locked = 0;
		while (!_case6_is_freq_locked)
		{
		  softspi_slave_select(SPI_PLL_SLAVE_ADDR);

		  rf_spi_transfer(SOFTSPI_READ, SOFTSPI_FIRST_BIT_MSB,
			  pllReg18.reg_addr,
			  pllReg18.reg_addr_size,
			  (uint32_t*) &pllReg18.reg_val,
			  pllReg18.reg_val_size);

		  softspi_slave_deselect();

		  if ((pllReg18.reg_val & (uint32_t) 0b10) != 0)
		  {
			HAL_GPIO_WritePin(INDICATOR1_GPIO_Port, INDICATOR1_Pin, GPIO_PIN_SET);
			HAL_Delay(1000);
			HAL_GPIO_WritePin(INDICATOR1_GPIO_Port, INDICATOR1_Pin, GPIO_PIN_RESET);
			_case6_is_freq_locked = 1;
			break;
		  }

		  HAL_GPIO_WritePin(INDICATOR1_GPIO_Port, INDICATOR1_Pin, GPIO_PIN_SET);
		  delay_us(1);
		  HAL_GPIO_WritePin(INDICATOR1_GPIO_Port, INDICATOR1_Pin, GPIO_PIN_RESET);
		}
		/**
		 * STEP 3: Decrease value by step size
		 */
		if (_case6_targetRegs.reg_val < _case6_step_size)
		  _case6_targetRegs.reg_val = _case6_max_range;
		else
		  _case6_targetRegs.reg_val -= _case6_step_size;
	  }
	  break;
	}

	default:
	  break;
  }
  return ret;
}

/* -------------------------------------------------------------------------- */
// Linh test
int qec_test(uint8_t slave_addr)
{
	spi_test_reg_t spi_write_reg = {
		.reg_addr = 420,
		.reg_val = 94,
		.reg_addr_size = 10,
		.reg_val_size = 8};
	if (0 != spi_write_slave(slave_addr, (spi_test_reg_t *)&spi_write_reg, 1))
	{
		printf("\r\n [ERR] spi_test_slave failed to write slave \r\n");
	}
	delay_us(1);
	spi_write_reg.reg_val = 0;
	if (0 != spi_write_slave(slave_addr, (spi_test_reg_t *)&spi_write_reg, 1))
	{
		printf("\r\n [ERR] spi_test_slave failed to write slave \r\n");
	}
	// Read reg 20-> 49
	spi_test_reg_t spi_read_reg = {
		.reg_addr_size = 10,
		.reg_val_size = 8,
	};
	for (uint16_t start_addr = 200; start_addr <= 419; start_addr++)
	{
		spi_read_reg.reg_addr = start_addr;
		if (0 != spi_read_slave(slave_addr, (spi_test_reg_t *)&spi_read_reg, 1))
		{
			printf("\r\n [ERR] spi_test_slave failed to read slave \r\n");
		}
		printf(
			"Read from slave addr: %d, reg addr: 0x%X, val: 0x%X, reg addr width: %d, reg val width: %d\n",
			slave_addr, (unsigned int)spi_read_reg.reg_addr,
			(unsigned int)spi_read_reg.reg_val,
			spi_read_reg.reg_addr_size, spi_read_reg.reg_val_size);
	}
	return 0;
}

/* -------------------------------------------------------------------------- */
// ADC test
int adc_test(uint8_t slave_addr)
{
	const spi_test_reg_t adc_init_seq_regs[] =
		{
			/* REG_ADDR,         REG_VALUE,  REG_ADDR_SIZE,  REG_VALUE_SIZE */
			{0x0, 0x100, 14, 32},
			{0x1, 0x1, 14, 32},
			{0x4, 0x66, 14, 32},
			{0x5, 0x0, 14, 32},
			{0x6, 0x2, 14, 32},
			{0x7, 0xd0d, 14, 32},
			{0x8, 0xd0d, 14, 32},
			{0x9, 0xd0d, 14, 32},
			{0xa, 0x3f, 14, 32},
			{0xa, 0xf, 14, 32},
			{0xb, 0x1115151, 14, 32},
			{0xc, 0x9, 14, 32},
			{0xd, 0x5, 14, 32},
			{0xe, 0x3, 14, 32},
			{0xf, 0xf, 14, 32},
			{0x10, 0x88, 14, 32},
			{0x11, 0x1, 14, 32},
			{0x12, 0x3, 14, 32},
			{0x13, 0x0, 14, 32},
			{0x14, 0x0, 14, 32},
		};
	if (0 != spi_write_slave(slave_addr, adc_init_seq_regs, ARRAY_SIZE(adc_init_seq_regs)))
	{
		printf("\r\n [ERR] spi_test_slave failed to write slave \r\n");
	}
	delay_us(10);
	return 0;
}

//------------------------------------------------------------------------------------
// TODO: TESTING ONLY
#if (SPI_SLAVE_TEST_EN == 1)
volatile uint32_t reg_addr = 0;
volatile uint32_t write_reg_value = 0xa0;
volatile uint8_t reg_addr_size = 7;
volatile uint8_t reg_val_size = 8;
volatile uint8_t slave_addr = SPI_RX1_RSSI_SLAVE_ADDR;
volatile spi_test_reg_t spi_write_reg;
volatile spi_test_reg_t spi_read_reg;
volatile uint8_t g_test_reg = 0;

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

	if (g_test_reg == 0)
	{
		return 0;
	}
	if (0 != spi_write_slave(slave_addr, (spi_test_reg_t *)&spi_write_reg, 1))
	{
		printf("\r\n [ERR] spi_test_slave failed to write slave \r\n");
	}

	if (0 != spi_read_slave(slave_addr, (spi_test_reg_t *)&spi_read_reg, 1))
	{
		printf("\r\n [ERR] spi_test_slave failed to read slave \r\n");
	}

	if (spi_read_reg.reg_val != spi_write_reg.reg_val)
	{
		printf("\r\n [ERR] Mis-match data: Read/Write %d/%d reg value \r\n", (int)spi_read_reg.reg_val, (int)spi_write_reg.reg_val);
		return -1;
	}

	return 0;
}
#endif /* End of (SPI_SLAVE_TEST_EN == 1) */
