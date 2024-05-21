/****************************************************************************
* Title                 :   SPI test header file
* Filename              :   spi_test.h
* Origin Date           :   2024/03/07
* Version               :   v0.0.0
* Notes                 :   None
*****************************************************************************/


/** \file spi_test.h
 *  \brief This module contains .
 *
 *  This is the header file for 
 */
#ifndef _SPI_TEST_H_
#define _SPI_TEST_H_

/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>
#include "../soft_spi/soft_spi.h"

/******************************************************************************
* Preprocessor Constants
*******************************************************************************/
#define SPI_SLAVE_TEST_EN            (0)
#define SPI_TEST_DBG_MSG_EN          (0)
#define SPI_ENHANCE_TIMING_EN        (1) // If 1-> send read/write and slave addr in 1 transaction

#define SPI_SRC_SELECT_ARM           (0)

#if (SPI_SRC_SELECT_ARM == 0)

#define SPI_SRC_EXTERNAL()                (HAL_GPIO_WritePin(SPI_SEL_PORT, SPI_SEL_PIN, GPIO_PIN_SET))
#define SPI_SRC_INTERNAL()                (HAL_GPIO_WritePin(SPI_SEL_PORT, SPI_SEL_PIN, GPIO_PIN_RESET))

#endif // (SPI_SRC_SELECT_ARM == 0))

/*============================== Slave addresses ============================== */
#define SPI_QEC_TX1_SLAVE_ADDR      (0x01)
#define SPI_FIR_TX1_SLAVE_ADDR      (0x02)
#define SPI_QEC_RX1_SLAVE_ADDR      (0x03)
#define SPI_FIR_RX1_SLAVE_ADDR      (0x04)
#define SPI_QEC_ORX1_SLAVE_ADDR     (0x05)
#define SPI_FIR_ORX1_SLAVE_ADDR     (0x06)
#define SPI_QEC_TX2_SLAVE_ADDR      (0x07)
#define SPI_FIR_TX2_SLAVE_ADDR      (0x08)
#define SPI_QEC_RX2_SLAVE_ADDR      (0x09)
#define SPI_FIR_RX2_SLAVE_ADDR      (0x0A)
#define SPI_QEC_ORX2_SLAVE_ADDR     (0x0B)
#define SPI_FIR_ORX2_SLAVE_ADDR     (0x0C)
#define SPI_RX1_SLAVE_ADDR          (0x0D)
#define SPI_RX2_SLAVE_ADDR          (0x0E)
#define SPI_ORX1_SLAVE_ADDR         (0x0F)
#define SPI_ORX2_SLAVE_ADDR         (0x10)
#define SPI_TX1_SLAVE_ADDR          (0x11)
#define SPI_TX2_SLAVE_ADDR          (0x12)
#define SPI_PLL_SLAVE_ADDR          (0x13)
#define SPI_ADC1_SLAVE_ADDR         (0x14)
#define SPI_ADC2_SLAVE_ADDR         (0x15)
#define SPI_ADC3_SLAVE_ADDR         (0x16)
#define SPI_ADC4_SLAVE_ADDR         (0x17)
#define SPI_DAC1_SLAVE_ADDR         (0x18)
#define SPI_DAC2_SLAVE_ADDR         (0x19)
#define SPI_PHY_SLAVE_ADDR          (0x1A)
#define SPI_RX1_RSSI_SLAVE_ADDR     (0x1B)
#define SPI_RX2_RSSI_SLAVE_ADDR     (0x1C)
#define SPI_JESD204_RX_SLAVE_ADDR   (0x1D)
#define SPI_JESD204_TX1_SLAVE_ADDR  (0x1E)
#define SPI_JESD204_TX2_SLAVE_ADDR  (0x1F)
#define SPI_NA_SLAVE_ADDR           (0xFF)
      
/* ============================== RX2 register ============================== */
#define PD_BGR_ADDR                 (0x00)
#define PD_ADDR                     (0x01)
#define CTRL_VOLT_ADDR              (0x02)
#define LNA1_PD_ADDR                (0x03)
#define LNA2_PD_ADDR                (0x04)
#define RF_GAIN_ADDR                (0x05)
#define TIA_GTL_ADDR                (0x06)
#define VGA_3_ADDR                  (0x07)
#define VGA1_GTL_ADDR               (0x08)
#define FG_ADDR                     (0x09)
#define LDF_H_ADDR                  (0x0C)
#define LGF_L_ADDR                  (0x0D)
#define LDF_UPDATE_ADDR             (0x0E)
#define ADC_BGR_EN_ADDR             (0x0F)
#define ADC_BGR_TR_ADDR             (0x10)
#define DAC_BGR_EN_ADDR             (0x11)
#define DAC_BGR_TR_ADDR             (0x12)


/* ============================== ADC register ============================== */
#define ADC_REG_0_ADDR              (0)  
#define ADC_REG_1_ADDR              (1)              
#define ADC_REG_2_ADDR              (2)              
#define ADC_REG_3_ADDR              (3)              
#define ADC_REG_4_ADDR              (4)              
#define ADC_REG_5_ADDR              (5)              
#define ADC_REG_6_ADDR              (6)              
#define ADC_REG_7_ADDR              (7)              
#define ADC_REG_8_ADDR              (8)              
#define ADC_REG_9_ADDR              (9)              
#define ADC_REG_10_ADDR             (10)             
#define ADC_REG_11_ADDR             (11)             
#define ADC_REG_12_ADDR             (12)             
#define ADC_REG_13_ADDR             (13)             
#define ADC_REG_14_ADDR             (14)             
#define ADC_REG_15_ADDR             (15)             
#define ADC_REG_16_ADDR             (16)             
#define ADC_REG_17_ADDR             (17)             
#define ADC_REG_18_ADDR             (18)             
#define ADC_REG_19_ADDR             (19)
#define ADC_REG_20_ADDR             (20)             
#define ADC_REG_21_ADDR             (21)

/* ============================== PLL registers ============================== */
#define DSM_ENABLE_ADDR             (0x0C)
#define PLL_ADDR_0D_ADDR            (0x0D)
#define FRAC_DIVE_B0_ADDR           (0x0E)
#define FRAC_DIVE_B1_ADDR           (0x0F)
#define FRAC_DIVE_B2_ADDR           (0x10)
#define FRAC_DIVE_B3_ADDR           (0x11)






/******************************************************************************
* Configuration Constants
*******************************************************************************/


/******************************************************************************
* Macros
*******************************************************************************/


/******************************************************************************
* Typedefs
*******************************************************************************/
enum softspi_cmd
{
    SOFTSPI_READ = 0,
    SOFTSPI_WRITE = 1,
};

enum spi_rf_cmd
{
    SPI_RF_READ = 0,
    SPI_RF_WRITE = 1,
};

enum spi_dac_cmd
{
    SPI_DAC_READ = 1,
    SPI_DAC_WRITE = 0,
};

enum spi_adc_cmd
{
    SPI_ADC_READ = 0,
    SPI_ADC_WRITE = 1,
};

typedef struct spi_test_reg{
      uint32_t reg_addr;
      uint32_t reg_val;
      uint8_t reg_addr_size;
      uint8_t reg_val_size;
}spi_test_reg_t;

typedef struct spi_slave_test{
      uint8_t slave_addr;
      uint16_t slave_numb_reg;
      const spi_test_reg_t *reg;
      char* slave_name;

}spi_slave_test_t;


/******************************************************************************
* Variables
*******************************************************************************/



/******************************************************************************
* Function Prototypes
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
                     uint32_t* p_data, uint8_t data_size);

int dac_spi_transfer(uint8_t write_read, uint8_t first_bit,
                     uint32_t reg_addr, uint8_t reg_addr_size,
                     uint32_t* p_data, uint8_t data_size);

int adc_spi_transfer(uint8_t write_read, uint8_t first_bit,
                     uint32_t reg_addr, uint8_t reg_addr_size,
                     uint32_t* p_data, uint8_t data_size);

/*
 * @brief   This function writes the given register values to the slave device
 * @param   slave_addr: Slave device address (0-255)
 * @param   regs: Pointer to the array of register values to be written
 * @param   num_regs: Number of register values to be written
 * @return  0 on success, -1 on failure
 */
int spi_write_slave(uint8_t slave_addr, const spi_test_reg_t *regs, uint32_t num_regs);

/**
 * @brief Read the register values from the slave device
 * @param slave_addr: Slave device address (0-255)
 * @param regs: Pointer to the array of register values to be read
 * @param num_regs: Number of register values to be read
 * @return int: 0 on success, -1 on failure
 */
int spi_read_slave(uint8_t slave_addr, const spi_test_reg_t *regs, uint32_t num_regs);

int spi_master_cfg();
#ifdef __cplusplus
extern "C"{
#endif
int spi_master_test();
int spi_slaves_config_dc();
int spi_test_slave();

#ifdef __cplusplus
} // extern "C"
#endif

#endif // _SPI_TEST_H_

/*** End of File **************************************************************/
