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
#include "main.h"
#include "spi_test.h"
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


/******************************************************************************
* Slave Registers database and their values
*******************************************************************************/
const spi_test_reg_t spi_tx1_test_regs[] =
{
/* REG_ADDR,         REG_VALUE,  REG_ADDR_SIZE,  REG_VALUE_SIZE */
    {0x0,                0x18,       7,              8},
    {0x1,                0x70,       7,              8},
    {0x2,                0x19,       7,              8},
    {0x3,                0x4F,       7,              8},
    {0x4,                0x60,       7,              8},
    {0x5,                0x60,       7,              8},
    {0x6,                0xFF,       7,              8},
    {0x7,                0x2,        7,              8},
    {0x8,                0x3,        7,              8},
    {0x9,                0x0,        7,              8},
    {0xA,                0x4,        7,              8},
    {0xB,                0x1,        7,              8},
    {0xC,                0x5,        7,              8},
    {0xD,                0x1,        7,              8},
    {0xE,                0x7F,       7,              8},
    {0xF,                0x1,        7,              8},
    {0x10,               0x45,       7,              8},
};


const spi_test_reg_t spi_tx2_test_regs[] =
{
    /* REG_ADDR,         REG_VALUE,  REG_ADDR_SIZE,  REG_VALUE_SIZE */
    {0x0,                0x18,       7,              8},
    {0x1,                0x70,       7,              8},
    {0x2,                0x19,       7,              8},
    {0x3,                0x4F,       7,              8},
    {0x4,                0x60,       7,              8},
    {0x5,                0x60,       7,              8},
    {0x6,                0xFF,       7,              8},
    {0x7,                0x2,        7,              8},
    {0x8,                0x3,        7,              8},
    {0x9,                0x0,        7,              8},
    {0xA,                0x4,        7,              8},
    {0xB,                0x1,        7,              8},
    {0xC,                0x5,        7,              8},
    {0xD,                0x1,        7,              8},
    {0xE,                0x7F,       7,              8},
    {0xF,                0x1,        7,              8},
    {0x10,               0x45,       7,              8},

};

const spi_test_reg_t spi_orx1_test_regs[] =
{
    /* REG_ADDR,         REG_VALUE,  REG_ADDR_SIZE,  REG_VALUE_SIZE */
    {PD_BGR_ADDR,         0x00,        7,              8},
    {PD_ADDR,             0x00,        7,              8},
    {CTRL_VOLT_ADDR,      0x1,         7,              8},
    {LNA1_PD_ADDR,        0x1,         7,              8},
    {LNA2_PD_ADDR,        0x1,         7,              8},
    {RF_GAIN_ADDR,        0x195,       7,              8},
    {TIA_GTL_ADDR,        0x1,         7,              8},
    {VGA_3_ADDR,          0x1,         7,              8},
    {VGA1_GTL_ADDR,       0x7,         7,              8},
    {FG_ADDR,             0xF,         7,              8},
    {LDF_H_ADDR,          0x1,         7,              8},
    {LGF_L_ADDR,          0x0,         7,              8},
    {LDF_UPDATE_ADDR,     0x1,         7,              8},
    {ADC_BGR_EN_ADDR,     0x1,         7,              8},
    {ADC_BGR_TR_ADDR,     0x45,        7,              8},
//    {DAC_BGR_EN_ADDR,     0x1,         7,              8}, // FAILED, maybe read-only
//    {DAC_BGR_TR_ADDR,     0x45,        7,              8},
};

const spi_test_reg_t spi_orx2_test_regs[] =
{
    /* REG_ADDR,         REG_VALUE,  REG_ADDR_SIZE,  REG_VALUE_SIZE */
    {PD_BGR_ADDR,         0x00,        7,              8},
    {PD_ADDR,             0x00,        7,              8},
    {CTRL_VOLT_ADDR,      0x1,         7,              8},
    {LNA1_PD_ADDR,        0x1,         7,              8},
    {LNA2_PD_ADDR,        0x1,         7,              8},
    {RF_GAIN_ADDR,        0x195,       7,              8},
    {TIA_GTL_ADDR,        0x1,         7,              8},
    {VGA_3_ADDR,          0x1,         7,              8},
    {VGA1_GTL_ADDR,       0x7,         7,              8},
    {FG_ADDR,             0xF,         7,              8},
    {LDF_H_ADDR,          0x1,         7,              8},
    {LGF_L_ADDR,          0x0,         7,              8},
    {LDF_UPDATE_ADDR,     0x1,         7,              8},
    {ADC_BGR_EN_ADDR,     0x1,         7,              8},
    {ADC_BGR_TR_ADDR,     0x45,        7,              8},
//    {DAC_BGR_EN_ADDR,     0x1,         7,              8}, // FAILED, maybe read-only
//    {DAC_BGR_TR_ADDR,     0x45,        7,              8},
};

const spi_test_reg_t spi_rx1_test_regs[] =
{
    /* REG_ADDR,         REG_VALUE,  REG_ADDR_SIZE,  REG_VALUE_SIZE */
    {PD_BGR_ADDR,         0x00,        7,              8},
    {PD_ADDR,             0x00,        7,              8},
    {CTRL_VOLT_ADDR,      0x1,         7,              8},
    {LNA1_PD_ADDR,        0x1,         7,              8},
    {LNA2_PD_ADDR,        0x1,         7,              8},
    {RF_GAIN_ADDR,        0x195,       7,              8},
    {TIA_GTL_ADDR,        0x1,         7,              8},
    {VGA_3_ADDR,          0x1,         7,              8},
    {VGA1_GTL_ADDR,       0x7,         7,              8},
    {FG_ADDR,             0xF,         7,              8},
    {LDF_H_ADDR,          0x1,         7,              8},
    {LGF_L_ADDR,          0x0,         7,              8},
    {LDF_UPDATE_ADDR,     0x1,         7,              8},
    {ADC_BGR_EN_ADDR,     0x1,         7,              8},
    {ADC_BGR_TR_ADDR,     0x45,        7,              8},
//    {DAC_BGR_EN_ADDR,     0x1,         7,              8}, // FAILED, maybe read-only
//    {DAC_BGR_TR_ADDR,     0x45,        7,              8},
};

const spi_test_reg_t spi_rx2_test_regs[] = 
{
    /* REG_ADDR,         REG_VALUE,  REG_ADDR_SIZE,  REG_VALUE_SIZE */
    {PD_BGR_ADDR,         0x00,        7,              8},
    {PD_ADDR,             0x00,        7,              8},
    {CTRL_VOLT_ADDR,      0x1,         7,              8},
    {LNA1_PD_ADDR,        0x1,         7,              8},
    {LNA2_PD_ADDR,        0x1,         7,              8},
    {RF_GAIN_ADDR,        0x195,       7,              8},
    {TIA_GTL_ADDR,        0x1,         7,              8},
    {VGA_3_ADDR,          0x1,         7,              8},
    {VGA1_GTL_ADDR,       0x7,         7,              8},
    {FG_ADDR,             0xF,         7,              8},
    {LDF_H_ADDR,          0x1,         7,              8},
    {LGF_L_ADDR,          0x0,         7,              8},
    {LDF_UPDATE_ADDR,     0x1,         7,              8},
    {ADC_BGR_EN_ADDR,     0x1,         7,              8},
    {ADC_BGR_TR_ADDR,     0x45,        7,              8},
//    {DAC_BGR_EN_ADDR,     0x1,         7,              8}, // FAILED, maybe read-only
//    {DAC_BGR_TR_ADDR,     0x45,        7,              8},
};

const spi_test_reg_t spi_adc1_test_regs[] = 
{
    /* REG_ADDR,         REG_VALUE,  REG_ADDR_SIZE,  REG_VALUE_SIZE */
    {ADC_REG_0_ADDR,      0x1000,     14,              32},
    {ADC_REG_1_ADDR,      0x01,       14,              32}, 
    {ADC_REG_2_ADDR,      0x02,       14,              32}, 
    {ADC_REG_3_ADDR,      0x03,       14,              32},
    {ADC_REG_4_ADDR,      0x66,       14,              32},
    {ADC_REG_5_ADDR,      0x02,       14,              32},
    {ADC_REG_6_ADDR,      0x0D0D,     14,              32},
    {ADC_REG_7_ADDR,      0x0D0D,     14,              32}, 
    {ADC_REG_8_ADDR,      0x0D0D,     14,              32}, 
    {ADC_REG_9_ADDR,      0x3F,       14,              32},
    {ADC_REG_10_ADDR,     0x1115151,  14,              32}, // Might need larger data type
    {ADC_REG_11_ADDR,     0x09,       14,              32},
    {ADC_REG_12_ADDR,     0x05,       14,              32},
    {ADC_REG_13_ADDR,     0x03,       14,              32},
    {ADC_REG_14_ADDR,     0xF,        14,              32},
    {ADC_REG_15_ADDR,     0xCC,       14,              32},
    {ADC_REG_16_ADDR,     0x1,        14,              32},
    {ADC_REG_17_ADDR,     0x3,        14,              32},
    {ADC_REG_18_ADDR,     0x00,       14,              32},
    {ADC_REG_19_ADDR,     0x00,       14,              32},
    {ADC_REG_20_ADDR,     0x00,       14,              32},
    // {ADC_REG_21_ADDR,     0x00,       14,              32} // FAILED, maybe read-only
};

const spi_test_reg_t spi_adc2_test_regs[] = 
{
    /* REG_ADDR,         REG_VALUE,  REG_ADDR_SIZE,  REG_VALUE_SIZE */
    {ADC_REG_0_ADDR,      0x1000,     14,              32},
    {ADC_REG_1_ADDR,      0x01,       14,              32}, 
    {ADC_REG_2_ADDR,      0x02,       14,              32}, 
    {ADC_REG_3_ADDR,      0x03,       14,              32},
    {ADC_REG_4_ADDR,      0x66,       14,              32},
    {ADC_REG_5_ADDR,      0x02,       14,              32},
    {ADC_REG_6_ADDR,      0x0D0D,     14,              32},
    {ADC_REG_7_ADDR,      0x0D0D,     14,              32}, 
    {ADC_REG_8_ADDR,      0x0D0D,     14,              32}, 
    {ADC_REG_9_ADDR,      0x3F,       14,              32},
    {ADC_REG_10_ADDR,     0x1115151,  14,              32}, // Might need larger data type
    {ADC_REG_11_ADDR,     0x09,       14,              32},
    {ADC_REG_12_ADDR,     0x05,       14,              32},
    {ADC_REG_13_ADDR,     0x03,       14,              32},
    {ADC_REG_14_ADDR,     0xF,        14,              32},
    {ADC_REG_15_ADDR,     0xCC,       14,              32},
    {ADC_REG_16_ADDR,     0x1,        14,              32},
    {ADC_REG_17_ADDR,     0x3,        14,              32},
    {ADC_REG_18_ADDR,     0x00,       14,              32},
    {ADC_REG_19_ADDR,     0x00,       14,              32},
    {ADC_REG_20_ADDR,     0x00,       14,              32},
//    {ADC_REG_21_ADDR,     0x00,       14,              32}  // FAILED, maybe read-only
};

const spi_test_reg_t spi_adc3_test_regs[] = 
{
    /* REG_ADDR,         REG_VALUE,  REG_ADDR_SIZE,  REG_VALUE_SIZE */
    {ADC_REG_0_ADDR,      0x1000,     14,              32},
    {ADC_REG_1_ADDR,      0x01,       14,              32}, 
    {ADC_REG_2_ADDR,      0x02,       14,              32}, 
    {ADC_REG_3_ADDR,      0x03,       14,              32},
    {ADC_REG_4_ADDR,      0x66,       14,              32},
    {ADC_REG_5_ADDR,      0x02,       14,              32},
    {ADC_REG_6_ADDR,      0x0D0D,     14,              32},
    {ADC_REG_7_ADDR,      0x0D0D,     14,              32}, 
    {ADC_REG_8_ADDR,      0x0D0D,     14,              32}, 
    {ADC_REG_9_ADDR,      0x3F,       14,              32},
    {ADC_REG_10_ADDR,     0x1115151,  14,              32}, // Might need larger data type
    {ADC_REG_11_ADDR,     0x09,       14,              32},
    {ADC_REG_12_ADDR,     0x05,       14,              32},
    {ADC_REG_13_ADDR,     0x03,       14,              32},
    {ADC_REG_14_ADDR,     0xF,        14,              32},
    {ADC_REG_15_ADDR,     0xCC,       14,              32},
    {ADC_REG_16_ADDR,     0x1,        14,              32},
    {ADC_REG_17_ADDR,     0x3,        14,              32},
    {ADC_REG_18_ADDR,     0x00,       14,              32},
    {ADC_REG_19_ADDR,     0x00,       14,              32},
    {ADC_REG_20_ADDR,     0x00,       14,              32},
//    {ADC_REG_21_ADDR,     0x00,       14,              32} // FAILED, maybe read-only
};

const spi_test_reg_t spi_adc4_test_regs[] = 
{
    /* REG_ADDR,         REG_VALUE,  REG_ADDR_SIZE,  REG_VALUE_SIZE */
    {ADC_REG_0_ADDR,      0x1000,     14,              32},
    {ADC_REG_1_ADDR,      0x01,       14,              32}, 
    {ADC_REG_2_ADDR,      0x02,       14,              32}, 
    {ADC_REG_3_ADDR,      0x03,       14,              32},
    {ADC_REG_4_ADDR,      0x66,       14,              32},
    {ADC_REG_5_ADDR,      0x02,       14,              32},
    {ADC_REG_6_ADDR,      0x0D0D,     14,              32},
    {ADC_REG_7_ADDR,      0x0D0D,     14,              32}, 
    {ADC_REG_8_ADDR,      0x0D0D,     14,              32}, 
    {ADC_REG_9_ADDR,      0x3F,       14,              32},
    {ADC_REG_10_ADDR,     0x1115151,  14,              32}, // Might need larger data type
    {ADC_REG_11_ADDR,     0x09,       14,              32},
    {ADC_REG_12_ADDR,     0x05,       14,              32},
    {ADC_REG_13_ADDR,     0x03,       14,              32},
    {ADC_REG_14_ADDR,     0xF,        14,              32},
    {ADC_REG_15_ADDR,     0xCC,       14,              32},
    {ADC_REG_16_ADDR,     0x1,        14,              32},
    {ADC_REG_17_ADDR,     0x3,        14,              32},
    {ADC_REG_18_ADDR,     0x00,       14,              32},
    {ADC_REG_19_ADDR,     0x00,       14,              32},
    {ADC_REG_20_ADDR,     0x00,       14,              32},
//    {ADC_REG_21_ADDR,     0x00,       14,              32} // FAILED, maybe read-only
};

const spi_test_reg_t spi_pll_test_regs[] = 
{
    /* REG_ADDR,         REG_VALUE,  REG_ADDR_SIZE,  REG_VALUE_SIZE */
    {0x00,                 0x01,       7,              8},
    {0x01,                 0x02,       7,              8},
    {0x02,                 0x03,       7,              8},
    {0x03,                 0x04,       7,              8},
    {0x04,                 0x05,       7,              8},
    {0x05,                 0x06,       7,              8},
    {0x06,                 0x07,       7,              8},
    {0x07,                 0x08,       7,              8},
    {0x08,                 0x09,       7,              8},
    {0x09,                 0x0A,       7,              8},
    {0x0A,                 0x0B,       7,              8},
    {0x0B,                 0x0C,       7,              8},
    {DSM_ENABLE_ADDR,      0x02,       7,              8},
    {PLL_ADDR_0D_ADDR,     0x07,       7,              8}, 
    {FRAC_DIVE_B0_ADDR,    0x00,       7,              8},
    {FRAC_DIVE_B1_ADDR,    0x00,       7,              8},
    {FRAC_DIVE_B2_ADDR,    0x80,       7,              8}, 
    {FRAC_DIVE_B3_ADDR,    0x03,       7,              8},
    {0x12,                 0x11,       7,              8},
    {0x13,                 0x12,       7,              8},
    {0x14,                 0x11,       7,              8},
    {0x15,                 0x12,       7,              8},
    {0x16,                 0x11,       7,              8},
    {0x17,                 0x12,       7,              8},
};
//------------------------------------------------------------------------------------
spi_test_reg_t spi_pll_cfg_regs[] =
{
    /* REG_ADDR,         REG_VALUE,  REG_ADDR_SIZE,  REG_VALUE_SIZE */
    {0x00,                 0x0,       7,              8},
    {0x01,                 0x0,       7,              8},
    {0x02,                 0x0,       7,              8},
    {0x03,                 0x0,       7,              8},
    {0x04,                 0x0,       7,              8},
    {0x05,                 0x0,       7,              8},
    {0x06,                 0x0,       7,              8},
    {0x07,                 0x0,       7,              8},
    {0x08,                 0x0,       7,              8},
    {0x09,                 0x0,       7,              8},
    {0x0A,                 0x0,       7,              8},
    {0x0B,                 0x0,       7,              8},
    {0x0C,      		   0x0,       7,              8},
    {0x0D,     			   0x0,       7,              8},
    {0x0E,    			   0x0,       7,              8},
    {0x0F,    			   0x0,       7,              8},
    {0x10,    			   0x0,       7,              8},
    {0x11,    			   0x0,       7,              8},
    {0x12,                 0x0,       7,              8},
    {0x13,                 0x0,       7,              8},
    {0x14,                 0x0,       7,              8},
    {0x15,                 0x0,       7,              8},
    {0x16,                 0x0,       7,              8},
    {0x17,                 0x0,       7,              8},
};

spi_test_reg_t spi_pll_cfg_regs_read[] =
{
    /* REG_ADDR,         REG_VALUE,  REG_ADDR_SIZE,  REG_VALUE_SIZE */
    {0x00,                 0x0,       7,              8},
    {0x01,                 0x0,       7,              8},
    {0x02,                 0x0,       7,              8},
    {0x03,                 0x0,       7,              8},
    {0x04,                 0x0,       7,              8},
    {0x05,                 0x0,       7,              8},
    {0x06,                 0x0,       7,              8},
    {0x07,                 0x0,       7,              8},
    {0x08,                 0x0,       7,              8},
    {0x09,                 0x0,       7,              8},
    {0x0A,                 0x0,       7,              8},
    {0x0B,                 0x0,       7,              8},
    {DSM_ENABLE_ADDR,      0x0,       7,              8},
    {PLL_ADDR_0D_ADDR,     0x0,       7,              8},
    {FRAC_DIVE_B0_ADDR,    0x0,       7,              8},
    {FRAC_DIVE_B1_ADDR,    0x0,       7,              8},
    {FRAC_DIVE_B2_ADDR,    0x0,       7,              8},
    {FRAC_DIVE_B3_ADDR,    0x0,       7,              8},
    {0x12,                 0x0,       7,              8},
    {0x13,                 0x0,       7,              8},
    {0x14,                 0x0,       7,              8},
    {0x15,                 0x0,       7,              8},
    {0x16,                 0x0,       7,              8},
    {0x17,                 0x0,       7,              8},
	{0x18,                 0x0,       7,              8},
    {0x19,                 0x0,       7,              8},
    {0x20,                 0x0,       7,              8},
	{0x21,                 0x0,       7,              8},
};

spi_test_reg_t spi_pll_cfg_regs_state_1[] =
{
    /* REG_ADDR,         REG_VALUE,  REG_ADDR_SIZE,  REG_VALUE_SIZE */
    {0x00,                 0b111,       7,              8},
    {0x01,                 0b1000,       7,              8},
    {0x02,                 0b111,       7,              8},
    {0x03,                 0b11,       7,              8},
    {0x04,                 0b0,       7,              8},
    {0x05,                 0b0,       7,              8},
    {0x06,                 0b0,       7,              8},
    {0x07,                 0b0,       7,              8},
    {0x08,                 0b1100,       7,              8},
    {0x09,                 0b11111,       7,              8},
    {0x0A,                 0b0,       7,              8},
    {0x0B,                 0b11,       7,              8},
    {DSM_ENABLE_ADDR,      0b111,       7,              8},
    {PLL_ADDR_0D_ADDR,     0b0,       7,              8},
    {FRAC_DIVE_B0_ADDR,    0b0,       7,              8},
    {FRAC_DIVE_B1_ADDR,    0b0,       7,              8},
    {FRAC_DIVE_B2_ADDR,    0b0,       7,              8},
    {FRAC_DIVE_B3_ADDR,    0b0,       7,              8},
    {0x12,                 0b11,       7,              8},
    {0x13,                 0b0,       7,              8},
    {0x14,                 0b1,       7,              8},
    {0x15,                 0b1,       7,              8},
    {0x16,                 0b0,       7,              8},
    {0x17,                 0b0,       7,              8},
	{0x18,                 0b0,       7,              8},
    {0x19,                 0b0,       7,              8},
    {0x20,                 0b0,       7,              8},
	{0x21,                 0b0,       7,              8},
};
//------------------------------------------------------------------------------------

const spi_test_reg_t spi_rx1_rssi_test_regs[] = 
{
    /* REG_ADDR,         REG_VALUE,  REG_ADDR_SIZE,  REG_VALUE_SIZE */
    {0x00,                 0x01,       7,              10},
    {0x01,                 0x02,       7,              10},
    {0x02,                 0x03,       7,              10},
    {0x03,                 0x04,       7,              10},
    {0x04,                 0x05,       7,              10},
    {0x05,                 0x06,       7,              10},
    {0x06,                 0x07,       7,              10},
    {0x07,                 0x08,       7,              10},
//    {0x08,                 0x09,       7,              10}, // FAILED, maybe read-only
    {0x09,                 0x0A,       7,              10},
};
//------------------------------------------------------------------------------------
spi_test_reg_t spi_rx_rssi_cfg_regs[] =
{
    /* REG_ADDR,         REG_VALUE,  REG_ADDR_SIZE,  REG_VALUE_SIZE */
    {0x00,                 0x0,       7,              10},
    {0x01,                 0x0,       7,              10},
    {0x02,                 0x0,       7,              10},
    {0x03,                 0x0,       7,              10},
    {0x04,                 0x0,       7,              10},
    {0x05,                 0x0,       7,              10},
    {0x06,                 0x0,       7,              10},
    {0x07,                 0x0,       7,              10},
    {0x09,                 0x0,       7,              10},
};

//------------------------------------------------------------------------------------
const spi_test_reg_t spi_rx2_rssi_test_regs[] = 
{
    /* REG_ADDR,         REG_VALUE,  REG_ADDR_SIZE,  REG_VALUE_SIZE */
    {0x00,                 0x01,       7,              10},
    {0x01,                 0x02,       7,              10},
    {0x02,                 0x03,       7,              10},
    {0x03,                 0x04,       7,              10},
    {0x04,                 0x05,       7,              10},
    {0x05,                 0x06,       7,              10},
    {0x06,                 0x07,       7,              10},
    {0x07,                 0x08,       7,              10},
//    {0x08,                 0x09,       7,              10}, // FAILED, maybe read-only
//    {0x09,                 0x0A,       7,              10},
};

const spi_test_reg_t spi_dac1_test_regs[] = 
{
    /* REG_ADDR,         REG_VALUE,  REG_ADDR_SIZE,  REG_VALUE_SIZE */
    {0x00,                 0x01,       7,              16},
    {0x01,                 0x02,       7,              16},
    {0x02,                 0x03,       7,              16},
    {0x03,                 0x04,       7,              16},
    {0x04,                 0x05,       7,              16},
    {0x05,                 0x06,       7,              16},
    {0x06,                 0x07,       7,              16},
    {0x07,                 0x08,       7,              16},
    {0x08,                 0x09,       7,              16},
    // {0x09,                 0x0A,       7,              16},  // Not exist
    // {0x0A,                 0x0B,       7,              16}, // Not exist
    // {0x0B,                 0x0C,       7,              16}, // Not exist
    // {0x0C,                 0x02,       7,              16}, // Not exist
    // {0x0D,                 0x07,       7,              16}, // Not exist 
    // {0x0E,                 0x00,       7,              16}, // Not exist
    // {0x0F,                 0x00,       7,              16}, // Not exist
    {0x10,                 0x80,       7,              16}, 
    {0x11,                 0x03,       7,              16},
    {0x12,                 0x11,       7,              16},
    {0x13,                 0x12,       7,              16},
    {0x14,                 0x11,       7,              16},
    {0x15,                 0x12,       7,              16},
    {0x16,                 0x11,       7,              16},
    {0x17,                 0x12,       7,              16},
    {0x18,                 0x11,       7,              16},
    {0x19,                 0x12,       7,              16},
};


const spi_test_reg_t spi_dac2_test_regs[] = 
{
    /* REG_ADDR,         REG_VALUE,  REG_ADDR_SIZE,  REG_VALUE_SIZE */
    {0x00,                 0x01,       7,              16},
    {0x01,                 0x02,       7,              16},
    {0x02,                 0x03,       7,              16},
    {0x03,                 0x04,       7,              16},
    {0x04,                 0x05,       7,              16},
    {0x05,                 0x06,       7,              16},
    {0x06,                 0x07,       7,              16},
    {0x07,                 0x08,       7,              16},
    {0x08,                 0x09,       7,              16},
    // {0x09,                 0x0A,       7,              16},  // Not exist
    // {0x0A,                 0x0B,       7,              16}, // Not exist
    // {0x0B,                 0x0C,       7,              16}, // Not exist
    // {0x0C,                 0x02,       7,              16}, // Not exist
    // {0x0D,                 0x07,       7,              16}, // Not exist 
    // {0x0E,                 0x00,       7,              16}, // Not exist
    // {0x0F,                 0x00,       7,              16}, // Not exist
    {0x10,                 0x80,       7,              16}, 
    {0x11,                 0x03,       7,              16},
    {0x12,                 0x11,       7,              16},
    {0x13,                 0x12,       7,              16},
    {0x14,                 0x11,       7,              16},
    {0x15,                 0x12,       7,              16},
    {0x16,                 0x11,       7,              16},
    {0x17,                 0x12,       7,              16},
    {0x18,                 0x11,       7,              16},
    {0x19,                 0x12,       7,              16},
};

const spi_test_reg_t spi_phy_test_regs[] = 
{
    /* REG_ADDR,         REG_VALUE,  REG_ADDR_SIZE,  REG_VALUE_SIZE */
    {0x01,                 0x32,       6,              8},
    {0x02,                 0x28,       6,              8},
    {0x03,                 0x00,       6,              8},
    {0x04,                 0x00,       6,              8},
    {0x05,                 0x22,       6,              8},
    {0x06,                 0x28,       6,              8},
    {0x07,                 0x30,       6,              8},
    {0x08,                 0x01,       6,              8},
    {0x09,                 0x23,       6,              8},
    {0x0A,                 0xC8,       6,              8},
    {0x0B,                 0x95,       6,              8},
    {0x0C,                 0x42,       6,              8},
    {0x0D,                 0x40,       6,              8},
    {0x0E,                 0x00,       6,              8},
    {0x0F,                 0xF3,       6,              8},
    {0x10,                 0x01,       6,              8},
    {0x11,                 0x26,       6,              8},
    {0x12,                 0x80,       6,              8},
    {0x13,                 0x60,       6,              8},
    {0x14,                 0x00,       6,              8},
    {0x15,                 0x00,       6,              8},
    {0x16,                 0x00,       6,              8},
    {0x17,                 0x20,       6,              8},
    {0x18,                 0x14,       6,              8},
    {0x19,                 0xA0,       6,              8},
    {0x1A,                 0x47,       6,              8},
    {0x1B,                 0x40,       6,              8},
    {0x1C,                 0x00,       6,              8},
    {0x1D,                 0x00,       6,              8},
    {0x1E,                 0x7F,       6,              8},
    {0x1F,                 0xA2,       6,              8},
    {0x20,                 0x00,       6,              8},
    {0x21,                 0x01,       6,              8},
    {0x22,                 0x28,       6,              8},
    {0x00,                 0x01,       6,              8},
};

const spi_test_reg_t spi_jesd_tx1_test_regs[] = 
{
    /* REG_ADDR,         REG_VALUE,  REG_ADDR_SIZE,  REG_VALUE_SIZE */
    {0x0008,              0X0A418820,  10,             32},
    {0x000C,              0X16A4A0E6,  10,             32},
    {0x0010,              0X2307B9AC,  10,             32},
    {0x0014,              0X2F6AD272,  10,             32},
    {0x013C,              0X00000001,  10,             32},
    {0x001C,              0X00000001,  10,             32},
    {0x0020,              0X00040000,  10,             32},
    {0x0024,              0X00000000,  10,             32},
    {0x0028,              0X00000001,  10,             32},
    {0x002C,              0X0000000F,  10,             32},
    {0x0030,              0X00000001,  10,             32},
    {0x0034,              0X0000000F,  10,             32},
    {0x0038,              0X00000001,  10,             32},
    {0x003C,              0X0000000F,  10,             32},
    {0x0040,              0X00000001,  10,             32},
    {0x0368,              0X00402081,  10,             32},
    {0x0044,              0X00000020,  10,             32},
    {0x02C0,              0X0000007E,  10,             32},
    {0x03E0,              0X00000001,  10,             32},
    {0x012C,              0X03000020,  10,             32},
    {0x0054,              0X00000000,  10,             32},
    {0x02DC,              0X00000000,  10,             32},
    {0x0048,              0X00000000,  10,             32},
    {0x0050,              0X00000001,  10,             32},
    {0x02E0,              0X00000000,  10,             32},
    {0x02E4,              0X00000000,  10,             32},
    {0x02D0,              0X00000000,  10,             32},
    {0x03F0,              0X00000000,  10,             32}
};

const spi_test_reg_t spi_jesd_tx2_test_regs[] = 
{
    /* REG_ADDR,         REG_VALUE,  REG_ADDR_SIZE,  REG_VALUE_SIZE */
    {0x0008,              0X0A418820,  10,             32},
    {0x000C,              0X16A4A0E6,  10,             32},
    {0x0010,              0X2307B9AC,  10,             32},
    {0x0014,              0X2F6AD272,  10,             32},
    {0x013C,              0X00000001,  10,             32},
    {0x001C,              0X00000001,  10,             32},
    {0x0020,              0X00040000,  10,             32},
    {0x0024,              0X00000000,  10,             32},
    {0x0028,              0X00000001,  10,             32},
    {0x002C,              0X0000000F,  10,             32},
    {0x0030,              0X00000001,  10,             32},
    {0x0034,              0X0000000F,  10,             32},
    {0x0038,              0X00000001,  10,             32},
    {0x003C,              0X0000000F,  10,             32},
    {0x0040,              0X00000001,  10,             32},
    {0x0368,              0X00402081,  10,             32},
    {0x0044,              0X00000020,  10,             32},
    {0x02C0,              0X0000007E,  10,             32},
    {0x03E0,              0X00000001,  10,             32},
    {0x012C,              0X03000020,  10,             32},
    {0x0054,              0X00000000,  10,             32},
    {0x02DC,              0X00000000,  10,             32},
    {0x0048,              0X00000000,  10,             32},
    {0x0050,              0X00000001,  10,             32},
    {0x02E0,              0X00000000,  10,             32},
    {0x02E4,              0X00000000,  10,             32},
    {0x02D0,              0X00000000,  10,             32},
    {0x03F0,              0X00000000,  10,             32}
};

const spi_test_reg_t spi_jesd_rx_test_regs[] = 
{
    /* REG_ADDR,         REG_VALUE,  REG_ADDR_SIZE,  REG_VALUE_SIZE */
    {0x0008,              0X0A418820,  10,             32},
    {0x000C,              0X16A4A0E6,  10,             32},
    {0x0010,              0X2307B9AC,  10,             32},
    {0x0014,              0X2F6AD272,  10,             32},
    {0x013C,              0X00000001,  10,             32},
    {0x001C,              0X00000001,  10,             32},
    {0x0020,              0X00000000,  10,             32},
    {0x0024,              0X00000000,  10,             32},
    {0x0028,              0X00000001,  10,             32},
    {0x002C,              0X0000000F,  10,             32},
    {0x0030,              0X00000003,  10,             32},
    {0x0034,              0X0000000F,  10,             32},
    {0x0038,              0X00000003,  10,             32},
    {0x003C,              0X0000000F,  10,             32},
    {0x0040,              0X00000001,  10,             32},
    {0x0368,              0X00402081,  10,             32},
    {0x012C,              0X03000020,  10,             32},
    {0x0044,              0X00000000,  10,             32},
    {0x0054,              0X00000000,  10,             32},
    {0x02DC,              0X00000000,  10,             32},
    {0x0048,              0X00000000,  10,             32},
    {0x0050,              0X00000001,  10,             32},
    {0x02E0,              0X00000000,  10,             32},
    {0x02E4,              0X00000000,  10,             32},
    {0x02E8,              0X00000001,  10,             32},
    {0x03F4,              0X00000000,  10,             32}
};

const spi_test_reg_t spi_qec_test_regs[] = 
{
    /* REG_ADDR,         REG_VALUE,  REG_ADDR_SIZE,  REG_VALUE_SIZE */
    {0x0,                 0x01,       10,             8},
    {0x1,                 0x00,       10,             8},
    {0x2,                 0x00,       10,             8},
    {0x3,                 0x1B,       10,             8},
    {0x4,                 0x58,       10,             8},
    {0x5,                 0x00,       10,             8},
    {0x6,                 0x00,       10,             8},
    {0x7,                 0x27,       10,             8},
    {0x8,                 0x10,       10,             8},
    {0x9,                 0x00,       10,             8},
    {0xA,                 0x00,       10,             8},
    {0xB,                 0xC3,       10,             8},
    {0xC,                 0x50,       10,             8},
    {0xD,                 0x00,       10,             8},
    {0xE,                 0x9D,       10,             8},
    {0xF,                 0x51,       10,             8},
    {0x10,                0x70,       10,             8},
    {0x11,                0x02,       10,             8},
    {0x12,                0x29,       10,             8},
    {0x13,                0xE4,       10,             8},
    {0x14,                0xE0,       10,             8},
    {0x15,                0xFF,       10,             8},
    {0x16,                0xFF,       10,             8},
    {0x17,                0xFF,       10,             8},
    {0x18,                0x00,       10,             8},
    {0x19,                0xFF,       10,             8},
    {0x1A,                0xFF,       10,             8},
    {0x1B,                0xFF,       10,             8},
    {0x1C,                0x02,       10,             8},
    {0x1D,                0xFF,       10,             8},
    {0x1E,                0xFF,       10,             8},
    {0x1F,                0xFF,       10,             8},
    {0x20,                0x04,       10,             8},
    {0x21,                0x00,       10,             8},
    {0x22,                0x0F,       10,             8},
    {0x23,                0x42,       10,             8},
    {0x24,                0x40,       10,             8},
    {0x25,                0x00,       10,             8},
    {0x26,                0x03,       10,             8},
    {0x27,                0x33,       10,             8},
    {0x28,                0x33,       10,             8},
    {0x29,                0x00,       10,             8},
    {0x2A,                0x02,       10,             8},
    {0x2B,                0x49,       10,             8},
    {0x2C,                0xF0,       10,             8},
    {0x2D,                0x00,       10,             8},
    {0x2E,                0x00,       10,             8},
    {0x2F,                0x08,       10,             8},
    {0x30,                0x32,       10,             8},
    {0x31,                0x00,       10,             8},
    {0x32,                0x00,       10,             8},
    {0x33,                0x04,       10,             8},
    {0x34,                0x18,       10,             8},
    {0x35,                0x00,       10,             8},
    {0x36,                0x00,       10,             8},
    {0x37,                0x04,       10,             8},
    {0x38,                0x18,       10,             8},
    {0x39,                0x00,       10,             8},
    {0x3A,                0x00,       10,             8},
    {0x3B,                0x04,       10,             8},
    {0x3C,                0x18,       10,             8},
    {0x3D,                0x00,       10,             8},
    {0x3E,                0x00,       10,             8},
    {0x3F,                0x04,       10,             8},
    {0x40,                0x18,       10,             8}
};

const spi_test_reg_t spi_fir_tx_test_regs[] = 
{
    /* REG_ADDR,         REG_VALUE,  REG_ADDR_SIZE,  REG_VALUE_SIZE */
    {0x0,                 0x1,        10,             8},
    {0x1,                 0x60,       10,             8},
    {0x2,                 0x0,        10,             8},
    {0x3,                 0x1,        10,             8},
    {0x4,                 0x2,        10,             8},
    {0x5,                 0x3,        10,             8},
    {0x6,                 0x4,        10,             8},
    {0x7,                 0x5,        10,             8},
    {0x8,                 0x6,        10,             8},
    {0x9,                 0x7,        10,             8},
    {0xA,                 0x8,        10,             8},
    {0xB,                 0x9,        10,             8},
    {0xC,                 0xA,        10,             8},
    {0xD,                 0xB,        10,             8},
    {0xE,                 0xC,        10,             8},
    {0xF,                 0xD,        10,             8},
    {0x10,                0xE,        10,             8},
    {0x11,                0xF,        10,             8},
    {0x12,                0x10,       10,             8},
    {0x13,                0x11,       10,             8},
    {0x14,                0x12,       10,             8},
    {0x15,                0x13,       10,             8},
    {0x16,                0x14,       10,             8},
    {0x17,                0x15,       10,             8},
    {0x18,                0x16,       10,             8},
    {0x19,                0x17,       10,             8},
    {0x1A,                0x18,       10,             8},
    {0x1B,                0x19,       10,             8},
    {0x1C,                0x1A,       10,             8},
    {0x1D,                0x1B,       10,             8},
    {0x1E,                0x1C,       10,             8},
    {0x1F,                0x1D,       10,             8},
    {0x20,                0x1E,       10,             8},
    {0x21,                0x1F,       10,             8},
    {0x22,                0x20,       10,             8},
    {0x23,                0x21,       10,             8},
    {0x24,                0x22,       10,             8},
    {0x25,                0x23,       10,             8},
    {0x26,                0x24,       10,             8},
    {0x27,                0x25,       10,             8},
    {0x28,                0x26,       10,             8},
    {0x29,                0x27,       10,             8},
    {0x2A,                0x28,       10,             8},
    {0x2B,                0x29,       10,             8},
    {0x2C,                0x2A,       10,             8},
    {0x2D,                0x2B,       10,             8},
    {0x2E,                0x2C,       10,             8},
    {0x2F,                0x2D,       10,             8},
    {0x30,                0x2E,       10,             8},
    {0x31,                0x2F,       10,             8},
    {0x32,                0x30,       10,             8},
    {0x33,                0x31,       10,             8},
    {0x34,                0x32,       10,             8},
    {0x35,                0x33,       10,             8},
    {0x36,                0x34,       10,             8},
    {0x37,                0x35,       10,             8},
    {0x38,                0x36,       10,             8},
    {0x39,                0x37,       10,             8},
    {0x3A,                0x38,       10,             8},
    {0x3B,                0x39,       10,             8},
    {0x3C,                0x3A,       10,             8},
    {0x3D,                0x3B,       10,             8},
    {0x3E,                0x3C,       10,             8},
    {0x3F,                0x3D,       10,             8},
    {0x40,                0x3E,       10,             8},
    {0x41,                0x3F,       10,             8},
    {0x42,                0x40,       10,             8},
    {0x43,                0x41,       10,             8},
    {0x44,                0x42,       10,             8},
    {0x45,                0x43,       10,             8},
    {0x46,                0x44,       10,             8},
    {0x47,                0x45,       10,             8},
    {0x48,                0x46,       10,             8},
    {0x49,                0x47,       10,             8},
    {0x4A,                0x48,       10,             8},
    {0x4B,                0x49,       10,             8},
    {0x4C,                0x4A,       10,             8},
    {0x4D,                0x4B,       10,             8},
    {0x4E,                0x4C,       10,             8},
    {0x4F,                0x4D,       10,             8},
    {0x50,                0x4E,       10,             8},
    {0x51,                0x4F,       10,             8},
    {0x52,                0x6,        10,             8},
    {0x53,                0xAF,       10,             8},
    {0x54,                0x0,        10,             8},
    {0x55,                0x5,        10,             8},
    {0x56,                0xFF,       10,             8},
    {0x57,                0xF5,       10,             8},
    {0x58,                0xFF,       10,             8},
    {0x59,                0xF4,       10,             8},
    {0x5A,                0x0,        10,             8},
    {0x5B,                0x8,        10,             8},
    {0x5C,                0x0,        10,             8},
    {0x5D,                0x18,       10,             8},
    {0x5E,                0x0,        10,             8},
    {0x5F,                0x3,        10,             8},
    {0x60,                0xFF,       10,             8},
    {0x61,                0xDE,       10,             8},
    {0x62,                0xFF,       10,             8},
    {0x63,                0xE3,       10,             8},
    {0x64,                0x0,        10,             8},
    {0x65,                0x1C,       10,             8},
    {0x66,                0x0,        10,             8},
    {0x67,                0x3B,       10,             8},
    {0x68,                0x0,        10,             8},
    {0x69,                0x0,        10,             8},
    {0x6A,                0xFF,       10,             8},
    {0x6B,                0xAF,       10,             8},
    {0x6C,                0xFF,       10,             8},
    {0x6D,                0xC8,       10,             8},
    {0x6E,                0x0,        10,             8},
    {0x6F,                0x47,       10,             8},
    {0x70,                0x0,        10,             8},
    {0x71,                0x79,       10,             8},
    {0x72,                0xFF,       10,             8},
    {0x73,                0xF1,       10,             8},
    {0x74,                0xFF,       10,             8},
    {0x75,                0x58,       10,             8},
    {0x76,                0xFF,       10,             8},
    {0x77,                0xA4,       10,             8},
    {0x78,                0x0,        10,             8},
    {0x79,                0x9C,       10,             8},
    {0x7A,                0x0,        10,             8},
    {0x7B,                0xDB,       10,             8},
    {0x7C,                0xFF,       10,             8},
    {0x7D,                0xC6,       10,             8},
    {0x7E,                0xFE,       10,             8},
    {0x7F,                0xC3,       10,             8},
    {0x80,                0xFF,       10,             8},
    {0x81,                0x7B,       10,             8},
    {0x82,                0x1,        10,             8},
    {0x83,                0x3A,       10,             8},
    {0x84,                0x1,        10,             8},
    {0x85,                0x73,       10,             8},
    {0x86,                0xFF,       10,             8},
    {0x87,                0x62,       10,             8},
    {0x88,                0xFD,       10,             8},
    {0x89,                0xC3,       10,             8},
    {0x8A,                0xFF,       10,             8},
    {0x8B,                0x52,       10,             8},
    {0x8C,                0x2,        10,             8},
    {0x8D,                0x6A,       10,             8},
    {0x8E,                0x2,        10,             8},
    {0x8F,                0x73,       10,             8},
    {0x90,                0xFE,       10,             8},
    {0x91,                0x7B,       10,             8},
    {0x92,                0xFB,       10,             8},
    {0x93,                0xC5,       10,             8},
    {0x94,                0xFF,       10,             8},
    {0x95,                0x32,       10,             8},
    {0x96,                0x5,        10,             8},
    {0x97,                0x42,       10,             8},
    {0x98,                0x4,        10,             8},
    {0x99,                0xCC,       10,             8},
    {0x9A,                0xFB,       10,             8},
    {0x9B,                0x90,       10,             8},
    {0x9C,                0xF4,       10,             8},
    {0x9D,                0xE8,       10,             8},
    {0x9E,                0xFF,       10,             8},
    {0x9F,                0x20,       10,             8},
    {0xA0,                0x19,       10,             8},
    {0xA1,                0x74,       10,             8},
    {0xA2,                0x30,       10,             8},
    {0xA3,                0x98,       10,             8},
    {0xA4,                0x30,       10,             8},
    {0xA5,                0x98,       10,             8},
    {0xA6,                0x19,       10,             8},
    {0xA7,                0x74,       10,             8},
    {0xA8,                0xFF,       10,             8},
    {0xA9,                0x20,       10,             8},
    {0xAA,                0xF4,       10,             8},
    {0xAB,                0xE8,       10,             8},
    {0xAC,                0xFB,       10,             8},
    {0xAD,                0x90,       10,             8},
    {0xAE,                0x4,        10,             8},
    {0xAF,                0xCC,       10,             8},
    {0xB0,                0x5,        10,             8},
    {0xB1,                0x42,       10,             8},
    {0xB2,                0xFF,       10,             8},
    {0xB3,                0x32,       10,             8},
    {0xB4,                0xFB,       10,             8},
    {0xB5,                0xC5,       10,             8},
    {0xB6,                0xFE,       10,             8},
    {0xB7,                0x7B,       10,             8},
    {0xB8,                0x2,        10,             8},
    {0xB9,                0x73,       10,             8},
    {0xBA,                0x2,        10,             8},
    {0xBB,                0x6A,       10,             8},
    {0xBC,                0xFF,       10,             8},
    {0xBD,                0x52,       10,             8},
    {0xBE,                0xFD,       10,             8},
    {0xBF,                0xC3,       10,             8},
    {0xC0,                0xFF,       10,             8},
    {0xC1,                0x62,       10,             8},
    {0xC2,                0x1,        10,             8},
    {0xC3,                0x73,       10,             8},
    {0xC4,                0x1,        10,             8},
    {0xC5,                0x3A,       10,             8},
    {0xC6,                0xFF,       10,             8},
    {0xC7,                0x7B,       10,             8},
    {0xC8,                0xFE,       10,             8},
    {0xC9,                0xC3,       10,             8},
    {0xCA,                0xFF,       10,             8},
    {0xCB,                0xC6,       10,             8},
    {0xCC,                0x0,        10,             8},
    {0xCD,                0xDB,       10,             8},
    {0xCE,                0x0,        10,             8},
    {0xCF,                0x9C,       10,             8},
    {0xD0,                0xFF,       10,             8},
    {0xD1,                0xA4,       10,             8},
    {0xD2,                0xFF,       10,             8},
    {0xD3,                0x58,       10,             8},
    {0xD4,                0xFF,       10,             8},
    {0xD5,                0xF1,       10,             8},
    {0xD6,                0x0,        10,             8},
    {0xD7,                0x79,       10,             8},
    {0xD8,                0x0,        10,             8},
    {0xD9,                0x47,       10,             8},
    {0xDA,                0xFF,       10,             8},
    {0xDB,                0xC8,       10,             8},
    {0xDC,                0xFF,       10,             8},
    {0xDD,                0xAF,       10,             8},
    {0xDE,                0x0,        10,             8},
    {0xDF,                0x0,        10,             8},
    {0xE0,                0x0,        10,             8},
    {0xE1,                0x3B,       10,             8},
    {0xE2,                0x0,        10,             8},
    {0xE3,                0x1C,       10,             8},
    {0xE4,                0xFF,       10,             8},
    {0xE5,                0xE3,       10,             8},
    {0xE6,                0xFF,       10,             8},
    {0xE7,                0xDE,       10,             8},
    {0xE8,                0x0,        10,             8},
    {0xE9,                0x3,        10,             8},
    {0xEA,                0x0,        10,             8},
    {0xEB,                0x18,       10,             8},
    {0xEC,                0x0,        10,             8},
    {0xED,                0x8,        10,             8},
    {0xEE,                0xFF,       10,             8},
    {0xEF,                0xF4,       10,             8},
    {0xF0,                0xFF,       10,             8},
    {0xF1,                0xF5,       10,             8},
    {0xF2,                0x0,        10,             8},
    {0xF3,                0x5,        10,             8},
};

const spi_test_reg_t spi_fir_rx_test_regs[] = {
/* REG_ADDR,         REG_VALUE,  REG_ADDR_SIZE,  REG_VALUE_SIZE */
    {421,           0b00010000,        10,          8},
    {422,           0b00000000,        10,          8},
    {423,           0b00001111,        10,          8},
    {424,           0b11011111,        10,          8},
    {425,           0b00001111,        10,          8},
    {426,           0b01111111,        10,          8},
    {427,           0b00001110,        10,          8},
    {428,           0b11100000,        10,          8},
    {429,           0b00001110,        10,          8},
    {430,           0b00000101,        10,          8},
    {431,           0b00001100,        10,          8},
    {432,           0b11110010,        10,          8},
    {433,           0b00001011,        10,          8},
    {434,           0b10101010,        10,          8},
    {435,           0b00001010,        10,          8},
    {436,           0b00110011,        10,          8},
    {437,           0b00001000,        10,          8},
    {438,           0b10010011,        10,          8},
    {439,           0b00000110,        10,          8},
    {440,           0b11010000,        10,          8},
    {441,           0b00000100,        10,          8},
    {442,           0b11110010,        10,          8},
    {443,           0b00000010,        10,          8},
    {444,           0b11111111,        10,          8},
    {445,           0b00000001,        10,          8},
    {446,           0b00000001,        10,          8},
    {447,           0b11111110,        10,          8},
    {448,           0b11111111,        10,          8},
    {449,           0b11111101,        10,          8},
    {450,           0b00000000,        10,          8},
    {451,           0b11111011,        10,          8},
    {452,           0b00001110,        10,          8},
    {453,           0b11111001,        10,          8},
    {454,           0b00110000,        10,          8},
    {455,           0b11110111,        10,          8},
    {456,           0b01101101,        10,          8},
    {457,           0b11110101,        10,          8},
    {458,           0b11001101,        10,          8},
    {459,           0b11110100,        10,          8},
    {460,           0b01010110,        10,          8},
    {461,           0b11110011,        10,          8},
    {462,           0b00001110,        10,          8},
    {463,           0b11110001,        10,          8},
    {464,           0b11111011,        10,          8},
    {465,           0b11110001,        10,          8},
    {466,           0b00100000,        10,          8},
    {467,           0b11110000,        10,          8},
    {468,           0b10000001,        10,          8},
    {469,           0b11110000,        10,          8},
    {470,           0b00100000,        10,          8},
    {471,           0b11110000,        10,          8},
    {472,           0b00000000,        10,          8},
    {473,           0b11110000,        10,          8},
    {474,           0b00100000,        10,          8},
    {475,           0b11110000,        10,          8},
    {476,           0b10000001,        10,          8},
    {477,           0b11110001,        10,          8},
    {478,           0b00100000,        10,          8},
    {479,           0b11110001,        10,          8},
    {480,           0b11111011,        10,          8},
    {481,           0b11110011,        10,          8},
    {482,           0b00001110,        10,          8},
    {483,           0b11110100,        10,          8},
    {484,           0b01010110,        10,          8},
    {485,           0b11110101,        10,          8},
    {486,           0b11001101,        10,          8},
    {487,           0b11110111,        10,          8},
    {488,           0b01101101,        10,          8},
    {489,           0b11111001,        10,          8},
    {490,           0b00110000,        10,          8},
    {491,           0b11111011,        10,          8},
    {492,           0b00001110,        10,          8},
    {493,           0b11111101,        10,          8},
    {494,           0b00000000,        10,          8},
    {495,           0b11111110,        10,          8},
    {496,           0b11111111,        10,          8},
    {497,           0b00000001,        10,          8},
    {498,           0b00000001,        10,          8},
    {499,           0b00000010,        10,          8},
    {500,           0b11111111,        10,          8},
    {501,           0b00000100,        10,          8},
    {502,           0b11110010,        10,          8},
    {503,           0b00000110,        10,          8},
    {504,           0b11010000,        10,          8},
    {505,           0b00001000,        10,          8},
    {506,           0b10010011,        10,          8},
    {507,           0b00001010,        10,          8},
    {508,           0b00110011,        10,          8},
    {509,           0b00001011,        10,          8},
    {510,           0b10101010,        10,          8},
    {511,           0b00001100,        10,          8},
    {512,           0b11110010,        10,          8},
    {513,           0b00001110,        10,          8},
    {514,           0b00000101,        10,          8},
    {515,           0b00001110,        10,          8},
    {516,           0b11100000,        10,          8},
    {517,           0b00001111,        10,          8},
    {518,           0b01111111,        10,          8},
    {519,           0b00001111,        10,          8},
    {520,           0b11011111,        10,          8},
    {521,           0b00000000,        10,          8},
    {522,           0b00000000,        10,          8},
    {523,           0b00000100,        10,          8},
    {524,           0b00000011,        10,          8},
    {525,           0b00000111,        10,          8},
    {526,           0b11110101,        10,          8},
    {527,           0b00001011,        10,          8},
    {528,           0b11000111,        10,          8},
    {529,           0b00001111,        10,          8},
    {530,           0b01101010,        10,          8},
    {531,           0b00010010,        10,          8},
    {532,           0b11001111,        10,          8},
    {533,           0b00010101,        10,          8},
    {534,           0b11100111,        10,          8},
    {535,           0b00011000,        10,          8},
    {536,           0b10101000,        10,          8},
    {537,           0b00011011,        10,          8},
    {538,           0b00000100,        10,          8},
    {539,           0b00011100,        10,          8},
    {540,           0b11110100,        10,          8},
    {541,           0b00011110,        10,          8},
    {542,           0b01101111,        10,          8},
    {543,           0b00011111,        10,          8},
    {544,           0b01101110,        10,          8},
    {545,           0b00011111,        10,          8},
    {546,           0b11101111,        10,          8},
    {547,           0b00011111,        10,          8},
    {548,           0b11101111,        10,          8},
    {549,           0b00011111,        10,          8},
    {550,           0b01101110,        10,          8},
    {551,           0b00011110,        10,          8},
    {552,           0b01101111,        10,          8},
    {553,           0b00011100,        10,          8},
    {554,           0b11110100,        10,          8},
    {555,           0b00011011,        10,          8},
    {556,           0b00000100,        10,          8},
    {557,           0b00011000,        10,          8},
    {558,           0b10101000,        10,          8},
    {559,           0b00010101,        10,          8},
    {560,           0b11100111,        10,          8},
    {561,           0b00010010,        10,          8},
    {562,           0b11001111,        10,          8},
    {563,           0b00001111,        10,          8},
    {564,           0b01101010,        10,          8},
    {565,           0b00001011,        10,          8},
    {566,           0b11000111,        10,          8},
    {567,           0b00000111,        10,          8},
    {568,           0b11110101,        10,          8},
    {569,           0b00000100,        10,          8},
    {570,           0b00000011,        10,          8},
    {571,           0b00000000,        10,          8},
    {572,           0b00000000,        10,          8},
    {573,           0b11111011,        10,          8},
    {574,           0b11111101,        10,          8},
    {575,           0b11111000,        10,          8},
    {576,           0b00001011,        10,          8},
    {577,           0b11110100,        10,          8},
    {578,           0b00111000,        10,          8},
    {579,           0b11110000,        10,          8},
    {580,           0b10010101,        10,          8},
    {581,           0b11101101,        10,          8},
    {582,           0b00110001,        10,          8},
    {583,           0b11101010,        10,          8},
    {584,           0b00011000,        10,          8},
    {585,           0b11100111,        10,          8},
    {586,           0b01011000,        10,          8},
    {587,           0b11100100,        10,          8},
    {588,           0b11111011,        10,          8},
    {589,           0b11100011,        10,          8},
    {590,           0b00001100,        10,          8},
    {591,           0b11100001,        10,          8},
    {592,           0b10010001,        10,          8},
    {593,           0b11100000,        10,          8},
    {594,           0b10010001,        10,          8},
    {595,           0b11100000,        10,          8},
    {596,           0b00010000,        10,          8},
    {597,           0b11100000,        10,          8},
    {598,           0b00010000,        10,          8},
    {599,           0b11100000,        10,          8},
    {600,           0b10010001,        10,          8},
    {601,           0b11100001,        10,          8},
    {602,           0b10010001,        10,          8},
    {603,           0b11100011,        10,          8},
    {604,           0b00001100,        10,          8},
    {605,           0b11100100,        10,          8},
    {606,           0b11111011,        10,          8},
    {607,           0b11100111,        10,          8},
    {608,           0b01011000,        10,          8},
    {609,           0b11101010,        10,          8},
    {610,           0b00011000,        10,          8},
    {611,           0b11101101,        10,          8},
    {612,           0b00110001,        10,          8},
    {613,           0b11110000,        10,          8},
    {614,           0b10010101,        10,          8},
    {615,           0b11110100,        10,          8},
    {616,           0b00111000,        10,          8},
    {617,           0b11111000,        10,          8},
    {618,           0b00001011,        10,          8},
    {619,           0b11111011,        10,          8},
    {620,           0b11111101,        10,          8},
    {1  ,           0b01100000,        10,          8},
    {2  ,           0b00000000,        10,          8},
    {3  ,           0b00000001,        10,          8},
    {4  ,           0b00000010,        10,          8},
    {5  ,           0b00000011,        10,          8},
    {6  ,           0b00000100,        10,          8},
    {7  ,           0b00000101,        10,          8},
    {8  ,           0b00000110,        10,          8},
    {9  ,           0b00000111,        10,          8},
    {10 ,           0b00001000,        10,          8},
    {11 ,           0b00001001,        10,          8},
    {12 ,           0b00001010,        10,          8},
    {13 ,           0b00001011,        10,          8},
    {14 ,           0b00001100,        10,          8},
    {15 ,           0b00001101,        10,          8},
    {16 ,           0b00001110,        10,          8},
    {17 ,           0b00001111,        10,          8},
    {18 ,           0b00010000,        10,          8},
    {19 ,           0b00010001,        10,          8},
    {20 ,           0b00010010,        10,          8},
    {21 ,           0b00010011,        10,          8},
    {22 ,           0b00010100,        10,          8},
    {23 ,           0b00010101,        10,          8},
    {24 ,           0b00010110,        10,          8},
    {25 ,           0b00010111,        10,          8},
    {26 ,           0b00100000,        10,          8},
    {27 ,           0b00100001,        10,          8},
    {28 ,           0b00100010,        10,          8},
    {29 ,           0b00100011,        10,          8},
    {30 ,           0b00100100,        10,          8},
    {31 ,           0b00100101,        10,          8},
    {32 ,           0b00100110,        10,          8},
    {33 ,           0b00100111,        10,          8},
    {34 ,           0b00101000,        10,          8},
    {35 ,           0b00101001,        10,          8},
    {36 ,           0b00101010,        10,          8},
    {37 ,           0b00101011,        10,          8},
    {38 ,           0b00101100,        10,          8},
    {39 ,           0b00101101,        10,          8},
    {40 ,           0b00101110,        10,          8},
    {41 ,           0b00101111,        10,          8},
    {42 ,           0b00110000,        10,          8},
    {43 ,           0b00110001,        10,          8},
    {44 ,           0b00110010,        10,          8},
    {45 ,           0b00110011,        10,          8},
    {46 ,           0b00110100,        10,          8},
    {47 ,           0b00110101,        10,          8},
    {48 ,           0b00110110,        10,          8},
    {49 ,           0b00110111,        10,          8},
    {50 ,           0b01000000,        10,          8},
    {51 ,           0b01000001,        10,          8},
    {52 ,           0b01000010,        10,          8},
    {53 ,           0b01000011,        10,          8},
    {54 ,           0b01000100,        10,          8},
    {55 ,           0b01000101,        10,          8},
    {56 ,           0b01000110,        10,          8},
    {57 ,           0b01000111,        10,          8},
    {58 ,           0b01001000,        10,          8},
    {59 ,           0b01001001,        10,          8},
    {60 ,           0b01001010,        10,          8},
    {61 ,           0b01001011,        10,          8},
    {62 ,           0b01001100,        10,          8},
    {63 ,           0b01001101,        10,          8},
    {64 ,           0b01001110,        10,          8},
    {65 ,           0b01001111,        10,          8},
    {66 ,           0b01010000,        10,          8},
    {67 ,           0b01010001,        10,          8},
    {68 ,           0b01010010,        10,          8},
    {69 ,           0b01010011,        10,          8},
    {70 ,           0b01010100,        10,          8},
    {71 ,           0b01010101,        10,          8},
    {72 ,           0b01010110,        10,          8},
    {73 ,           0b01010111,        10,          8},
    {74 ,           0b00000100,        10,          8},
    {75 ,           0b01101111,        10,          8},
    {76 ,           0b00000000,        10,          8},
    {77 ,           0b00000000,        10,          8},
    {78 ,           0b00000000,        10,          8},
    {79 ,           0b00000000,        10,          8},
    {80 ,           0b00000000,        10,          8},
    {81 ,           0b00000010,        10,          8},
    {82 ,           0b00000000,        10,          8},
    {83 ,           0b00000000,        10,          8},
    {84 ,           0b11111111,        10,          8},
    {85 ,           0b11111001,        10,          8},
    {86 ,           0b11111111,        10,          8},
    {87 ,           0b11111111,        10,          8},
    {88 ,           0b00000000,        10,          8},
    {89 ,           0b00001111,        10,          8},
    {90 ,           0b00000000,        10,          8},
    {91 ,           0b00000100,        10,          8},
    {92 ,           0b11111111,        10,          8},
    {93 ,           0b11100001,        10,          8},
    {94 ,           0b11111111,        10,          8},
    {95 ,           0b11110110,        10,          8},
    {96 ,           0b00000000,        10,          8},
    {97 ,           0b00111001,        10,          8},
    {98 ,           0b00000000,        10,          8},
    {99 ,           0b00010111,        10,          8},
    {100,           0b11111111,        10,          8},
    {101,           0b10011101,        10,          8},
    {102,           0b11111111,        10,          8},
    {103,           0b11010001,        10,          8},
    {104,           0b00000000,        10,          8},
    {105,           0b10100001,        10,          8},
    {106,           0b00000000,        10,          8},
    {107,           0b01010110,        10,          8},
    {108,           0b11111111,        10,          8},
    {109,           0b00001001,        10,          8},
    {110,           0b11111111,        10,          8},
    {111,           0b01101110,        10,          8},
    {112,           0b00000001,        10,          8},
    {113,           0b01110100,        10,          8},
    {114,           0b00000000,        10,          8},
    {115,           0b11101110,        10,          8},
    {116,           0b11111101,        10,          8},
    {117,           0b11100010,        10,          8},
    {118,           0b11111110,        10,          8},
    {119,           0b10000011,        10,          8},
    {120,           0b00000010,        10,          8},
    {121,           0b11111010,        10,          8},
    {122,           0b00000010,        10,          8},
    {123,           0b01000111,        10,          8},
    {124,           0b11111011,        10,          8},
    {125,           0b11011000,        10,          8},
    {126,           0b11111100,        10,          8},
    {127,           0b10001101,        10,          8},
    {128,           0b00000101,        10,          8},
    {129,           0b10111110,        10,          8},
    {130,           0b00000101,        10,          8},
    {131,           0b00110011,        10,          8},
    {132,           0b11110111,        10,          8},
    {133,           0b11110111,        10,          8},
    {134,           0b11110111,        10,          8},
    {135,           0b11110101,        10,          8},
    {136,           0b00001011,        10,          8},
    {137,           0b10011111,        10,          8},
    {138,           0b00001101,        10,          8},
    {139,           0b01001011,        10,          8},
    {140,           0b11101101,        10,          8},
    {141,           0b01110100,        10,          8},
    {142,           0b11100101,        10,          8},
    {143,           0b00001110,        10,          8},
    {144,           0b00100101,        10,          8},
    {145,           0b11100010,        10,          8},
    {146,           0b01111000,        10,          8},
    {147,           0b01010110,        10,          8},
    {148,           0b01111000,        10,          8},
    {149,           0b01010110,        10,          8},
    {150,           0b00100101,        10,          8},
    {151,           0b11100010,        10,          8},
    {152,           0b11100101,        10,          8},
    {153,           0b00001110,        10,          8},
    {154,           0b11101101,        10,          8},
    {155,           0b01110100,        10,          8},
    {156,           0b00001101,        10,          8},
    {157,           0b01001011,        10,          8},
    {158,           0b00001011,        10,          8},
    {159,           0b10011111,        10,          8},
    {160,           0b11110111,        10,          8},
    {161,           0b11110101,        10,          8},
    {162,           0b11110111,        10,          8},
    {163,           0b11110111,        10,          8},
    {164,           0b00000101,        10,          8},
    {165,           0b00000101,        10,          8},
    {166,           0b00110011,        10,          8},
    {167,           0b10111110,        10,          8},
    {168,           0b11111100,        10,          8},
    {169,           0b10001101,        10,          8},
    {170,           0b11111011,        10,          8},
    {171,           0b11011000,        10,          8},
    {172,           0b00000010,        10,          8},
    {173,           0b01000111,        10,          8},
    {174,           0b00000010,        10,          8},
    {175,           0b11111010,        10,          8},
    {176,           0b11111110,        10,          8},
    {177,           0b10000011,        10,          8},
    {178,           0b11111101,        10,          8},
    {179,           0b11100010,        10,          8},
    {180,           0b00000000,        10,          8},
    {181,           0b11101110,        10,          8},
    {182,           0b00000001,        10,          8},
    {183,           0b01110100,        10,          8},
    {184,           0b11111111,        10,          8},
    {185,           0b01101110,        10,          8},
    {186,           0b11111111,        10,          8},
    {187,           0b00001001,        10,          8},
    {188,           0b00000000,        10,          8},
    {189,           0b01010110,        10,          8},
    {190,           0b00000000,        10,          8},
    {191,           0b10100001,        10,          8},
    {192,           0b11111111,        10,          8},
    {193,           0b11010001,        10,          8},
    {194,           0b11111111,        10,          8},
    {195,           0b10011101,        10,          8},
    {196,           0b00000000,        10,          8},
    {197,           0b00010111,        10,          8},
    {198,           0b00000000,        10,          8},
    {199,           0b00111001,        10,          8},
    {200,           0b11111111,        10,          8},
    {201,           0b11110110,        10,          8},
    {202,           0b11111111,        10,          8},
    {203,           0b11100001,        10,          8},
    {204,           0b00000000,        10,          8},
    {205,           0b00000100,        10,          8},
    {206,           0b00000000,        10,          8},
    {207,           0b00001111,        10,          8},
    {208,           0b11111111,        10,          8},
    {209,           0b11111111,        10,          8},
    {210,           0b11111111,        10,          8},
    {211,           0b11111001,        10,          8},
    {212,           0b00000000,        10,          8},
    {213,           0b00000000,        10,          8},
    {214,           0b00000000,        10,          8},
    {215,           0b00000010,        10,          8},
    {216,           0b00000000,        10,          8},
    {217,           0b00000000,        10,          8},
    {218,           0b00000000,        10,          8},
    {219,           0b00000000,        10,          8},
    {0  ,           0b00000001,        10,          8},

};

/******************************************************************************
* Slave devices
*******************************************************************************/
/**
 * @brief Array of SPI slave devices.
 */
spi_slave_test_t spi_slave_devices[] = 
{
    /* SLAVE ADDR,                  SLAVE_NUMB_REGS,                         SLAVE_REGS,                    SLAVE_NAME*/
//	{SPI_RX1_SLAVE_ADDR,            ARRAY_SIZE(spi_rx1_test_regs),          spi_rx1_test_regs,              "RX1"},
//	{SPI_RX2_SLAVE_ADDR,            ARRAY_SIZE(spi_rx2_test_regs),          spi_rx2_test_regs,              "RX2"},
//	{SPI_ORX1_SLAVE_ADDR,           ARRAY_SIZE(spi_orx1_test_regs),         spi_orx1_test_regs,             "ORX1"},
//	{SPI_ORX2_SLAVE_ADDR,           ARRAY_SIZE(spi_orx2_test_regs),         spi_orx2_test_regs,             "ORX2"},
//	{SPI_TX1_SLAVE_ADDR,            ARRAY_SIZE(spi_tx1_test_regs),          spi_tx1_test_regs,              "TX1"},
//	{SPI_TX2_SLAVE_ADDR,            ARRAY_SIZE(spi_tx2_test_regs),          spi_tx2_test_regs,              "TX2"},
//	{SPI_PLL_SLAVE_ADDR,            ARRAY_SIZE(spi_pll_test_regs),          spi_pll_test_regs,              "PLL"},
	{SPI_RX1_RSSI_SLAVE_ADDR,       ARRAY_SIZE(spi_rx1_rssi_test_regs),     spi_rx1_rssi_test_regs,         "RX1 RSSI"},
	{SPI_RX2_RSSI_SLAVE_ADDR,       ARRAY_SIZE(spi_rx2_rssi_test_regs),     spi_rx2_rssi_test_regs,         "RX2 RSSI"},
//	{SPI_DAC1_SLAVE_ADDR,           ARRAY_SIZE(spi_dac1_test_regs),         spi_dac1_test_regs,             "DAC1"},
//	{SPI_DAC2_SLAVE_ADDR,           ARRAY_SIZE(spi_dac2_test_regs),         spi_dac2_test_regs,             "DAC2"},
//	{SPI_ADC1_SLAVE_ADDR,           ARRAY_SIZE(spi_adc1_test_regs),         spi_adc1_test_regs,             "ADC1"},
//	{SPI_ADC2_SLAVE_ADDR,           ARRAY_SIZE(spi_adc2_test_regs),         spi_adc2_test_regs,             "ADC2"},
//	{SPI_ADC3_SLAVE_ADDR,           ARRAY_SIZE(spi_adc3_test_regs),         spi_adc3_test_regs,             "ADC3"},
//	{SPI_ADC4_SLAVE_ADDR,           ARRAY_SIZE(spi_adc4_test_regs),         spi_adc4_test_regs,             "ADC4"},
//
////  TODO:
//	{SPI_PHY_SLAVE_ADDR,            ARRAY_SIZE(spi_phy_test_regs),          spi_phy_test_regs,              "PHY"},
//	{SPI_JESD204_TX1_SLAVE_ADDR,    ARRAY_SIZE(spi_jesd_tx1_test_regs),     spi_jesd_tx1_test_regs,         "JESD204 TX1"},
//	{SPI_JESD204_TX2_SLAVE_ADDR,    ARRAY_SIZE(spi_jesd_tx2_test_regs),     spi_jesd_tx2_test_regs,         "JESD204 TX2"},
//	{SPI_JESD204_RX_SLAVE_ADDR,     ARRAY_SIZE(spi_jesd_rx_test_regs),      spi_jesd_rx_test_regs,          "JESD204 RX"},
//// TODO: Wait clock
//	{SPI_QEC_RX1_SLAVE_ADDR,        ARRAY_SIZE(spi_qec_test_regs),          spi_qec_test_regs,              "QEC RX1"},
//	{SPI_QEC_RX2_SLAVE_ADDR,        ARRAY_SIZE(spi_qec_test_regs),          spi_qec_test_regs,              "QEC RX2"},
//	{SPI_QEC_ORX1_SLAVE_ADDR,       ARRAY_SIZE(spi_qec_test_regs),          spi_qec_test_regs,              "QEC ORX1"},
//	{SPI_QEC_ORX2_SLAVE_ADDR,       ARRAY_SIZE(spi_qec_test_regs),          spi_qec_test_regs,              "QEC ORX2"},
//	{SPI_QEC_TX1_SLAVE_ADDR,        ARRAY_SIZE(spi_qec_test_regs),          spi_qec_test_regs,          	"QEC TX1"},
//	{SPI_QEC_TX2_SLAVE_ADDR,        ARRAY_SIZE(spi_qec_test_regs),          spi_qec_test_regs,          	"QEC TX2"},
//	{SPI_FIR_TX1_SLAVE_ADDR,        ARRAY_SIZE(spi_fir_tx_test_regs),  	    spi_fir_tx_test_regs,           "FIR TX1"},
//	{SPI_FIR_TX2_SLAVE_ADDR,        ARRAY_SIZE(spi_fir_tx_test_regs),  	    spi_fir_tx_test_regs,           "FIR TX2"},
//	{SPI_FIR_RX1_SLAVE_ADDR,        ARRAY_SIZE(spi_fir_rx_test_regs),       spi_fir_rx_test_regs,           "FIR RX1"},
//	{SPI_FIR_RX2_SLAVE_ADDR,        ARRAY_SIZE(spi_fir_rx_test_regs),       spi_fir_rx_test_regs,           "FIR RX2"},
//	{SPI_FIR_ORX1_SLAVE_ADDR,       ARRAY_SIZE(spi_fir_rx_test_regs),       spi_fir_rx_test_regs,           "FIR ORX1"},
//	{SPI_FIR_ORX2_SLAVE_ADDR,       ARRAY_SIZE(spi_fir_rx_test_regs),       spi_fir_rx_test_regs,           "FIR ORX2"},
};

//------------------------------------------------------------------------------------

spi_slave_test_t spi_slave_devices_for_cfg[] =
{
    /* SLAVE ADDR,                  SLAVE_NUMB_REGS,                         SLAVE_REGS,                    SLAVE_NAME*/
	{SPI_RX1_RSSI_SLAVE_ADDR,       ARRAY_SIZE(spi_rx_rssi_cfg_regs),     	spi_rx_rssi_cfg_regs,   	"RX1 RSSI"},
	{SPI_RX2_RSSI_SLAVE_ADDR,       ARRAY_SIZE(spi_rx_rssi_cfg_regs),     	spi_rx_rssi_cfg_regs,   	"RX2 RSSI"},
	{SPI_PLL_SLAVE_ADDR,       		ARRAY_SIZE(spi_pll_cfg_regs),     		spi_pll_cfg_regs,         	"PLL"},
	{SPI_PLL_SLAVE_ADDR,       		ARRAY_SIZE(spi_pll_cfg_regs_read),     	spi_pll_cfg_regs_read,      "PLL"},
	{SPI_PLL_SLAVE_ADDR,       		ARRAY_SIZE(spi_pll_cfg_regs_state_1),   spi_pll_cfg_regs_state_1,   "PLL"},
};

//------------------------------------------------------------------------------------


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
 */
int cfg_counter = 1;

/**
 * 0: Write
 * 1: Read
 */
int ctrl_token = 1;
unsigned int run_cnt = 0;
int read_delay = 10;
int spi_master_cfg() {
	int ret = 0;
	while (1) {
		if (ctrl_token == 0) {
			ret = spi_write_slave(spi_slave_devices_for_cfg[cfg_counter].slave_addr,
					spi_slave_devices_for_cfg[cfg_counter].reg,
					spi_slave_devices_for_cfg[cfg_counter].slave_numb_reg);
			if (ret != 0) {
				printf("[ERR] Failed to write to %s\n",
						spi_slave_devices_for_cfg[cfg_counter].slave_name);
			}
			HAL_Delay(3000); // 10_000 ms
//		cfg_counter = (cfg_counter == 0) ? 1 : 0;
		} else if (ctrl_token == 1) {
			run_cnt++;
			printf("%u\n", run_cnt);
			ret = spi_validate_slave(spi_slave_devices_for_cfg[cfg_counter].slave_addr,
					spi_slave_devices_for_cfg[cfg_counter].reg,
					spi_slave_devices_for_cfg[cfg_counter].slave_numb_reg);
			if (ret != 0) {
				printf("[ERR] Failed to write to %s\n",
						spi_slave_devices_for_cfg[cfg_counter].slave_name);
			}
			HAL_Delay(read_delay); // 3000 ms
			//		cfg_counter = (cfg_counter == 0) ? 1 : 0;
		}
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
