/****************************************************************************
* Title                 :   CMD handler header file
* Filename              :   cmd.h
* Author                :   thuantm5
* Origin Date           :   2024/04/03
* Version               :   v0.0.0
* Compiler              :   STM32CubeIDE
* Target                :   STM32F407-Disco
* Notes                 :   None
*****************************************************************************/

/** \file cmd.h
 *  \brief This module contains .
 *
 *  This is the header file for 
 */
#ifndef _CMD_CMD_H_
#define _CMD_CMD_H_

/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>


/******************************************************************************
* Preprocessor Constants
*******************************************************************************/
#define CMD_PREAMBLE                    (0x0A)	// Host -> Device
#define CMD_PREAMBLE_ACK                (0xA0)	// Device -> Host

#define CMD_TYPE_SPI_WRITE              (0x01)

// CMD ID
#define CMD_SPI_WRITE_SLAVE             (0x01)	// Write only
#define CMD_SPI_WRITE_SLAVE_ACK         (0x02)	// Write and verify
#define CMD_SPI_READ_SLAVE				(0x03)	// Read
#define CMD_SPI_AUTO_READ				(0x04)	// Auto read until stop or new cmd
/******************************************************************************
* Configuration Constants
*******************************************************************************/


/******************************************************************************
* Macros
*******************************************************************************/
#define FIELD_SIZEOF(obj_t, member_t) (sizeof(((obj_t*)0)->member_t))

/******************************************************************************
* Typedefs
*******************************************************************************/
typedef struct __attribute__((packed))
{
    uint8_t frame_header;
    uint16_t frame_len;
    uint8_t* p_frame_payload;
    uint16_t frame_crc;
} frame_t;

typedef int (*cmd_handler)(void* p_param, void* p_param_len);

typedef struct
{
    uint8_t cmd_id;
    cmd_handler executer;
}cmd_t;

typedef enum 								
{
	 ERR_OK = 0,						// "No error"
	 ERR_INVALID_FRAME = 0x10,			// "Invalid frame"
	 ERR_INVALID_LEN,					// "Invalid length"
	 ERR_INVALID_CMD,					// "Invalid command"
	 ERR_INVALID_CRC,					// "Invalid CRC"
	 ERR_CMD_EXEC_FAIL,					// "Command execution failed"
	 ERR_CMD_TIMEOUT,					// "Command timeout"
	 ERR_UNKNOWN = 0xFF,				// "Unknown error"
} error_code_t; 						// User error codes		


/******************************************************************************
* Variables
*******************************************************************************/


/******************************************************************************
* Function Prototypes
*******************************************************************************/

#ifdef __cplusplus
extern "C"{
#endif

int frame_processing(uint8_t* p_input, uint16_t* p_input_len,
                     uint8_t *p_output, uint16_t* p_output_maxlen);
// CMD declaration
int cmd_spi_write_slave(uint8_t* p_payload, uint16_t* p_len);
int cmd_spi_write_slave_ack(uint8_t* p_payload, uint16_t* p_len);
int cmd_spi_read_slave(uint8_t* p_payload, uint16_t* p_len);
int cmd_spi_auto_read(uint8_t* p_payload, uint16_t* p_len);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // _CMD_CMD_H_

/*** End of File **************************************************************/
