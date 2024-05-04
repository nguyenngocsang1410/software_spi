/*******************************************************************************
* Title                 :   CMD handler
* Filename              :   cmd.c
* Origin Date           :   2024/04/03
* Version               :   0.0.0
* Compiler              :   STM32CubeIDE
* Target                :   STM32F407-Disco
* Notes                 :   None
*******************************************************************************/

/******************************************************************************
* Includes
*******************************************************************************/
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include "cmd.h"

#include "../test_spi/spi_test.h"
/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/

/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
static cmd_t cmd_table[] = 
{
    {.cmd_id = CMD_SPI_WRITE_SLAVE,               .executer = (cmd_handler)&cmd_spi_write_slave},
    {.cmd_id = CMD_SPI_WRITE_SLAVE_ACK,           .executer = (cmd_handler)&cmd_spi_write_slave_ack},
};
/******************************************************************************
* Module Function
*******************************************************************************/
int cmd_process(uint8_t* p_payload, uint16_t* p_len);
int frame_resp(uint8_t resp_code, uint8_t* p_cmd_payload, uint16_t* p_cmd_len,
                                  uint8_t* p_frame_resp, uint16_t* p_frame_max_len);
/******************************************************************************
* Function Definitions
*******************************************************************************/
int frame_processing(uint8_t* p_input, uint16_t* p_input_len,
                     uint8_t *p_output, uint16_t* p_output_maxlen)
{
    assert(p_input != NULL);
    assert(p_input_len != NULL);

    uint16_t frame_len = *p_input_len;
    uint8_t frame_status = -1;

    uint16_t cmd_len =  frame_len - 1 - 2 - 2; /* Ignore 1B PRE + 2B Len + 2B CRC */
    uint8_t cmd_idx = offsetof(frame_t, p_frame_payload); /* Command index in frame received */

    // Received frame
    printf("\r\n FRAME: ");
    for(uint8_t idx=0; idx< frame_len; idx++)
        printf("%02X", p_input[idx]);
    printf("\r\n");

    do
    {
        /* Validate frame header */
        if( CMD_PREAMBLE != *((uint8_t*)p_input))
        {
            frame_status = ERR_INVALID_FRAME;
            printf(" \r\n [ERR]Invalid frame header \r\n");
            break;
        }

        /* Validate length */
        uint16_t len_expect = ((uint8_t)p_input[2] <<8) | (uint8_t)p_input[1];
        if(len_expect != (frame_len - 1 - 2)) /* Ignore 1B PRE + 2B Len */
        {
            frame_status = ERR_INVALID_LEN;
            printf(" \r\n [ERR]Invalid frame len \r\n");
            break; 
        }

        /* Validate checksum */
        uint16_t sum_cal=0;
        uint16_t sum_recv = ((uint8_t)p_input[frame_len-1] <<8) | (uint8_t)p_input[frame_len-2];
        for(uint8_t idx=0; idx < frame_len - 2; idx++)
        {
            sum_cal += p_input[idx];
        }
        if(sum_cal != sum_recv)
        {
            frame_status = ERR_INVALID_CRC;
            printf(" \r\n [ERR]Invalid CRC, cal_data: 0x%04X, recv_data: 0x%04X \r\n", sum_cal, sum_recv);
            break;
        }

        /* Execute command */

        // print CMD data
        printf("\r\n REQ: ");
        for(uint8_t idx=0; idx< cmd_len; idx++)
            printf("%02X", p_input[cmd_idx + idx]);
        printf("\r\n");

        frame_status = cmd_process(&p_input[cmd_idx], &cmd_len);

    } while (0);

    // Construct frame response
    int ret_val = 0;
    // Can't execute command, return err code
    if ((frame_status > ERR_OK) &&  (frame_status <= ERR_INVALID_CRC) )
    {
        char output_msg[20] = {0};
        sprintf(output_msg, "ERR: 0x%02X \r\n", frame_status);
        uint16_t len = strlen(output_msg);
        uint16_t max_len = *p_output_maxlen;
        uint16_t out_len = len < max_len ? len : max_len;
        strncpy((char*)p_output, output_msg, out_len);
        *p_output_maxlen = out_len;
    }
    else
    {
        ret_val = frame_resp(frame_status, &p_input[cmd_idx], &cmd_len, p_output, p_output_maxlen);
    }
    
    return ret_val;
}

int cmd_process(uint8_t* p_payload, uint16_t* p_len)
{
    /* Lookup command */
    for(uint8_t idx=0; idx < (sizeof(cmd_table) / sizeof(cmd_table[0])); idx++)
    {
        if(cmd_table[idx].cmd_id == *((uint8_t*)p_payload))
        {
            /* Execute command */
            uint8_t cmd_id_size = FIELD_SIZEOF(cmd_t, cmd_id);
            uint16_t cmd_data_size = *p_len - cmd_id_size;
            uint8_t status = cmd_table[idx].executer(&p_payload[cmd_id_size], &cmd_data_size);
            return status;
        }
    }
    return ERR_INVALID_CMD;
}

int frame_resp(uint8_t resp_code, uint8_t* p_cmd_payload, uint16_t* p_cmd_len,
                                  uint8_t* p_frame_resp, uint16_t* p_frame_max_len)
{
    assert(p_cmd_payload != NULL);
    assert(p_cmd_len != NULL);

    uint16_t cmd_len = *p_cmd_len;
    uint16_t resp_len = cmd_len + 1 + 2; /* 1B resp_code + 2B CRC */
    uint16_t frame_len = cmd_len + 1 + 1 + 2 + 2; /* 1B resp_code + 1B PRE + 2B LEN + 2B CRC */
    if(frame_len > *p_frame_max_len)
    {
        printf("\r\n [ERR] %d (frame len) /%d (max frame len) \r\n", frame_len, *p_frame_max_len);
        return -1;
    }
    uint8_t frame_resp_buf[frame_len];
    uint8_t idx = 0;
    frame_resp_buf[idx++] = CMD_PREAMBLE_ACK;
    frame_resp_buf[idx++] = (resp_len) & 0xFF;
    frame_resp_buf[idx++] = (resp_len >> 8) & 0xFF;
    // CMD data
    for(uint8_t i = 0; i < cmd_len; i++)
    {
        frame_resp_buf[i + 3] = p_cmd_payload[i];
        idx++;
    }
    // Resp code
    frame_resp_buf[idx++] = resp_code;
    // CRC
    uint16_t sum = 0;
    for(uint8_t i = 0; i < frame_len - 2; i++)
    {
        sum += frame_resp_buf[i];
    }
    frame_resp_buf[idx++] = sum & 0xFF;
    frame_resp_buf[idx++] = (sum >> 8) & 0xFF;

    // Update frame response
    memcpy(p_frame_resp, frame_resp_buf, frame_len);
    *p_frame_max_len = frame_len;
    return 0;
}

/*
 *   
 * +------------+----------+---------------+-----------+----------------+
 * | SLAVE_ADDR | REG_ADDR | REG_ADDR_SIZE | REG_VALUE | REG_VALUE_SIZE |
 * +------------+----------+---------------+-----------+----------------+
 * | 1B         | 4B       | 1B            | 4B        | 1B             |
 * +------------+----------+---------------+-----------+----------------+
 * | [0]        | [4:1]    | [5]           | [9:6]     | [10]           | (11B)
 * +------------+----------+---------------+-----------+----------------+
 * Ex: Write to slave 0x10, reg: 0xABCD (16-bit) with value 0x1234 (32-bit)
 * CMD: 0x10 0xCD 0xAB 0x00 0x00 0x10 0x34 0x12 0x00 0x00 0x20 
 * Frame: 0x0A 0x0E 0x00 0x01 0x10 0xCD 0xAB 0x00 0x00 0x10 0x34 0x12 0x00 0x00 0x20 0xF7 0x01
 */
int cmd_spi_write_slave(uint8_t* p_payload, uint16_t* p_len)
{
    assert(p_payload != NULL);
    assert(p_len != NULL);
    if(*p_len != 11)
    {
        printf("\r\n [ERR] Invalid length \r\n");
        return ERR_INVALID_CMD;
    }
    printf("\r\n [INFO] CMD_SPI_WRITE_SLAVE \r\n");
    uint8_t slave_addr = p_payload[0];
    uint8_t reg_addr_size = p_payload[5];
    uint8_t reg_val_size = p_payload[10];
    uint32_t reg_addr = *(uint32_t*)&p_payload[1];
    uint32_t reg_val = *(uint32_t*)&p_payload[6];
    spi_test_reg_t spi_reg = 
    {
        .reg_addr = reg_addr,
        .reg_val = reg_val,
        .reg_addr_size = reg_addr_size,
        .reg_val_size = reg_val_size,
    };
    printf("\r[INFO] Slave: %d, reg_addr: 0x%08X, reg_addr_size: %db, reg_val: 0x%08X, reg_val_size: %db \r\n",
                slave_addr, (unsigned int) spi_reg.reg_addr, spi_reg.reg_addr_size, (unsigned int) spi_reg.reg_val, spi_reg.reg_val_size);
    if (0 != spi_write_slave(slave_addr, &spi_reg, 1))
    {
        printf("\r\n [ERR] CMD_SPI_WRITE_SLAVE failed \r\n");
        return ERR_CMD_EXEC_FAIL;
    }
    return ERR_OK;
}

int cmd_spi_write_slave_ack(uint8_t* p_payload, uint16_t* p_len)
{
    assert(p_payload != NULL);
    assert(p_len != NULL);
    if(*p_len != 11)
    {
        printf("\r\n [ERR] Invalid length \r\n");
        return ERR_INVALID_CMD;
    }
    printf("\r\n [INFO] CMD_SPI_WRITE_SLAVE_ACK \r\n");
    uint8_t slave_addr = p_payload[0];
    uint8_t reg_addr_size = p_payload[5];
    uint8_t reg_val_size = p_payload[10];
    uint32_t reg_addr = *(uint32_t*)&p_payload[1];
    uint32_t reg_val = *(uint32_t*)&p_payload[6];
    spi_test_reg_t spi_write_reg = 
    {
        .reg_addr = reg_addr,
        .reg_val = reg_val,
        .reg_addr_size = reg_addr_size,
        .reg_val_size = reg_val_size,
    };

    spi_test_reg_t spi_read_reg = {
        .reg_addr = reg_addr,
        .reg_addr_size = reg_addr_size,
        .reg_val_size = reg_val_size,
    };

    if (0 != spi_write_slave(slave_addr, &spi_write_reg, 1))
    {
        printf("\r\n [ERR] CMD_SPI_WRITE_SLAVE_ACK failed to write slave \r\n");
        return ERR_CMD_EXEC_FAIL;
    }

    if (0 != spi_read_slave(slave_addr, &spi_read_reg, 1))
    {
        printf("\r\n [ERR] CMD_SPI_WRITE_SLAVE_ACK failed to read slave \r\n");
        return ERR_CMD_EXEC_FAIL;
    }

    printf("\r[INFO] Slave: %d, reg_addr: 0x%08X, reg_addr_size: %db, reg_val_size: %db \r\n",
                slave_addr, (unsigned int) spi_write_reg.reg_addr, spi_write_reg.reg_addr_size, spi_write_reg.reg_val_size);
    if(spi_write_reg.reg_val != spi_read_reg.reg_val)
    {
        printf("\r\n [ERR] CMD_SPI_WRITE_SLAVE_ACK failed \r\n");
        printf("\r\n [ERR] Write: 0x%08X, Read: 0x%08X \r\n", (unsigned int) spi_write_reg.reg_val, (unsigned int) spi_read_reg.reg_val);
        return ERR_CMD_EXEC_FAIL;
    }
    return ERR_OK;
}
