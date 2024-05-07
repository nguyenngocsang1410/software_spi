/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v1.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#define USB_CDC_DGB_MSG_EN 1
#if !(USB_CDC_DGB_MSG_EN == 0)
#include <stdio.h>
#define USB_CDC_PRINTF(...) printf(__VA_ARGS__)
#else
#define USB_CDC_PRINTF(...) 
#endif


/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static bool is_port_open = false;
/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
/**
 * @brief: Parse Get/Set Line Coding Request
 * @param pbuf: Request msg
 * @param length: Length of request msg (7B)
 * @return int: 0 if success, -1 if fail
 */
static int CDC_Parse_LineCoding(uint8_t *pbuf, uint16_t length)
{
  if(pbuf == NULL)
  {
    return -1;
  }
  USBD_CDC_LineCodingTypeDef line_coding = {0};
  if (length < 7)
  {
    return -1;
  }
  line_coding.bitrate = (uint32_t)(pbuf[0] | (pbuf[1] << 8) | (pbuf[2] << 16) | (pbuf[3] << 24));
  line_coding.format = pbuf[4];
  line_coding.paritytype = pbuf[5];
  line_coding.datatype = pbuf[6];
  USB_CDC_PRINTF("Line Coding: %ld %d %d %d\n", (long)line_coding.bitrate, line_coding.format, line_coding.paritytype, line_coding.datatype);
  return 0;
}

/**
 * @brief: Parse Control Line Request
 * @param: pbuf Request msg
 * @param length: Length of request msg
 * @return int: 0 if success, -1 if fail
 * @note: ST USB lib has a bug in parsing Control Line Request, the length field is not correct (0)
 */
static int CDC_Parse_ControlLine(uint8_t *pbuf, uint16_t length)
{
  if(pbuf == NULL)
  {
    return -1;
  }
  USB_CDC_PRINTF("Control Line: %s \n", pbuf[2] & 1? "DTR PRESENT" : "DTR NOT PRESENT");
  is_port_open = pbuf[2] & 1;
  return 0;
}

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
USB_CDC_PRINTF("\n=================== CDC_Control_FS ===================\n");
  USB_CDC_PRINTF("EVT: %d\n", cmd);
  USB_CDC_PRINTF("LEN: %d, Data: ", length);
  // Dump data 
  for (int i = 0; i < length; i++)
  {
    USB_CDC_PRINTF("0x%02X ", pbuf[i]);
  }
  USB_CDC_PRINTF("\n");

  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:
USB_CDC_PRINTF("CDC_SEND_ENCAPSULATED_COMMAND\n\n");
    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:
USB_CDC_PRINTF("CDC_GET_ENCAPSULATED_RESPONSE\n\n");
    break;

    case CDC_SET_COMM_FEATURE:
USB_CDC_PRINTF("CDC_SET_COMM_FEATURE\n\n");
    break;

    case CDC_GET_COMM_FEATURE:
USB_CDC_PRINTF("CDC_GET_COMM_FEATURE\n\n");
    break;

    case CDC_CLEAR_COMM_FEATURE:
USB_CDC_PRINTF("CDC_CLEAR_COMM_FEATURE\n\n");
    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:
CDC_Parse_LineCoding(pbuf, length);
		  USB_CDC_PRINTF("CDC_SET_LINE_CODING\n\n");
    break;

    case CDC_GET_LINE_CODING:
CDC_Parse_LineCoding(pbuf, length);
		  USB_CDC_PRINTF("CDC_GET_LINE_CODING\n\n");
    break;
/*******************************************************************************/
  /* CDC_SET_CONTROL_LINE_STATE                                                  */
  /* Note: control signal value contain in the "wValue" of the request (B2)      */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | DTR         |   1  | 0 or 1 | Data Terminal Ready                  */
  /* 1      | RTS         |   1  | 0 or 1 | Request to Send                      */
  /* 15:2   | RESERVED    |  14  |  NONE  | Reset to zero                        */
  /*******************************************************************************/
    case CDC_SET_CONTROL_LINE_STATE:
CDC_Parse_ControlLine(pbuf, length);
    	USB_CDC_PRINTF("CDC_SET_CONTROL_LINE_STATE\n\n");
    break;

    case CDC_SEND_BREAK:
USB_CDC_PRINTF("CDC_SEND_BREAK\n\n");
    break;

  default:
    break;
  }
USB_CDC_PRINTF("=====================================================\n\n");
  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
// Loopback incoming data
  CDC_Transmit_FS(Buf, *Len);
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/**
  * @brief  CDC_TransmitCplt_FS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 13 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  /* USER CODE END 13 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
bool CDC_IsPortOpen(void)
{
  return is_port_open;
}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
