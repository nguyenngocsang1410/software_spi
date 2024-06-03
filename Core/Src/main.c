/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <assert.h>

#include "../../soft_spi/soft_spi.h"
#include "../../test_spi/spi_test.h"
#include "../../uart_cmd/uart_app.h"
#include "../../uart_cmd/cmd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOG_MSG_CDC		  	  0
#define LOG_MSG_ITM 		    1
#define MAIN_DBG_MSG_EN   	1

#if (MAIN_DBG_MSG_EN != 0)
#define PRINTF_MAIN(...)  printf(__VA_ARGS__)
#else
#define PRINTF_MAIN(...)  (void)0
#endif /* End of (MAIN_DBG_MSG_EN != 0) */


/* Override low-level _write system call */
#if(LOG_MSG_CDC == 1)
#include "usbd_cdc_if.h"
int _write(int file, char *ptr, int len){
    CDC_Transmit_FS((uint8_t*)ptr, len);
    return len;
}
#elif (LOG_MSG_ITM == 1)
int _write(int file, char *ptr, int len) {
    int DataIdx;
    for (DataIdx = 0; DataIdx < len; DataIdx++) {
        ITM_SendChar(*ptr++);
    }
    return len;
}
#else
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t frame_recv_size;
uint8_t frame_received_buf[UART_RX_DMABUFFER_SIZE];
uint8_t frame_resp_buf[UART_RX_DMABUFFER_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* Delay us using TIM1 */
void delay_ns(uint32_t ns) 
{
  // Calculate the number of ticks to wait based on the input delay in ns
  uint32_t tick_2wait = (ns * (TIM1_SRC_CLK / (TIM1_PRES * 1000000))) / 1000;
  if(tick_2wait < 1)
  {
      PRINTF_MAIN("Error: delay_ns too short\n");
      tick_2wait = 1;
  }
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < tick_2wait);
}

void delay_us(uint32_t us)
{
    delay_ns(us * 1000);
}
uint32_t TIM2_GetTick_us(){
	return __HAL_TIM_GET_COUNTER(&htim2);
}
/**
 * @brief Reverse the bit pattern of a given input data
 * @param data_input: The input data to reverse
 * @param data_bit_width: actual data bit width of data_input since data_input not always 32-bit width
 * @return uint32_t: The reversed bit pattern of the input data
 */
uint32_t reverse_bit(uint32_t data_input, uint8_t data_bit_width)
{
    uint32_t data_output = 0;
    for (uint8_t i = 0; i < data_bit_width; i++)
    {
        data_output |= ((data_input >> i) & 0x1) << (data_bit_width - 1 - i);
    }
    return data_output;
}

    // Returns 0 for a 32-bit input with a bit width of 0
void test_reverse_bit_32bit_width_0() {
    // Arrange
    uint32_t input = 0xFFFFFFFF;
    uint8_t bit_width = 0;
    uint32_t expected_output = 0;

    // Act
    uint32_t result = reverse_bit(input, bit_width);

    // Assert
    assert(result == expected_output);
}

void leds_err_signal(void)
{
  HAL_GPIO_TogglePin(GPIOD, LD3_Pin| LD4_Pin | LD5_Pin | LD6_Pin);
  HAL_Delay(500);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
#if (LOG_MSG_CDC != 0)
  MX_USB_DEVICE_Init();
  while(!CDC_IsPortOpen());
#endif /* End of (LOG_MSG_CDC != 0) */
  // SPI IO init
  spi_io_init();
  // SPI timer init
  if(spi_timer_init())
  {
	  PRINTF_MAIN("[ERR] Failed to init TIM1 \n");
  }

  HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start(&htim2);

	PRINTF_MAIN("[INFO] TEST START!\n");
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // Test SPI
	SPI_SRC_INTERNAL();
  SPI_SRC_EXTERNAL();

	if (uart_app_init()) {
    PRINTF_MAIN("[ERR] Failed to start UART\n");
  }

#if (SPI_MASTER_TEST != 0)

	spi_test_reg_t spi_read_reg = {
			.reg_addr_size = 10,
			.reg_val_size = 8,
			.reg_addr = 0x3,
			.reg_val = 0x1B
	};

	uint8_t slave_addr = SPI_QEC_RX1_SLAVE_ADDR;
	spi_write_slave(slave_addr, (spi_test_reg_t*) &spi_read_reg, 1);
	HAL_Delay(1);

	if (0 != spi_read_slave(slave_addr, (spi_test_reg_t*) &spi_read_reg, 1))
	{
		printf("\r\n [ERR] spi_test_slave failed to read slave \r\n");
	}
	printf(
			"Read from slave addr: %d, reg addr: 0x%X, val: 0x%X, reg addr width: %d, reg val width: %d\n",
			slave_addr, (unsigned int) spi_read_reg.reg_addr,
			(unsigned int) spi_read_reg.reg_val,
			spi_read_reg.reg_addr_size, spi_read_reg.reg_val_size);

	spi_read_reg = (spi_test_reg_t ) {
					.reg_addr_size = 10,
					.reg_val_size = 8,
					.reg_addr = 421,
					.reg_val = 0b00010000
			};
	spi_write_slave(slave_addr, (spi_test_reg_t*) &spi_read_reg, 1);
	HAL_Delay(1);

	slave_addr = SPI_FIR_ORX2_SLAVE_ADDR;
	if (0 != spi_read_slave(slave_addr, (spi_test_reg_t*) &spi_read_reg, 1))
	{
		printf("\r\n [ERR] spi_test_slave failed to read slave \r\n");
	}
	printf(
			"Read from slave addr: %d, reg addr: 0x%X, val: 0x%X, reg addr width: %d, reg val width: %d\n",
			slave_addr, (unsigned int) spi_read_reg.reg_addr,
			(unsigned int) spi_read_reg.reg_val,
			spi_read_reg.reg_addr_size, spi_read_reg.reg_val_size);

	spi_master_test();
#endif

#if (LINH_QEC_TEST != 0)
  printf("\n================== Starting QEC Test ======================\n");
  for(uint8_t retry=0; retry<5; retry++)
  {
      printf("Current iteration: %d\n", retry);
      printf("==================================================================\n");
      printf("Writing to SPI slaves...\n");
      uint8_t test_slaves[]= {SPI_QEC_RX1_SLAVE_ADDR, SPI_QEC_RX2_SLAVE_ADDR, SPI_QEC_ORX1_SLAVE_ADDR, SPI_QEC_ORX2_SLAVE_ADDR};
      for(uint8_t idx=0; idx<sizeof(test_slaves); idx++)
      {
      if (qec_test(test_slaves[idx]))
      {
        PRINTF_MAIN("[ERR] Failed to config SPI slave\n");
      }
      }
      printf("==================================================================\n");
      HAL_Delay(1000);
  }
  printf("==================================================================\n");
#endif /* End of (LINH_QEC_TEST != 0) */

#if (HUNG_ADC_TEST != 0)
  adc_test();
#endif /* End of (HUNG_ADC_TEST != 0) */
	while (1) {
#if (SANG_TEST == 1)
		spi_master_cfg();
#endif
#if (SPI_SLAVE_TEST_EN == 1)
    while(1)
    {
      HAL_Delay(1000);
      spi_master_test();
    }
#endif /* End of (SPI_SLAVE_TEST_EN == 1) */
    if(uart_fifo_receive(frame_received_buf, &frame_recv_size) == 0)
    {
        PRINTF_MAIN("Data recv from UART FIFO: %.*s\n", frame_recv_size, frame_received_buf);
        uint16_t frame_resp_maxsize = sizeof(frame_resp_buf);
        if (0 == frame_processing(frame_received_buf, &frame_recv_size, frame_resp_buf, &frame_resp_maxsize))
        {
            if (0 != uart_app_send(frame_resp_buf, frame_resp_maxsize))
            {
                PRINTF_MAIN("[ERR] Failed to send response to UART\n");
            }
        }
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
		PRINTF_MAIN("[INFO] TEST FINISH\n");
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    leds_err_signal();
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  volatile bool g_break = false;
  PRINTF_MAIN("Wrong parameters value: file %s on line %d\r\n", file, (int)line);
  while(!g_break)
  {
    leds_err_signal();
  }
  /* User can add his own implementation to report the file name and line number,
     ex: PRINTF_MAIN("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
