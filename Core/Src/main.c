/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//89Byte:
//#define TFMINI_DATA_Len             64
//#define TFMINT_DATA_HEAD            0x59
//
//#define TFMINI_ACTION_DIST          200
//
//extern DMA_HandleTypeDef hdma_usart1_rx;
//extern DMA_HandleTypeDef hdma_usart2_rx;
//
//uint8_t g_usart1_rx_buf[USART_BUF_SIZE] = {0};    /*usart1_rx_buf*/
//uint8_t g_usart2_rx_buf[USART_BUF_SIZE] = {0};    /*usart2_rx_buf*/


//PIX:
#define TFMINI_PIX_FLAG_END         0x0D0A     /*\r\n*/
#define TFMINI_PIX_FLAG_NEG         0x2D       /* - */
#define TFMINI_PIX_FLAG_DECPIONT    0x2E       /* . */

#define ASCII_0                     0x30       /*ASCII:0*/

#define TFMINI_ACTION_DIST          200

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;

uint8_t g_usart1_rx_buf[USART_BUF_SIZE] = {0};    /*usart1_rx_buf*/
uint8_t g_usart2_rx_buf[USART_BUF_SIZE] = {0};    /*usart2_rx_buf*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART1_UART_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_DMA(&huart1, g_usart1_rx_buf, USART_BUF_SIZE);
  HAL_UART_Receive_DMA(&huart2, g_usart2_rx_buf, USART_BUF_SIZE);

  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);    //UART_IT_IDLE
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);    //UART_IT_IDLE

  printf("TFMini\r\n");
  printf("USART_2_DOUT\r\n");
  printf("Argement: 89Byte\r\n");
  printf("dist > %d cm, PA8 set Low;\r\ndist <= %d cm, PA8 set High.\r\n", TFMINI_ACTION_DIST, TFMINI_ACTION_DIST);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint16_t distance = 0;
void USART1_RX_Proc(uint8_t *buf, uint32_t len)
{
	uint16_t cordist = 0;

	/*xxx.xx\r\n*/
	if((TFMINI_PIX_FLAG_END == (buf[len - 1] | (buf[len - 2] << 8))) \
		&& (TFMINI_PIX_FLAG_DECPIONT == buf[len - 5]))
	{

		if(buf[0] == TFMINI_PIX_FLAG_NEG)   /*Negative, the amplitude value too low.*/
		{
			cordist = 1200;
		}
		else
		{
			cordist = ((buf[len - 6] - ASCII_0) * 100) + ((buf[len - 4] - ASCII_0) * 10) + (buf[len - 3] - ASCII_0);
			cordist+= (len == 7) ? ((buf[len - 7] - ASCII_0) * 1000) : 0;
		}

		/*cordist > TFMINI_ACTION_DIST cm, PA8 set Low;
		  cordist <= TFMINI_ACTION_DIST cm, PA8 set High.*/
		if(HAL_GPIO_ReadPin(GPIOA, RX1_Pin) != GPIO_PIN_RESET)
		{
			if(cordist > TFMINI_ACTION_DIST)
			{
				HAL_GPIO_WritePin(GPIOA, RX1_Pin, GPIO_PIN_RESET);
			}
		}
		else
		{
			if(cordist <= TFMINI_ACTION_DIST)
			{
				HAL_GPIO_WritePin(GPIOA, RX1_Pin, GPIO_PIN_SET);
			}
		}
	}
}


void USART2_RX_Proc(uint8_t *buf, uint32_t len)
{

}
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
