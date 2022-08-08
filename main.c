/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
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
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
int16_t Pixy_Result[8];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void getVersion(void);
void getResolution(void);
uint32_t setCameraBrightness(uint8_t);
void getBlocks(void);
//void print(void);
uint8_t getDirection (void);
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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  getVersion();
  getResolution();

  uint8_t trans[4];
  trans[0] = $7A;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  getBlocks();
	  trans[1] = getDirection();
	  trans[2] = 20;
	  trans[3] = (trans[1] ^ trans[2]) ^ $7F;

	  HAL_UART_Transmit(&huart1, (uint8_t*) trans, 4, 100);

	  HAL_Delay(1000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 19200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void getVersion(void) {
	uint8_t	tx_pixy2[4] = {0xae, 0xc1, 0x0e, 0x00};
	uint8_t	rx_pixy2[22];

	HAL_UART_Transmit(&huart3, (uint8_t *) tx_pixy2, 4, 100);
	HAL_UART_Receive(&huart3, rx_pixy2, sizeof(rx_pixy2), 100);

	uint16_t Ver 	  = (rx_pixy2[6] | (rx_pixy2[7] << 8));
	uint16_t Checksum = (rx_pixy2[4] | (rx_pixy2[5] << 8));
	uint16_t CKC 	  = 0;

	for (int i = 6; i < 22; i++) {
		CKC += rx_pixy2[i];
	}
}

void getResolution() {
	uint8_t tx_pixy2[] = {0xae, 0xc1, 0x0c, 0x01, 0x00};
	uint8_t rx_pixy2[11];

	HAL_UART_Transmit(&huart3, (uint8_t *) tx_pixy2, 5, 100);
	HAL_UART_Receive(&huart3, (uint8_t *) rx_pixy2, sizeof(rx_pixy2), 100);

	uint16_t width    = (rx_pixy2[6] | (rx_pixy2[7] << 8));
	uint16_t height   = (rx_pixy2[8] | (rx_pixy2[9] << 8));
	uint16_t Checksum = (rx_pixy2[4] | (rx_pixy2[5] << 8));
	uint16_t CKC 	  = 0;

	for (int i = 6; i < 10; i++) {
		CKC += rx_pixy2[i];
	}
}

uint32_t setCameraBrightness(uint8_t brightness) {
	uint8_t tx_pixy2[] = {0xae, 0xc1, 0x10, 0x01, brightness};
	uint8_t rx_pixy2[10];

	HAL_UART_Transmit(&huart3, (uint8_t *) tx_pixy2, 5, 100);
	HAL_UART_Receive(&huart3, (uint8_t *) rx_pixy2, sizeof(rx_pixy2), 100);

	uint32_t Result   = (rx_pixy2[6] | (rx_pixy2[7] << 8) | (rx_pixy2[8] << 16) | (rx_pixy2[9] << 24));

	return Result;
}

void getBlocks(void) {
	uint8_t tx_pixy2[] = {0xae, 0xc1, 0x20, 0x02, 0x01, 0x01};
	uint8_t rx_pixy2[20];

	HAL_UART_Transmit(&huart3, (uint8_t *) tx_pixy2, 6, 100);
	HAL_UART_Receive(&huart3, (int8_t *) rx_pixy2, sizeof(rx_pixy2), 100);

	uint16_t Checksum  = (rx_pixy2[4] | (rx_pixy2[5] << 8));
	uint16_t CKC	   = 0;

	for (int i = 6; i < 20; i++) {
			CKC += rx_pixy2[i];
	}

	Pixy_Result[0] = (rx_pixy2[6] | (rx_pixy2[7] << 8));
	Pixy_Result[1] = (rx_pixy2[8] | (rx_pixy2[9] << 8));
	Pixy_Result[2] = (rx_pixy2[10] | (rx_pixy2[11] << 8));
	Pixy_Result[3] = (rx_pixy2[12] | (rx_pixy2[13] << 8));
	Pixy_Result[4] = (rx_pixy2[14] | (rx_pixy2[15] << 8));
	Pixy_Result[5] = (rx_pixy2[16] | (rx_pixy2[17] << 8));
	Pixy_Result[6] = rx_pixy2[18];
	Pixy_Result[7] = rx_pixy2[19];
}

/*void print(void) {
	char str[255];

	int str_trans = sprintf(str, "Colour: %d\rX-Center: %d\rY-Center: %d\rWidth: %d\rHeight: %d\rAngle: %d\rIndex: %d\rAge: %d\r Dir: %d\n\r",
						Pixy_Result[0],Pixy_Result[1],Pixy_Result[2],Pixy_Result[3],Pixy_Result[4],Pixy_Result[5],Pixy_Result[6],Pixy_Result[7], Dir);

	HAL_UART_Transmit(&huart2, str, str_trans, 100);

	if (Dir == 1) {
		str_trans = sprintf(str, "Direction: Forward\r\r");
	} else if (Dir == 2) {
		str_trans = sprintf(str, "Direction: Reverse\r\r");
	} else if (Dir == 3) {
		str_trans = sprintf(str, "Direction: Strafe Right\r\r");
	} else if (Dir == 4) {
		str_trans = sprintf(str, "Direction: Strafe Left\r\r");
	} else if (Dir == 5) {
		str_trans = sprintf(str, "Direction: Right Forward Diagonal\r\r");
	} else if (Dir == 6) {
		str_trans = sprintf(str, "Direction: Left Forward Diagonal\r\r");
	} else if (Dir == 7) {
		str_trans = sprintf(str, "Direction: Right Diagonal Reverse\r\r");
	} else if (Dir == 8) {
		str_trans = sprintf(str, "Direction: Left Diagonal Reverse\r\r");
	} else if (Dir == 11) {
		str_trans = sprintf(str, "Direction: Stop\r\r");
	}

	HAL_UART_Transmit(&huart1, (uint8_t*) trans, 4, 100);
} */

uint8_t getDirection (void) {
	uint8_t Dir;

	if (Pixy_Result[0] != 1) {
		Dir = 11;
	} else {
		if ((Pixy_Result[3] * Pixy_Result[4]) < 6400 && Pixy_Result[1] > 200) {
			Dir = 5;
		} else if ((Pixy_Result[3] * Pixy_Result[4]) < 6400 && Pixy_Result[1] < 100) {
			Dir = 6;
		} else if ((Pixy_Result[3] * Pixy_Result[4]) > 10000 && Pixy_Result[1] > 200) {
			Dir = 7;
		} else if ((Pixy_Result[3] * Pixy_Result[4]) > 10000 && Pixy_Result[1] < 100) {
			Dir = 8;
		} else if ((Pixy_Result[3] * Pixy_Result[4]) < 6400) {
			Dir = 1;
		} else if ((Pixy_Result[3] * Pixy_Result[4]) > 10000) {
			Dir = 2;
		} else if (Pixy_Result[1] > 200) {
			Dir = 3;
		} else if (Pixy_Result[1] < 100) {
			Dir = 4;
		}
	}

	return Dir;
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

