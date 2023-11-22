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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

int cntSucc = 0;
int cntErr = 0;

#define BUFFER_SIZE 33
//uint8_t aTxBuffer[BUFFER_SIZE] = { 0xA5, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0xb9};

uint8_t aTxBuffer[BUFFER_SIZE] = { 0xA5, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0xfc };

const unsigned char _crc8_table[256] = { 0, 94, 188, 226, 97, 63, 221, 131, 194,
		156, 126, 32, 163, 253, 31, 65, 157, 195, 33, 127, 252, 162, 64, 30, 95,
		1, 227, 189, 62, 96, 130, 220, 35, 125, 159, 193, 66, 28, 254, 160, 225,
		191, 93, 3, 128, 222, 60, 98, 190, 224, 2, 92, 223, 129, 99, 61, 124,
		34, 192, 158, 29, 67, 161, 255, 70, 24, 250, 164, 39, 121, 155, 197,
		132, 218, 56, 102, 229, 187, 89, 7, 219, 133, 103, 57, 186, 228, 6, 88,
		25, 71, 165, 251, 120, 38, 196, 154, 101, 59, 217, 135, 4, 90, 184, 230,
		167, 249, 27, 69, 198, 152, 122, 36, 248, 166, 68, 26, 153, 199, 37,
		123, 58, 100, 134, 216, 91, 5, 231, 185, 140, 210, 48, 110, 237, 179,
		81, 15, 78, 16, 242, 172, 47, 113, 147, 205, 17, 79, 173, 243, 112, 46,
		204, 146, 211, 141, 111, 49, 178, 236, 14, 80, 175, 241, 19, 77, 206,
		144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238, 50, 108, 142, 208,
		83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115, 202, 148, 118,
		40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139, 87, 9, 235,
		181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22, 233, 183,
		85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168, 116, 42,
		200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53 };

uint8_t crc8(const uint8_t *buf, uint8_t length) {
	uint8_t crc = 0xff;
	while (length) {
		//crc = pgm_read_byte_near(_crc8_table + (*buf ^ crc));
		crc = _crc8_table[(*buf ^ crc)];
		buf++;
		length--;
	}
	return crc;
}

void calcOutgoingCrc() {
	aTxBuffer[BUFFER_SIZE - 1] = crc8(aTxBuffer, BUFFER_SIZE - 1); // calculates crc8 for outgoing packet
}

uint8_t aRxBuffer[BUFFER_SIZE];
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
enum {
	TRANSFER_WAIT, TRANSFER_COMPLETE, TRANSFER_ERROR
};
__IO uint32_t wTransferState = TRANSFER_WAIT;

uint8_t buttonStates[14] = { 0 };

void readAllButtons() {

	buttonStates[0] = !HAL_GPIO_ReadPin(IN_G1_GPIO_Port, IN_G1_Pin);
	buttonStates[1] = !HAL_GPIO_ReadPin(IN_G2_GPIO_Port, IN_G2_Pin);
	buttonStates[2] = !HAL_GPIO_ReadPin(IN_G3_GPIO_Port, IN_G3_Pin);
	buttonStates[3] = !HAL_GPIO_ReadPin(IN_G4_GPIO_Port, IN_G4_Pin);
	buttonStates[4] = !HAL_GPIO_ReadPin(IN_G5_GPIO_Port, IN_G5_Pin);
	buttonStates[5] = !HAL_GPIO_ReadPin(IN_G6_GPIO_Port, IN_G6_Pin);
	buttonStates[6] = !HAL_GPIO_ReadPin(IN_G7_GPIO_Port, IN_G7_Pin);
	buttonStates[7] = !HAL_GPIO_ReadPin(IN_G8_GPIO_Port, IN_G8_Pin);
	buttonStates[8] = !HAL_GPIO_ReadPin(IN_G9_GPIO_Port, IN_G9_Pin);
	buttonStates[9] = !HAL_GPIO_ReadPin(IN_G10_GPIO_Port, IN_G10_Pin);
	buttonStates[10] = !HAL_GPIO_ReadPin(IN_G11_GPIO_Port, IN_G11_Pin);
	buttonStates[11] = !HAL_GPIO_ReadPin(IN_G12_GPIO_Port, IN_G12_Pin);
	buttonStates[12] = !HAL_GPIO_ReadPin(IN_SHIFTER_LEFT_GPIO_Port,
	IN_SHIFTER_LEFT_Pin);
	buttonStates[13] = !HAL_GPIO_ReadPin(IN_SHIFTER_RIGHT_GPIO_Port,
	IN_SHIFTER_RIGHT_Pin);
}

void assembleButtons() {
	uint8_t buttonByte0 = (buttonStates[9] << 4)/*a*/
	| (buttonStates[10] << 5)/*RSB*/
	| (buttonStates[8] << 6)/*rt*/
	| (buttonStates[11] << 7)/*rb*/;

	uint8_t buttonByte1 = (buttonStates[12] << 3)/*shift l*/
	| (buttonStates[13] << 0)/*shift r*/
	| (buttonStates[3] << 1)/*lsb*/
	| (buttonStates[5] << 2)/*option 3 stripes*/
	| (buttonStates[1] << 4)/*lt*/
	| (buttonStates[2] << 5)/*lb*/
	| (buttonStates[0] << 6)/*2 square*/
	| (buttonStates[7] << 7)/*y*/;

	uint8_t buttonByte2 = (buttonStates[6] << 2)/*B*/
	| (buttonStates[4] << 3)/*x*/;

	aTxBuffer[2] = buttonByte0;
	aTxBuffer[3] = buttonByte1;
	aTxBuffer[4] = buttonByte2;
	calcOutgoingCrc();
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_SPI1_Init();
	/* USER CODE BEGIN 2 */
	wTransferState == TRANSFER_WAIT;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		calcOutgoingCrc();
		wTransferState == TRANSFER_WAIT;
		if (HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*) aTxBuffer,
				(uint8_t*) aRxBuffer, BUFFER_SIZE) != HAL_OK) {
			/* Transfer error in transmission process */
			//Error_Handler();
		}
		while (wTransferState == TRANSFER_WAIT) {
		}
		readAllButtons();
		assembleButtons();

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_SLAVE;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA1_Channel2_3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : IN_G10_Pin IN_G6_Pin IN_G7_Pin IN_G12_Pin
	 IN_G11_Pin IN_G1_Pin IN_G2_Pin IN_G8_Pin */
	GPIO_InitStruct.Pin = IN_G10_Pin | IN_G6_Pin | IN_G7_Pin | IN_G12_Pin
			| IN_G11_Pin | IN_G1_Pin | IN_G2_Pin | IN_G8_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : IN_SHIFTER_LEFT_Pin IN_G9_Pin IN_G5_Pin IN_G4_Pin
	 IN_G3_Pin IN_SHIFTER_RIGHT_Pin */
	GPIO_InitStruct.Pin = IN_SHIFTER_LEFT_Pin | IN_G9_Pin | IN_G5_Pin
			| IN_G4_Pin | IN_G3_Pin | IN_SHIFTER_RIGHT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : LD3_Pin */
	GPIO_InitStruct.Pin = LD3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief  TxRx Transfer completed callback.
 * @param  hspi: SPI handle
 * @note   This example shows a simple way to report end of DMA TxRx transfer, and
 *         you can add your own implementation.
 * @retval None
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	/* Turn LED3 on: Transfer in transmission/reception process is complete */
	//BSP_LED_On(LED3);
	wTransferState = TRANSFER_COMPLETE;
	cntSucc++;
}

/**
 * @brief  SPI error callbacks.
 * @param  hspi: SPI handle
 * @note   This example shows a simple way to report transfer error, and you can
 *         add your own implementation.
 * @retval None
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	wTransferState = TRANSFER_ERROR;
	cntErr++;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
