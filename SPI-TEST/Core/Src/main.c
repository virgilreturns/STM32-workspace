/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal_spi.h"
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
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define GPIO_SET_PIN(port, pin) ((port)->BSRR = (pin)) //lower 16 bits sets a pin
#define GPIO_CLEAR_PIN(port, pin) ((port->BSRR = pin << 16u)) //upper 16 bits resets a pin
#define BIT_READ(reg, pos, mask) (((reg)>>(pos)) & (mask))

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

	//SLCK PA5
	GPIOA->MODER &= ~GPIO_MODER_MODER5;
	GPIOA->MODER |= 0x1<<GPIO_MODER_MODER5_Pos;//output
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR5; //floating
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT5; //push-pull
	GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED5; //low speed
	GPIOA->OSPEEDR |= 0x2<<GPIO_OSPEEDR_OSPEED5_Pos;  //high speed

	//MISO PA7
	GPIOA->MODER &= ~GPIO_MODER_MODER7;
	GPIOA->MODER |= 0x1<<GPIO_MODER_MODER7_Pos;//output
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR7; //floating
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT7; //push-pull
	GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED7; //low speed
	GPIOA->OSPEEDR |= 0x2<<GPIO_OSPEEDR_OSPEED7_Pos;  //high speed

	//SS1 PA9
	GPIOA->MODER &= ~GPIO_MODER_MODER9;
	GPIOA->MODER |= 0x1<<GPIO_MODER_MODER9_Pos;//output
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD9;
	GPIOA->PUPDR |= 0x1<<GPIO_PUPDR_PUPD9_Pos;//pull up
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT9; //push-pull
	GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED9; //low speed
	GPIOA->OSPEEDR |= 0x2<<GPIO_OSPEEDR_OSPEED9_Pos;  //high speed

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	__NVIC_EnableIRQ(EXTI4_IRQn); //CMSIS function
	__NVIC_SetPriority(EXTI4_IRQn, 0);


	//SPI1 CONFIGS
	printf("%x",SPI1->SR);
	SPI1->CR1 &= ~SPI_CR1_DFF;  //RESET FOR 8 BIT COMMUNICATION
	SPI1->CR1 &= ~SPI_CR1_CPOL; //LOW CLOCK POLARITY
	SPI1->CR1 &= ~SPI_CR1_CPHA; //FIRST DATA EDGE (RISE)
	SPI1->CR1 |= SPI_CR1_BIDIMODE;
	SPI1->CR1 |= SPI_CR1_BIDIOE;
	SPI1->CR1 |= ~SPI_CR1_BR; //assign 111 to CR1_BR Baud Rate = CLK/256

	//Interrupt CONFIGS
	//let PA15 EXT15 be external interrupt pin rising edge
	RCC->APB2ENR |= (RCC_APB2ENR_SYSCFGEN);
	SYSCFG->EXTICR[4] &= (~SYSCFG_EXTICR4_EXTI15);
	//SYSCFG->EXTICR[4] |= SYSCFG_EXTICR4_EXTI15_PA; clearing 3:0 bits will enable PA pin anyways (0x0)
	EXTI->IMR |= 1<<15;
	EXTI->RTSR |= EXTI_RTSR_TR15;
	EXTI->FTSR &= ~EXTI_FTSR_TR15;



	uint8_t *data_buf = 0x69;
	uint32_t length = 2;
	

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
  //MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  GPIOB->MODER &= ~(GPIO_MODER_MODER1); //SET PA1 TO OUTPUT ON, OR SET ALL BITS TO 00
  GPIOB->BSRR = GPIO_BSRR_BR1; //send "set pin" tick signal
  GPIOB-> MODER &= ~(GPIO_MODER_MODER1);

  GPIOB->MODER &= ~(GPIO_MODER_MODER2);
  GPIOB->MODER |= GPIO_MODER_MODER2_1;
  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD2);
  GPIOB->PUPDR |= GPIO_PUPDR_PUPD2_1;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  GPIOB->BSRR = GPIO_BSRR_BS1;
	  HAL_Delay(100);
	  GPIOB->BSRR = GPIO_BSRR_BR1;
	  HAL_Delay(100);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retrieval None
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
