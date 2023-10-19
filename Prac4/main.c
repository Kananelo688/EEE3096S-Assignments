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
#include "stm32f0xx.h"
#include <lcd_stm32f0.c>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// TODO: Add values for below variables
#define NS 201          // Number of samples in LUT
#define TIM2CLK 8000000 // STM Clock frequency
#define F_SIGNAL 100    // Frequency of output analog signal

/*Wave form flags*/
#define SINE_FLAG 0
#define TRI_FLAG 1
#define SAW_FLAG 2
#define NUM_FLAGS 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */
// TODO: Add code for global variables, including LUTs

uint32_t Sin_LUT[NS] = {
    511,  527,  543,  559,  575,  591,  607,  623,  638,  654,  669,  684,
    699,  714,  729,  743,  757,  771,  785,  799,  812,  825,  837,  849,
    861,  873,  884,  895,  905,  915,  925,  934,  943,  951,  959,  967,
    974,  980,  987,  992,  997,  1002, 1006, 1010, 1013, 1016, 1018, 1020,
    1021, 1022, 1023, 1022, 1021, 1020, 1018, 1016, 1013, 1010, 1006, 1002,
    997,  992,  987,  980,  974,  967,  959,  951,  943,  934,  925,  915,
    905,  895,  884,  873,  861,  849,  837,  825,  812,  799,  785,  771,
    757,  743,  729,  714,  699,  684,  669,  654,  638,  623,  607,  591,
    575,  559,  543,  527,  511,  495,  479,  463,  447,  431,  415,  399,
    384,  368,  353,  338,  323,  308,  293,  279,  265,  251,  237,  223,
    210,  197,  185,  173,  161,  149,  138,  127,  117,  107,  97,   88,
    79,   71,   63,   55,   48,   42,   35,   30,   25,   20,   16,   12,
    9,    6,    4,    2,    1,    0,    0,    0,    1,    2,    4,    6,
    9,    12,   16,   20,   25,   30,   35,   42,   48,   55,   63,   71,
    79,   88,   97,   107,  117,  127,  138,  149,  161,  173,  185,  197,
    210,  223,  237,  251,  265,  279,  293,  308,  323,  338,  353,  368,
    384,  399,  415,  431,  447,  463,  479,  495,  511

};

uint32_t saw_LUT[NS] = {
    0,    5,    10,   15,   20,  25,  30,  35,  40,  46,  51,  56,  61,  66,
    71,   76,   81,   86,   92,  97,  102, 107, 112, 117, 122, 127, 132, 138,
    143,  148,  153,  158,  163, 168, 173, 179, 184, 189, 194, 199, 204, 209,
    214,  219,  225,  230,  235, 240, 245, 250, 255, 260, 265, 271, 276, 281,
    286,  291,  296,  301,  306, 312, 317, 322, 327, 332, 337, 342, 347, 352,
    358,  363,  368,  373,  378, 383, 388, 393, 398, 404, 409, 414, 419, 424,
    429,  434,  439,  445,  450, 455, 460, 465, 470, 475, 480, 485, 491, 496,
    501,  506,  511,  516,  521, 526, 531, 537, 542, 547, 552, 557, 562, 567,
    572,  577,  583,  588,  593, 598, 603, 608, 613, 618, 624, 629, 634, 639,
    644,  649,  654,  659,  664, 670, 675, 680, 685, 690, 695, 700, 705, 710,
    716,  721,  726,  731,  736, 741, 746, 751, 757, 762, 767, 772, 777, 782,
    787,  792,  797,  803,  808, 813, 818, 823, 828, 833, 838, 843, 849, 854,
    859,  864,  869,  874,  879, 884, 890, 895, 900, 905, 910, 915, 920, 925,
    930,  936,  941,  946,  951, 956, 961, 966, 971, 976, 982, 987, 992, 997,
    1002, 1007, 1012, 1017, 0};

uint32_t triangle_LUT[NS] = {
    0,    10,   20,   30,   40,   51,  61,  71,  81,  92,  102, 112, 122, 132,
    143,  153,  163,  173,  184,  194, 204, 214, 225, 235, 245, 255, 265, 276,
    286,  296,  306,  317,  327,  337, 347, 358, 368, 378, 388, 398, 409, 419,
    429,  439,  450,  460,  470,  480, 491, 501, 511, 521, 531, 542, 552, 562,
    572,  583,  593,  603,  613,  624, 634, 644, 654, 664, 675, 685, 695, 705,
    716,  726,  736,  746,  757,  767, 777, 787, 797, 808, 818, 828, 838, 849,
    859,  869,  879,  890,  900,  910, 920, 930, 941, 951, 961, 971, 982, 992,
    1002, 1012, 1023, 1012, 1002, 992, 982, 971, 961, 951, 941, 930, 920, 910,
    900,  890,  879,  869,  859,  849, 838, 828, 818, 808, 797, 787, 777, 767,
    757,  746,  736,  726,  716,  705, 695, 685, 675, 664, 654, 644, 634, 624,
    613,  603,  593,  583,  572,  562, 552, 542, 531, 521, 511, 501, 491, 480,
    470,  460,  450,  439,  429,  419, 409, 398, 388, 378, 368, 358, 347, 337,
    327,  317,  306,  296,  286,  276, 265, 255, 245, 235, 225, 214, 204, 194,
    184,  173,  163,  153,  143,  132, 122, 112, 102, 92,  81,  71,  61,  51,
    40,   30,   20,   10,   0};
// TODO: Equation to calculate TIM2_Ticks
/*TIM2_TIcks = ceil(TIMCLK/NS*FSIGNAL)-1 = ceil(8MHZ/(500*401))-1=39*/
uint32_t TIM2_Ticks =
    (TIM2CLK / (NS * F_SIGNAL)); // How often to write new LUT value
uint32_t DestAddress =
    (uint32_t) &
    (TIM3->CCR3); // Write LUT TO TIM3->CCR3 to modify PWM duty cycle

uint8_t flag = 0; /*start with sine wave*/

uint32_t curr_ticks, prev_ticks;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
void EXTI0_1_IRQHandler(void);
static void update_flag(void);
/* USER CODE END PFP */

/* Private user code
   ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */
  init_LCD();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  // TODO: Start TIM3 in PWM mode on channel 3
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  // TODO: Start TIM2 in Output Compare (OC) mode on channel 1.
  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);
  // TODO: Start DMA in IT mode on TIM2->CH1; Source is LUT and Dest is
  // TIM3->CCR3; start with Sine LUT
  HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)Sin_LUT, DestAddress, NS);

  // TODO: Write current waveform to LCD ("Sine")
  lcd_putstring("Sine");
  delay(3000);

  // TODO: Enable DMA (start transfer from LUT to CCR)
  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0) {
  }
  LL_RCC_HSI_Enable();

  /* Wait till HSI is ready */
  while (LL_RCC_HSI_IsReady() != 1) {
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

  /* Wait till System clock is ready */
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI) {
  }
  LL_SetSystemCoreClock(8000000);

  /* Update the time base */
  if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TIM2_Ticks - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1023;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);

  /**/
  LL_GPIO_SetPinPull(Button0_GPIO_Port, Button0_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinMode(Button0_GPIO_Port, Button0_Pin, LL_GPIO_MODE_INPUT);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void EXTI0_1_IRQHandler(void) {
  // TODO: Debounce using HAL_GetTick()
  curr_ticks = HAL_GetTick(); /*get current number of ticks*/

  if ((curr_ticks - prev_ticks) > 50) {
    update_flag();
    lcd_command(CLEAR);
    // TODO: Disable DMA transfer and abort IT, then start DMA in IT mode with
    // new
    // LUT and re-enable transfer HINT: Consider using C's "switch" function to
    // handle LUT changes
    __HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_CC1);
    HAL_DMA_Abort_IT(&hdma_tim2_ch1);

    switch (flag) {
    case SINE_FLAG:
      HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)Sin_LUT, DestAddress, NS);
      lcd_putstring("Sine");
      break;
    case TRI_FLAG:
      HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)triangle_LUT, DestAddress, NS);
      lcd_putstring("Triangle");
      break;
    case SAW_FLAG:
      HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)saw_LUT, DestAddress, NS);
      lcd_putstring("Sawtooth");
      break;

    default:
      HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)Sin_LUT, DestAddress, NS);
      lcd_putstring("Sine");
      break;
    }
  }
  prev_ticks = curr_ticks;
  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);
  HAL_GPIO_EXTI_IRQHandler(Button0_Pin); // Clear interrupt flags
}

static void update_flag() {
  flag++;
  flag %= NUM_FLAGS;
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

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
