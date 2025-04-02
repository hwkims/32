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
  * IMPORTANT: This code assumes you have configured the following in CubeMX
  *            and regenerated the code:
  *            1. PC0 (or your chosen pin) as GPIO_Output (for Trig).
  *            2. PE9 (or your chosen TIM1_CHx pin) as TIM1_CH1 Input Capture.
  *            3. TIM1_CH1 Input Capture mode enabled (Rising Edge initially).
  *            4. TIM1 Capture Compare interrupt enabled in NVIC.
  *            5. TIM4_CH3 PWM Output enabled (PB8).
  *            6. TIM1 Base running at 1MHz (Prescaler=83 for 84MHz clock).
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h> // Include for printf

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// extern unsigned long wTick32; // Seems unused, commenting out
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef int Bool;
#define TRUE 1
#define FALSE 0
#define DIV	60 // Servo speed control for the simple toggle logic
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- Servo Related ---
Bool pwmReady=FALSE;
uint32_t pwmCNT=0;

// --- Ultrasonic Sensor Related ---
#define TRIG_PIN GPIO_PIN_0   // Example: PC0
#define TRIG_PORT GPIOC       // Example: GPIOC Port

// --- Ultrasonic Sensor Global Variables ---
volatile uint32_t IC_Val1 = 0;
volatile uint32_t IC_Val2 = 0;
volatile uint32_t Difference = 0;
volatile uint8_t Is_First_Captured = 0;  // 0: Waiting for first edge, 1: Waiting for second edge
volatile uint8_t Capture_Done = 0;       // Flag to indicate measurement completion
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1; // Used for Delay and Input Capture
TIM_HandleTypeDef htim4; // Used for Servo PWM

UART_HandleTypeDef huart1; // Used for printf

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
// int timer_init(void); // Seems unused
// uint32_t get_timer(uint32_t base); // Seems unused
void delay_us(uint16_t us); // Declaration for delay_us
void delay_ms(uint16_t ms); // Declaration for delay_ms
void Trigger_Sonar(void);   // Declaration for Trigger_Sonar
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
 if ( ch == '\n' )
	 HAL_UART_Transmit(&huart1, (uint8_t*)"\r", 1, HAL_MAX_DELAY);
 HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
 return ch;
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
  MX_TIM1_Init();     // Initializes TIM1 for delay and Input Capture (based on CubeMX)
  MX_USART1_UART_Init();
  MX_TIM4_Init();     // Initializes TIM4 for Servo PWM (based on CubeMX)
  /* USER CODE BEGIN 2 */
  printf("\n\n\nHELLO, STM32 - Servo & Sonar Demo\n");

  // Start TIM1 base timer (needed for delay_us and Input Capture counter)
  if (HAL_TIM_Base_Start(&htim1) != HAL_OK)
  {
      Error_Handler();
  }

  // Start Servo PWM (TIM4 CH3) with Interrupt for periodic change
  if (HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_3) != HAL_OK)
  {
      Error_Handler();
  }

  printf("Initialization Complete. Starting main loop.\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // 1. Trigger the ultrasonic sensor
    Trigger_Sonar(); // This also starts the Input Capture interrupt

    // 2. Wait for the capture to complete (using the flag set by the ISR)
    //    Add a timeout mechanism to prevent getting stuck indefinitely
    uint32_t wait_start_tick = HAL_GetTick();
    while(!Capture_Done && (HAL_GetTick() - wait_start_tick < 100)) // ~100ms timeout
    {
        // You could do other non-blocking tasks here if needed
        // HAL_Delay(1); // Small delay to yield CPU slightly
    }

    // 3. Process the result if capture was successful
    if (Capture_Done) {
      // Calculate distance in cm (Difference holds the pulse width in microseconds)
      float distance_cm = (float)Difference / 58.0f;

      // Print the distance
      // Limit distance reading for practical purposes
      if (distance_cm > 0 && distance_cm < 400) { // HC-SR04 typical range ~2cm to 400cm
           printf("Distance: %.2f cm (Duration: %lu us)\n", distance_cm, Difference);
      } else if (Difference > 0) {
           printf("Distance: Out of range (Duration: %lu us)\n", Difference);
      } else {
           // This might happen if timeout occurred before capture done
           // printf("Measurement Timeout or Error\n");
      }


      // Reset the flag for the next measurement
      Capture_Done = 0;
      Difference = 0; // Reset difference too
    }
    else
    {
        // Timeout occurred before capture finished
        printf("Measurement Timeout!\n");
        // Ensure IC is stopped and polarity is reset if timeout happens during capture
        HAL_TIM_IC_Stop_IT(&htim1, TIM_CHANNEL_1);
        Is_First_Captured = 0;
        __HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

    }

    // 4. Wait a bit before the next measurement cycle
    HAL_Delay(100); // Measure roughly every ~100ms + measurement time

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
  // CubeMX should configure TIM1 Base (for delay)
  // and TIM1 Channel 1 (or other channel) for Input Capture here.
  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0}; // Added for Input Capture config display

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83; // Should result in 1MHz clock (84MHz/(83+1))
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535; // Max period for 16-bit timer
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) // Initialize Base Timer first
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK) // Initialize Input Capture part
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  // CubeMX *should* generate this Input Capture channel config
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING; // Initial polarity
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0; // No filter initially
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) // Configure Channel 1 for IC
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  // Ensure CubeMX generated the correct setup for TIM1_CH1 (PE9 or other)
  // and enabled the TIM1 Capture Compare interrupt in NVIC.
  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */
  // This should configure TIM4 CH3 (PB8) for PWM Output for the servo.
  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 830; // Should result in ~50Hz ( 42MHz / (830+1) / (2000+1) )
  htim4.Init.CounterMode = TIM_COUNTERMODE_DOWN; // Or UP, check CubeMX
  htim4.Init.Period = 2000; // Check CubeMX
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 150; // Start servo near center (Adjust 100-200 range based on servo)
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  // Ensure CubeMX generated the TIM4_CH3 pin (PB8) configuration
  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ACP_RST_GPIO_Port, ACP_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RDX_Pin|WRX_DCX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LD3_Pin|LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET); // Initialize Trig pin low

  /*Configure GPIO pins : A0_Pin A1_Pin ... */ // Assuming CubeMX generated these
  // ... LOTS OF GPIO INIT CODE GENERATED BY CUBEMX ...
  // Ensure CubeMX generated the TRIG_PIN (PC0) as Output Push Pull

  /*Configure GPIO pin : TRIG_PIN */
  GPIO_InitStruct.Pin = TRIG_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_PORT, &GPIO_InitStruct);


  // ... REST OF GPIO INIT CODE GENERATED BY CUBEMX ...
  // Ensure CubeMX generated the ECHO_PIN (PE9) as TIM1_CH1 Alternate Function

  /*Configure GPIO pins : LD3_Pin LD4_Pin */ // Example, CubeMX handles this
  GPIO_InitStruct.Pin = LD3_Pin|LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  PWM Pulse finished callback for Servo Control
  * @param  htim TIM handle
  * @retval None
  */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  // Make sure this is TIM4 interrupt
  if (htim->Instance == TIM4)
  {
    pwmCNT++;

    if (!(pwmCNT % DIV)) // Control speed of servo toggle
    {
        // printf("[Servo Callback] Toggling PWM\n"); // Optional debug
        if (pwmReady) {
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, 100); // Set pulse for angle 1 (~1ms)
            pwmReady = FALSE;
        } else {
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, 200); // Set pulse for angle 2 (~2ms)
            pwmReady = TRUE;
        }
    }
  }
}

/**
  * @brief  Input Capture callback for Ultrasonic Sensor
  * @param  htim TIM handle
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  // Make sure this is TIM1 Capture Compare interrupt
  // Use HAL_TIM_ACTIVE_CHANNEL_1 for specific channel check if needed
  if (htim->Instance == TIM1)
  {
    if (Is_First_Captured == 0) // First edge (Rising) captured
    {
      IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // Read time of rising edge
      Is_First_Captured = 1;
      // Change polarity to capture the Falling edge next
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else if (Is_First_Captured == 1) // Second edge (Falling) captured
    {
      IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // Read time of falling edge

      // Calculate pulse duration, handle timer overflow
      if (IC_Val2 > IC_Val1)
      {
        Difference = IC_Val2 - IC_Val1;
      }
      else if (IC_Val1 > IC_Val2) // Timer overflow between edges
      {
        // Timer period is 65535 (0xFFFF), counter is 0-based
        Difference = (65535 - IC_Val1) + IC_Val2 + 1;
      }
      else
      {
        // Should not happen, error condition
        Difference = 0;
      }

      Capture_Done = 1; // Set flag indicating measurement is complete
      Is_First_Captured = 0; // Reset state for next measurement

      // Change polarity back to Rising edge for the next measurement cycle
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

      // Stop Input Capture interrupt until next trigger (optional, reduces CPU load)
      HAL_TIM_IC_Stop_IT(htim, TIM_CHANNEL_1);
      // NOTE: Consider if stopping/starting IT impacts very fast measurements.
      //       Leaving it running might be okay too, just ignore callbacks until Capture_Done is reset.
    }
  }
}


/**
  * @brief  Sends a 10us pulse on the Trig pin and starts Input Capture.
  * @param  None
  * @retval None
  */
void Trigger_Sonar(void)
{
    // Send 10us Trig pulse
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
    delay_us(2); // Ensure low state before pulse
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    delay_us(10); // 10us pulse
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

    // Prepare for Echo pulse measurement
    Capture_Done = 0;       // Reset capture flag
    Is_First_Captured = 0;  // Reset capture state
    Difference = 0;         // Reset difference
    __HAL_TIM_SET_COUNTER(&htim1, 0); // Reset TIM1 counter just before starting capture

    // Start Input Capture interrupt for Echo pin (TIM1 CH1 assumed)
    if (HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1) != HAL_OK)
    {
        // Handle error, maybe print message
        printf("Error starting Input Capture!\n");
    }
}

/**
  * @brief  Provides a delay in microseconds based on TIM1.
  * @param  us: Delay time in microseconds.
  * @retval None
  */
void delay_us(uint16_t us)
{
	// Note: HAL_TIM_Base_Start(&htim1) must be called once before using this.
	__HAL_TIM_SET_COUNTER(&htim1, 0);  // Reset the counter
	while (__HAL_TIM_GET_COUNTER(&htim1) < us); // Wait until counter reaches 'us'
}

/**
  * @brief  Provides a delay in milliseconds.
  * @param  ms: Delay time in milliseconds.
  * @retval None
  */
void delay_ms(uint16_t ms)
{
	// Relies on HAL_Delay which uses SysTick by default
    HAL_Delay(ms);
    /* Or using delay_us: (Less accurate for long delays due to loop overhead)
	while(ms-- > 0)
	{
		delay_us(1000);
	}
    */
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) { // TIM6 is default for HAL_IncTick
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  printf("!!! ERROR HANDLER CALLED !!!\n");
  __disable_irq();
  while (1)
  {
      // Blink an LED rapidly or something to indicate error
      HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin); // Assuming LD4 is the RED LED
      for(uint32_t i=0; i<SystemCoreClock/100; ++i); // Crude delay
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
   printf("!!! ASSERT FAILED !!! File %s, Line %lu\r\n", file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
