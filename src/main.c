#include "stm32g0xx_hal.h"
#include "stdbool.h"
#include <math.h>
#include <stdarg.h>  

/* Private variables */
#define BUFFER_SIZE 100
uint16_t adc_buf[100];    // Buffer to store ADC values
UART_HandleTypeDef huart2; 
ADC_HandleTypeDef hadc1;
DAC_HandleTypeDef hdac1;
TIM_HandleTypeDef htim3;  // Add handle for Time
DMA_HandleTypeDef hdma_adc1; // Add handle for DMA

#define DAC_RESOLUTION 4095
#define SAMPLES 100

#define INTERVAL_LED_BLINK      100 // Interval for LED blinking in milliseconds

#define LED_PIN                                GPIO_PIN_13
#define LED_GPIO_PORT                          GPIOC
#define LED_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOC_CLK_ENABLE()


#define LED_F0                                GPIO_PIN_0
#define LEDF0_GPIO_PORT                       GPIOF
#define LEDF0_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOF_CLK_ENABLE()



// void LED_Init(); 
void MX_ADC1_Init(void);
void MX_USART2_UART_Init(void);
void Error_Handler(void); 
void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle);
void MX_GPIO_Init(void); 
void MX_DAC1_Init(void);
void MX_TIM3_Init(void);  // Add timer initialization function declaration
void MX_DMA_Init(void);
void GenerateSineWave(void);
void uart_print(const char *fmt, ...);

volatile uint16_t ledCounter = 0;
volatile uint32_t adcValue = 0;
bool isConversionComplete = false;
uint16_t sineWave[SAMPLES];

long currentTicks = 0; // Variable to store current time in microseconds

int main(void)
{
    char msg[50];
    int count = 0;
    /* System initializations */
    HAL_Init(); 

    // Configure the system clock 
    SystemClock_Config();
 
    
    // Initialize GPIO (LED)
    MX_GPIO_Init();
    //MX_DMA_Init(); //<-- Add this to initialize DMA before ADC
    
    // Initialize ADC
    MX_ADC1_Init();  

    /* Initialize peripherals */ 
    MX_USART2_UART_Init(); 
    MX_DAC1_Init();

    // Initialize and start Timer3 (1ms interrupt)
    MX_TIM3_Init();

    sprintf(msg, "\nSys Init..");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100); 

    HAL_Delay(10); // Delay for 10ms
 
    // Calibrate ADC before use
    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
    {
      Error_Handler();
    }
        
   
    // Start DAC channel 1 (PA4)
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
    
    // Start ADC in continuous mode with interrupt enabled
    HAL_ADC_Start_IT(&hadc1);

    // Start timer with interrupt
    HAL_TIM_Base_Start_IT(&htim3);

  //  // Start ADC with DMA
  //   if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, BUFFER_SIZE) != HAL_OK) {
  //     Error_Handler();
  //   }

    sprintf(msg, ".Ready\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100); 

    // Main loop
    GenerateSineWave();
    
    while (1)
    {
        //HAL_Delay(1);  // Small delay to prevent flooding serial output 

        if(HAL_GetTick() - currentTicks >= INTERVAL_LED_BLINK) // Check if 1 ms has passed
        {
            currentTicks = HAL_GetTick(); // Update current time            
            HAL_GPIO_TogglePin(LEDF0_GPIO_PORT, LED_F0); // Toggle LED state
        }

         
        // Variable to store ADC value
        uint32_t adc_value = 0; 

        if(isConversionComplete)
        {
            static uint8_t index = 0;
            isConversionComplete = false; 
            
            // Write ADC value to DAC (12-bit value)
            adc_value = adcValue;
            uint32_t frequency = 100 + (adc_value * 10); // Adjust frequency dynamically
            __HAL_TIM_SET_AUTORELOAD(&htim3, HAL_RCC_GetHCLKFreq() / (SAMPLES * frequency));
               

            // Optionally, print some debug info every few cycles
            if(++count >= 10000) {
              uart_print("ADC: %lu\r\n", (unsigned long)adc_value); 
              count = 0;
          }  
             
        }
    }
}

 
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE(); // Enable clock for GPIOF 

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LEDF0_GPIO_PORT, LED_F0, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);


   /*Configure GPIO pin : LED_Pin F0 */
  GPIO_InitStruct.Pin = LED_F0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LEDF0_GPIO_PORT, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

void MX_ADC1_Init(void)
{ 
   /* USER CODE BEGIN ADC1_Init 0 */
  
   __HAL_RCC_ADC_CLK_ENABLE();      //! <-- Must for ADC to work
  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
// Reset ADC peripheral
__HAL_RCC_ADC_FORCE_RESET();
__HAL_RCC_ADC_RELEASE_RESET();
  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode =  ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_160CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  } 

  /** Configure first Regular Channel (PB1 - ADC_CHANNEL_9)
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  //sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
   // Configure channels
  sConfig.SamplingTime = ADC_SAMPLETIME_160CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
 
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

// Global variable for tracking current ADC channel being read
volatile uint8_t currentAdcChannel = 0;

// ADC DMA Transfer Complete callback
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance == ADC1)
  {
    // DMA has completed transferring all data
    // Set a flag to indicate data is ready for processing
    adcValue = HAL_ADC_GetValue(hadc);
    isConversionComplete = true;
  }
}

// ADC DMA Half-Transfer Complete callback
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance == ADC1)
  {
    // Half of the buffer has been filled
    // You can process this half while the other half is being filled
    // This enables continuous processing
  }
}

// Add this to handle ADC interrupts
void ADC1_COMP_IRQHandler(void)
{
  HAL_ADC_IRQHandler(&hadc1);
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(adcHandle->Instance == ADC1)
    {
       /* Peripheral clock enable */
    __HAL_RCC_ADC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

      /**ADC1 GPIO Configuration
      PB1     ------> ADC1_IN9 //! Agafia pin A5
      */
      GPIO_InitStruct.Pin = GPIO_PIN_1;
      GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); 

    /* ADC1 interrupt Init */
    HAL_NVIC_SetPriority(ADC1_COMP_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC1_COMP_IRQn);
    /* USER CODE BEGIN ADC1_MspInit 1 */

    /* USER CODE END ADC1_MspInit 1 */
    }
}

// DMA Initialization (typically generated by STM32CubeMX)
void MX_DMA_Init(void) {
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

      /* ADC1 DMA Init */
      /* ADC1 Init */
      hdma_adc1.Instance = DMA1_Channel1;
      hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
      hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
      hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
      hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
      hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
      hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
      hdma_adc1.Init.Mode = DMA_CIRCULAR;
      hdma_adc1.Init.Priority = DMA_PRIORITY_VERY_HIGH;

      if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
      {
        Error_Handler();
      }

    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

    /* DMA interrupt init */
    /* DMA1_Channel1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    /* DMA1_Ch4_7_DMAMUX1_OVR_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Ch4_7_DMAMUX1_OVR_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Ch4_7_DMAMUX1_OVR_IRQn);
}

// DMA Interrupt handler
void DMA1_Channel1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_adc1);
}
 


/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
 void MX_DAC1_Init(void)
 {
 
   /* USER CODE BEGIN DAC1_Init 0 */
 
   /* USER CODE END DAC1_Init 0 */
 
   DAC_ChannelConfTypeDef sConfig = {0};
 
   /* USER CODE BEGIN DAC1_Init 1 */
 
   /* USER CODE END DAC1_Init 1 */
 
   /** DAC Initialization
   */
   hdac1.Instance = DAC1;
   if (HAL_DAC_Init(&hdac1) != HAL_OK)
   {
     Error_Handler();
   }
 
   /** DAC channel OUT1 config
   */
   sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
   sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
   sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
   sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
   sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
   if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
   {
     Error_Handler();
   }
   /* USER CODE BEGIN DAC1_Init 2 */
 
   /* USER CODE END DAC1_Init 2 */
 
 }

 void HAL_DAC_MspInit(DAC_HandleTypeDef* hdac)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hdac->Instance==DAC1)
  {
    /* DAC1 clock enable */
    __HAL_RCC_DAC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    /* DAC1 GPIO Configuration: PA4 -> DAC1_OUT1 */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

void MX_USART2_UART_Init(void){

  GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Enable clocks for GPIOA and USART2
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    // Configure UART2 on pins PA2 and PA3
    // Configure PA2 (TX) and PA3 (RX)
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
   
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */ 
}

/**
  * @brief TIM3 Initialization Function - 1ms timer
  * @param None
  * @retval None
  */
 void MX_TIM3_Init(void)
 {
   TIM_ClockConfigTypeDef sClockSourceConfig = {0};
   TIM_MasterConfigTypeDef sMasterConfig = {0};
 
   /* Timer 3 base initialization */
   htim3.Instance = TIM3;
   
   /* Calculate for 1ms:
    * Timer clock = SystemCoreClock / 1
    * Period = (Timer clock / 1000) - 1
    */
   htim3.Init.Prescaler = (SystemCoreClock / 1000000) - 1; // Prescaler for 1MHz 
   htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
   htim3.Init.Period = 1000 - 1; // For 1ms interrupt (1000 * 1µs = 1ms)
   htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
   htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
   
   if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
   {
     Error_Handler();
   }
   
   sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
   if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
   {
     Error_Handler();
   }
   
   sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
   sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
   if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
   {
     Error_Handler();
   }
 }

 /**
  * @brief  Timer period elapsed callback
  * @param  htim Timer handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    // 1ms timer callback - place code to execute every 1ms here
    // For example, you could toggle an LED or set a flag

    if(ledCounter++ >= 500)
    {
      HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
      ledCounter = 0;
    }
    
    static uint8_t index = 0; 
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, sineWave[index]);
    index = (index + 1) % SAMPLES;
    
    // // Start a new ADC conversion if needed
    // if (!isConversionComplete)
    // {
    //   HAL_ADC_Start_IT(&hadc1);
    // }
  }
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim3);
}
 
 /**
   * @brief TIM MSP Initialization
   * @param htim: TIM handle pointer
   * @retval None
   */
 void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim)
 {
   if(htim->Instance == TIM3)
   {
     /* TIM3 clock enable */
     __HAL_RCC_TIM3_CLK_ENABLE();
 
     /* TIM3 interrupt Init */
     HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
     HAL_NVIC_EnableIRQ(TIM3_IRQn);
   }
 }
  
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  // Enable clock for GPIOB and ADC //! <-- Must for ADC or any other peripheral
   __HAL_RCC_GPIOB_CLK_ENABLE();
   __HAL_RCC_ADC_CLK_ENABLE();
}

void GenerateSineWave()
{
    for (int i = 0; i < SAMPLES; i++)
    {
        sineWave[i] = (uint16_t)((sin(2 * M_PI * i / SAMPLES) + 1) * (DAC_RESOLUTION / 2));
    }
}

// Add this function
void uart_print(const char *fmt, ...) {
    char buf[100];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100);
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}
 

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
    HAL_Delay(100);
  }
  /* USER CODE END Error_Handler_Debug */
}