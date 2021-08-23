/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


//TODO use a infinite loop till user enters the HR or SPO2 mode


#include "fonts.h"
#include "ssd1306.h"
#include "stdio.h"
#include "string.h"
#include "MAX30102.h"
//#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define NUM_BAT_ADC_READ_AVG		50
// Minimum allowed voltage for the battery
// to function properly proportional to NUM_BAT_ADC_READ_AVG
// which is equal to 3.6 * NUM_BAT_ADC_READ_AVG
#define MIN_ALLOWED_VOLTAGE_SUM 	180	//(3.6 * NUM_BAT_ADC_READ_AVG)
#define SPO2ORHR_HR					'h'
#define SPO2ORHR_SPO2				's'

#define maxSampleCheck				30
#define HRMAX						115
#define HRMIN						70
#define SPO2MAX						100
#define SPO2MIN						95

#define TEST_Pin GPIO_PIN_15
#define TEST_GPIO_Port GPIOB
#define INT_Pin GPIO_PIN_9
#define INT_GPIO_Port GPIOA
#define INT_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

float _BatteryVoltage(void);
void _BluetoothSend(uint8_t value, uint8_t SPO2_OR_HR);
void _Show_HeartRate_SPO2(uint8_t value, uint8_t SPO2_OR_HR);
char _BluetoothReceive(void);
void _BootUpSequence(void);
void _Check_HR_SPO2(uint8_t data, char selectedMode);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// counter for battery ADC value reading
uint8_t _counter = 0;

// Average battery voltage
float _BAT_Voltage_AVG = 0.00;

// 500 ms interrupt
uint8_t _timer_3_interrupt = 0;

// 300 ms interrupt
uint8_t _timer_2_interrupt = 0;

// User heart rate
uint8_t _heartRate = 0;

// User SPO2
uint8_t _SPO2 = 0;

// The initiated values to warn the user
uint8_t dangerZoneDataSum = 0;
uint8_t dangerZoneDataCnt = 0;


// User input buffer
char blueToothInput;

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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */


  SSD1306_Init();
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  Max30102_Init(&hi2c2);

  // Boot up sequence
    _BootUpSequence();

  // Refresh the OLED display
	SSD1306_Clear();
	SSD1306_UpdateScreen();

  // Resetting the state of the vibrarion motor
	HAL_GPIO_WritePin(Motor_GPIO_Port, Motor_Pin, GPIO_PIN_RESET);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 Max30102_Task();

	 if(_timer_3_interrupt)
	 {
		 if(blueToothInput == SPO2ORHR_HR){
			 _heartRate = (uint8_t)Max30102_GetHeartRate();

			 // Sending the heart rate value with bluetooth
			 _BluetoothSend(_heartRate, SPO2ORHR_HR);

			 // Showing heart rate on the display
			 _Show_HeartRate_SPO2(_heartRate, SPO2ORHR_HR);

			 // Checking HR and SPO2
			 _Check_HR_SPO2(_heartRate, SPO2ORHR_HR);


			 HAL_GPIO_TogglePin(User_Led_GPIO_Port, User_Led_Pin);
		 }

		 _timer_3_interrupt = 0;
	 }

	 if(_timer_2_interrupt)
	 {
	  /*
	   * Battery voltage exhibition section
	   */
	  // Preparing to convert the float value into string to be shown on display
	  char voltageText[10];
	  float BAT_Voltage = _BatteryVoltage();
	  _BAT_Voltage_AVG += BAT_Voltage;
	  _counter++;

	  if(_counter == NUM_BAT_ADC_READ_AVG)
	  {
		  _BAT_Voltage_AVG /= _counter;
		  sprintf(voltageText, "Bat_Vol: %.2f", _BAT_Voltage_AVG);
		  // Printing the battery voltage value on display
		  SSD1306_SetPosition(0, 0);
		  SSD1306_PrintString(voltageText, &Font_7x10, SSD1306_COLOR_WHITE);
		  SSD1306_UpdateScreen();

		  // Checking if the battery voltage is low
		  if(_BAT_Voltage_AVG <= MIN_ALLOWED_VOLTAGE_SUM){

			  // Printing the warning message
			  //SSD1306_Clear();
			  SSD1306_SetPosition(0, 90);
			  SSD1306_PrintString("Low BAT.", &Font_7x10, SSD1306_COLOR_WHITE);
			  SSD1306_UpdateScreen();
		  }


		  _counter = 0;
	  }

	  if(blueToothInput == SPO2ORHR_SPO2){

		 _SPO2 = (uint8_t) Max30102_GetSpO2Value();

		 // Sending the heart rate value with bluetooth
		 _BluetoothSend(_SPO2, SPO2ORHR_SPO2);

		 // Showing heart rate on the display
		 _Show_HeartRate_SPO2(_SPO2, SPO2ORHR_SPO2);

		 // Checking HR and SPO2
		 _Check_HR_SPO2(_SPO2, SPO2ORHR_SPO2);
	  }


	  _timer_2_interrupt = 0;
	 }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2400;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
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
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(User_Led_GPIO_Port, User_Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Motor_GPIO_Port, Motor_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : User_Led_Pin */
  GPIO_InitStruct.Pin = User_Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(User_Led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Motor_Pin */
  GPIO_InitStruct.Pin = Motor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Motor_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_Pin */
  GPIO_InitStruct.Pin = INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

// Boot up sequence
void _BootUpSequence(void){

	SSD1306_SetPosition(0, 0);
	SSD1306_PrintString("choose HR or SPO2", &Font_7x10, SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();

	//blueToothInput = _BluetoothReceive();

	// Waiting till the user chooses the mode
	while(1){
		// Reading user's choice of HR or SPO2
		blueToothInput = _BluetoothReceive();
		if((blueToothInput == SPO2ORHR_HR) || (blueToothInput == SPO2ORHR_SPO2))
			break;
	}
}

// receiving the user choice of HR or SPO2
char _BluetoothReceive(void){
	char value;
	HAL_UART_Receive(&huart2, &value, sizeof(value), 5000);
	return value;
}

// Get and show heart rate and blood oxygen level
void _Show_HeartRate_SPO2(uint8_t value, uint8_t SPO2_OR_HR){

	// if SPO2 is selected
	if(SPO2_OR_HR == SPO2ORHR_SPO2){
		char _valueText[9];
		sprintf(_valueText, "SPO2:   %3d", value);
		SSD1306_SetPosition(0, 25);
		SSD1306_PrintString(_valueText, &Font_11x18, SSD1306_COLOR_WHITE);
		SSD1306_UpdateScreen();
	}
	// if HR is selected
	else if(SPO2_OR_HR == SPO2ORHR_HR){
		char _valueText[9];
		sprintf(_valueText, "BPM:    %3d", value);
		SSD1306_SetPosition(0, 25);
		SSD1306_PrintString(_valueText, &Font_11x18, SSD1306_COLOR_WHITE);
		SSD1306_UpdateScreen();
	}
}


// Battery voltage reading
float _BatteryVoltage(void){
	uint16_t rawAnalogVal = 0;
	float batteryVoltage = 0.0;

	// Initializing the ADC
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);

	// Reading ADC Value
	// NOTE: it is a 12-bit ADC
	rawAnalogVal = HAL_ADC_GetValue(&hadc1);

	// Mapping the ADC value to readable voltage
	// NOTE: the value is multiplied by 2 because of the voltage divider circuit
	// which divides the voltage by 2
	batteryVoltage = (rawAnalogVal * 2  / 4095.00) * 3.30;
	// Removing some inefficiency
	//batteryVoltage += 0.1;
	HAL_ADC_Stop(&hadc1);
	return batteryVoltage;
}


void _BluetoothSend(uint8_t value, uint8_t SPO2_OR_HR){
	char UartBuffer[8];
	//uint8_t val = (uint8_t)value;
	if(SPO2_OR_HR == SPO2ORHR_HR)
	{
		sprintf(UartBuffer, "%d",  value);
		HAL_UART_Transmit(&huart2, (uint8_t*)UartBuffer, strlen(UartBuffer), 100);
		HAL_Delay(100);
	}

	else if(SPO2_OR_HR == SPO2ORHR_SPO2){
		sprintf(UartBuffer, "%d", value);
		HAL_UART_Transmit(&huart2, (uint8_t*)UartBuffer, strlen(UartBuffer), 100);
		HAL_Delay(100);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == INT_Pin) {
		Max30102_InterruptCallback();
	}
}


// Timer_2 ISR
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if(htim->Instance == TIM2)
	  _timer_2_interrupt = 1;
  else if(htim->Instance == TIM3)
	  _timer_3_interrupt = 1;
}

// CHecking the HR and SPO2
void _Check_HR_SPO2(uint8_t data, char selectedMode){
        //heart rate has been selected
        if(selectedMode == SPO2ORHR_HR) {
            // heart rate exceeds the natural level
            if (data > HRMAX) {
                dangerZoneDataCnt++;
                if (dangerZoneDataCnt < maxSampleCheck) {
                    dangerZoneDataSum += data;
                }
                if (dangerZoneDataSum > HRMAX * maxSampleCheck) {
                	// Setup the vibration motor
                	HAL_GPIO_WritePin(Motor_GPIO_Port, Motor_Pin, GPIO_PIN_SET);
                }
            }

            // heart rate is less than natural level minimum
            else if (data < HRMIN) {
                dangerZoneDataCnt++;
                if (dangerZoneDataCnt < maxSampleCheck) {
                    dangerZoneDataSum += data;
                }
                if (dangerZoneDataSum < HRMIN * maxSampleCheck) {
                    // Setup the vibration motor
                	HAL_GPIO_WritePin(Motor_GPIO_Port, Motor_Pin, GPIO_PIN_SET);
                }
            }
            else{
                dangerZoneDataCnt = 0;
            }
        }
            // oxygen level has been selected
            else if(selectedMode == SPO2ORHR_SPO2){
                // oxygen level exceeds the natural level
                if(data > SPO2MAX){
                    dangerZoneDataCnt++;
                    if (dangerZoneDataCnt < maxSampleCheck){
                        dangerZoneDataSum += data;
                    }
                    if (dangerZoneDataSum > SPO2MAX * maxSampleCheck){
                    	// Setup the vibration motor
                    	HAL_GPIO_WritePin(Motor_GPIO_Port, Motor_Pin, GPIO_PIN_SET);
                    }
                }

                // oxygen level is less than natural level minimum
                else if(data < SPO2MIN) {
                    dangerZoneDataCnt++;
                    if (dangerZoneDataCnt < maxSampleCheck) {
                        dangerZoneDataSum += data;
                    }
                    if (dangerZoneDataSum < SPO2MIN * maxSampleCheck) {
                    	// Setup the vibration motor
                    	HAL_GPIO_WritePin(Motor_GPIO_Port, Motor_Pin, GPIO_PIN_SET);
                    }
                }
                else{
                    dangerZoneDataCnt = 0;
                }
        }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
