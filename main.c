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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <math.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define V_CC 5
#define RO 2.12
#define RL 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t state=0; 
volatile uint8_t uartDataReceived = 0;
volatile uint8_t adcConversionComplete = 0;
uint8_t Rx_dataDust[32];
uint8_t PmsTransfer_cplt = 0;
uint8_t inFrame = 0;
uint8_t detectOff = 0;
unsigned char Buff_Sensor[32];
uint8_t txData[12];
uint8_t frameLen;
char message[20];
static uint32_t PM1_0_temp;
static uint32_t PM2_5_temp;
static uint32_t PM10_0_temp;

uint16_t var,dem=0;
double volts,first;
uint16_t peak=0;
uint16_t max=0;
uint16_t min=4096;


char buffer1[50];
char sound_char[20];
float sound_level,value,sum,TB;
uint16_t adc[2];
float RS;
float CO=0;
	float CO_ppm (uint16_t adc_value)
{
	float VRL = adc_value * (V_CC / 4096.0);
	float RS = ((V_CC / VRL)-1)*RL;
	float ratio = RS/RO;
	float ppm = pow ((19.709/ratio), (1/0.652));
	return ppm;
}
 float MAX4466 ( uint16_t var)
 {
							if (var < 4096)  
							{
								if (var > max)
								{
									max = var; 
								}
								else if (var < min)
								{
									min = var; 
								}
							}
							peak = max - min;
					volts=((peak*3.3)/4096)*0.707;
					first=log10(volts/0.00631)*20;
					return sound_level=first+1.;
					max=0;
					min=4096;
 }
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct {
    uint16_t PM1;
    uint16_t PM2_5;
    uint16_t PM10_0;
	uint16_t CO;
	uint16_t Sound;
} SensorValues_t;
SensorValues_t ValueSensor_t;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    //-Interrupt-Function-for-Read-DUST-Data-------------------------------------------------------------------//
    if (huart->Instance == USART2)
    {	
			
      HAL_UART_Receive_IT(&huart2, (uint8_t *)Rx_dataDust, 1); 

        if (inFrame == 0)
        {
            if (Rx_dataDust[0] == 0x42 && detectOff == 0)
            {
                Buff_Sensor[detectOff] = Rx_dataDust[0];
                detectOff++;
            }
            else if (Rx_dataDust[0] == 0x4D && detectOff == 1)
            {
                Buff_Sensor[detectOff] = Rx_dataDust[0];
                inFrame = 1;
                detectOff++;
            }
        }
        else
        {
            Buff_Sensor[detectOff] = Rx_dataDust[0];
            detectOff++;
            frameLen = Buff_Sensor[3] + (Buff_Sensor[2] << 8) + 4;
            if (detectOff >= frameLen)
            {
                PmsTransfer_cplt = 1;
            }
        }
				
    }
}

int CheckSumFuncDust(unsigned char *buf)
{

    int checkSum = (buf[30] << 8) + buf[31];
    int sum = 0x42;
    for (int i = 1; i < 30; i++)
    {
        sum = sum + buf[i];
    }
    return sum == checkSum;
}
	
void getPMS7003_Value()
{
		ValueSensor_t.PM1    = 0;	
		ValueSensor_t.PM2_5  = 0; 
		ValueSensor_t.PM10_0 = 0; 
    if (PmsTransfer_cplt == 1)
    {
        if (CheckSumFuncDust(Buff_Sensor))
        {
            // Extract and process data here
						PM1_0_temp 	= Buff_Sensor[11]+(Buff_Sensor[10]<<8);
						PM2_5_temp	= Buff_Sensor[13]+(Buff_Sensor[12]<<8);
						PM10_0_temp 	= Buff_Sensor[15]+(Buff_Sensor[14]<<8);
						detectOff = 0;
						inFrame = 0;
						PmsTransfer_cplt =0;
						ValueSensor_t.PM1 		=	PM1_0_temp;
						ValueSensor_t.PM2_5		=	PM2_5_temp;
						ValueSensor_t.PM10_0	= PM10_0_temp;						
						
//				char buffer[50];
//				sprintf(buffer, "PM1: %u, PM2.5: %u, PM10: %u\n", PM1_0_temp, PM2_5_temp, PM10_0_temp);
//				HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);	
					
        }
	
    }		
}

void getADC_Value()
{
	HAL_ADC_Start(&hadc1);
	sound_level = MAX4466(HAL_ADC_GetValue(&hadc1));
	HAL_ADC_Stop(&hadc1);
	HAL_Delay(1000);
	
	HAL_ADC_Start(&hadc2);
	CO = CO_ppm(HAL_ADC_GetValue(&hadc2));
	HAL_ADC_Stop(&hadc2);
	HAL_Delay(1000);
	
	ValueSensor_t.CO	= CO;
	ValueSensor_t.Sound	= sound_level;
//	char buffer1[50];
//	sprintf(buffer1, "CO: %.2f, Sound: %.2f\n", CO, sound_level);
//	HAL_UART_Transmit(&huart3, (uint8_t *)buffer1, strlen(buffer1), HAL_MAX_DELAY);
}

void sendData()
{

//	}		
				char buffer[50];
				//sprintf(buffer, "PM1: %u, PM2.5: %u, PM10: %u\n", ValueSensor_t.PM1, ValueSensor_t.PM2_5, ValueSensor_t.PM10_0);
				//sprintf(buffer, "C0: %u, Sound: %u\n", ValueSensor_t.CO, ValueSensor_t.Sound); 
			//	sprintf(buffer, "CO: %u, Sound: %u, PM1: %u, PM2.5: %u, PM10: %u\n", ValueSensor_t.CO, ValueSensor_t.Sound, ValueSensor_t.PM1, ValueSensor_t.PM2_5, ValueSensor_t.PM10_0);
			//	HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
//					char buffer[50];
				sprintf(buffer, "PM1: %u, PM2.5: %u, PM10: %u\n", ValueSensor_t.PM1, ValueSensor_t.PM2_5, ValueSensor_t.PM10_0);
				HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, (uint8_t *)Rx_dataDust, 1);
	
            
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
						getPMS7003_Value();				
					//getADC_Value();
						sendData();
						HAL_Delay(1000);
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
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  huart3.Init.BaudRate = 9600;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SIM_PRW_GPIO_Port, SIM_PRW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SIM_PRW_Pin */
  GPIO_InitStruct.Pin = SIM_PRW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SIM_PRW_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
