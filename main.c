/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	
	low=0x00,
	high=0xff,
	
}Bool;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t python_flag=0;

uint16_t ADCvalues[4];
uint16_t dcYuk[4];

uint16_t RisingEdgeD;
uint16_t frekans;

char tx_buffer[50];
char rx_buffer[1];
char flowMeter[50];

Bool DigitalOUT11;
Bool DigitalOUT10;
Bool DigitalOUT1;
Bool DigitalOUT0;
Bool manual_flag=low;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1; //ADC 4 channel
DMA_HandleTypeDef hdma_adc1; //ADC Direct Memory Access

TIM_HandleTypeDef htim1; //to generate pwm signals
TIM_HandleTypeDef htim2; //to capture the rising edges of the flow sensor output

UART_HandleTypeDef huart1; //serial communication with the PC via serial port

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){ //RX interrupt handler. 
	manual_flag=high; //manual mode is activated 
		switch (rx_buffer[0])
	{
		//DC-Motor control flags
		//dcYuk-1 manual ON/OFF + LED5 indicator of the process
    case 'A':
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET); 
			python_flag=0;
      break;

    case 'B':
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
			python_flag=1;
      break;
		//dcYuk-2 manual ON/OFF + LED6 indicator of the process
		case 'C':
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
			python_flag=2;
      break;
		
		case 'D':
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
			python_flag=3;
      break;
		//dcYuk-3 manual ON/OFF + LED7 indicator of the process
		case 'E':
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
			python_flag=4;
      break;
		
		case 'F':
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
			python_flag=5;
      break;
		//dcYuk-4 manual ON/OFF + LED8 indicator of the process
		case 'G':
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
			python_flag=6;
      break;
		
		case 'H':
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
			python_flag=7;
      break;
		//LED1-2-3-4 manual indication
		case '1':
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET); 
      break;
    case '2':
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
      break;
		case '3':
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
      break;
		case '4':
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
      break;
		case '5':
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);
      break;
		case '6':
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);
      break;
		case '7':
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET);
      break;
		case '8':
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);
      break;
		//exit from the manual mode
		case 'O':
			manual_flag=low;
		break;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){ //ADC with DMA + TIM1 duty cycles + manual control via serial interface	
		if(hadc->Instance==ADC1){
			dcYuk[0]=ADCvalues[0]; //channel-0
			dcYuk[1]=ADCvalues[1]; //channel-1
			dcYuk[2]=ADCvalues[2]; //channel-2
			dcYuk[3]=ADCvalues[3]; //channel-3
			if(manual_flag==low){
				if(dcYuk[0]>= 2048){  //dcYuk1 is either HIGH or LOW not between
					htim1.Instance-> CCR1 = 100;
				}
				else if(dcYuk[0] < 2048){
					htim1.Instance-> CCR1 = 0;
				}
				htim1.Instance-> CCR2 = 100*dcYuk[1]/4095; //making duty cycles between 0-100
				htim1.Instance-> CCR3 = 100*dcYuk[2]/4095;
				htim1.Instance-> CCR4 = 100*dcYuk[3]/4095;
			}
			else if(manual_flag==high){ //manual control with buttons on python interface via serial com.
				char *str5="dcYuk-1 is ON\r\n";
				char *str6="dcYuk-1 is OFF\r\n";
				char *str7="dcYuk-2 is ON\r\n";
				char *str8="dcYuk-2 is OFF\r\n";
				char *str9="dcYuk-3 is ON\r\n";
				char *str10="dcYuk-3 is OFF\r\n";
				char *str11="dcYuk-4 is ON\r\n";
				char *str12="dcYuk-4 is OFF\r\n";
				
				switch (python_flag){ //In manual Control Duty cycles are fixed to "0" and "100"
					//+ serial port indicaiton
					
					case 0:
						HAL_UART_Transmit(&huart1,(uint8_t*)str5,strlen(str5),1000);
						htim1.Instance-> CCR1 = 100;
						break;
					
					case 1:
						HAL_UART_Transmit(&huart1,(uint8_t*)str6,strlen(str6),1000);
						htim1.Instance-> CCR1 =	0;
						break;
					
					case 2:
						HAL_UART_Transmit(&huart1,(uint8_t*)str7,strlen(str7),1000);
						htim1.Instance-> CCR2 = 100;
						break;
					
					case 3:
						HAL_UART_Transmit(&huart1,(uint8_t*)str8,strlen(str8),1000);
						htim1.Instance-> CCR2 = 0;
						break;
					
					case 4:
						HAL_UART_Transmit(&huart1,(uint8_t*)str9,strlen(str9),1000);
						htim1.Instance-> CCR3 = 100;
						break;
					
					case 5:
						HAL_UART_Transmit(&huart1,(uint8_t*)str10,strlen(str10),1000);
						htim1.Instance-> CCR3 = 0;
						break;
					
					case 6:
						HAL_UART_Transmit(&huart1,(uint8_t*)str11,strlen(str11),1000);
						htim1.Instance-> CCR4 = 100;
						break;
					
					case 7:
						HAL_UART_Transmit(&huart1,(uint8_t*)str12,strlen(str12),1000);
						htim1.Instance-> CCR4 = 0;
						break;	
				}
			}	
		}
} 

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){ //flow sensor + capture mode of timer2
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1){
			RisingEdgeD=HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_1);
			frekans=72000000/RisingEdgeD; //calculating the frequency of the signal coming based on tim2 frequency which is 72mhz
		}
	}

void DIGITAL_READER(){ //Digital Reader of 4 digital input pin
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15)==1){
		 DigitalOUT11=high;
	}
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15)==0){
		 DigitalOUT11=low;
	}
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)==1){
		 DigitalOUT10=high;
	}
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)==0){
		 DigitalOUT10=low;
	}
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13)==1){
		 DigitalOUT1=high;
	}
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13)==0){
		 DigitalOUT1=low;
	}
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12)==1){
		 DigitalOUT0=high;
	}
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12)==0){
		 DigitalOUT0=low;
	}
}

void Serial_Transmission_and_Leds(){
		if(manual_flag==low){ //Streaming 4-channel ADC data and the frequency of the flow sensor every 250ms
		sprintf(tx_buffer,"streaming %02d::%02d::%02d::%02d\r\n",dcYuk[0],dcYuk[1],dcYuk[2],dcYuk[3]);
		sprintf(flowMeter,"frequency is %02d \r\n",frekans);
		HAL_UART_Transmit(&huart1,(uint8_t*)tx_buffer,strlen(tx_buffer),HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart1,(uint8_t*)flowMeter,strlen(flowMeter),HAL_MAX_DELAY);
			//led and serial indicator of the digital inputs
		if( DigitalOUT11==high){ 
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET);
			char *str1="LED4 is ON\r\n";
			HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),1000);
		}
		if( DigitalOUT11==low){
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);
			char *str1="LED4 is OFF\r\n";
			HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),1000);
		}
		if( DigitalOUT10==high){
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);
			char *str1="LED3 is ON\r\n";
			HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),1000);
		}
		if( DigitalOUT10==low){
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);
			char *str2="LED3 is OFF\r\n";
			HAL_UART_Transmit(&huart1,(uint8_t*)str2,strlen(str2),1000);
		}
		if( DigitalOUT1==high){
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
			char *str2="LED2 is ON\r\n";
			HAL_UART_Transmit(&huart1,(uint8_t*)str2,strlen(str2),1000);
		}
		if( DigitalOUT1==low){
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
			char *str3="LED2 is OFF\r\n";
			HAL_UART_Transmit(&huart1,(uint8_t*)str3,strlen(str3),1000);
		}
		if( DigitalOUT0==high){
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
			char *str4="LED1 is ON\r\n";
			HAL_UART_Transmit(&huart1,(uint8_t*)str4,strlen(str4),1000);
		}
		if( DigitalOUT0==low){
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
			char *str4="LED1 IS OFF\r\n";
			HAL_UART_Transmit(&huart1,(uint8_t*)str4,strlen(str4),1000);
		}
		HAL_Delay(250); // Transmitting the datas every 250 ms
	}
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADCvalues,4); //ADC - DMA
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1); 
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1); //Input-Capture interrup
	HAL_UART_Receive_IT(&huart1,(uint8_t*)rx_buffer,1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		DIGITAL_READER();
		Serial_Transmission_and_Leds();
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
