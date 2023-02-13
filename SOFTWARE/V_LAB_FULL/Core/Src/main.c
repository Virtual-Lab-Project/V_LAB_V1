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
#include "string.h"
#include "math.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "STM_MY_LCD16X2.h"

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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void analog_read();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t receive_data[350];
uint8_t transmit_data1[27]="****HCDVB02CID2COMP001A####";
uint8_t transmit_data2[35]="****RDAVM12VB02CID2COMP001AXXXX####";

uint8_t RCD_VMUID[4];
uint8_t RCD_TOF[3];
uint8_t RCD_PAYLOAD[10];

uint8_t HDD_VMUID[4];
uint8_t HDD_VBUID[4];
uint8_t HDD_CID[4];
uint8_t HDD_TOF[3];
uint8_t HDD_PAYLOAD[10];

uint8_t RLD_VMUID[4];
uint8_t RLD_VBUID[4];
uint8_t RLD_CID[3];
uint8_t RLD_TOF[3];
uint8_t RLD_PAYLOAD_INT[8];
uint8_t RLD_PAYLOAD_INFO[13];

uint8_t RCD_flag,HDD_flag,RLD_flag,flag_HCD,flag_RDA;
uint8_t state=1;

uint8_t flag1,flag2,i,j,l;

uint32_t raw_value[1],live_data;


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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  LCD1602_Begin4BIT(RS_GPIO_Port, RS_Pin, EN_Pin, D4_GPIO_Port, D4_Pin, D5_Pin, D6_Pin, D7_Pin);
  LCD1602_setCursor(1,1);// 0x80RS
  LCD1602_print("VLAB_PROJECT");
  LCD1602_setCursor(2,1);// 0xC0
  LCD1602_print("V_LAB_COM");
  HAL_Delay(5000);
  LCD1602_clear();

  HAL_UART_Receive_IT(&huart2, receive_data, 251);
  HAL_UART_Transmit_IT(&huart2, transmit_data1, 247);
  HAL_UART_Transmit_IT(&huart2, transmit_data2, 35);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	switch(state)
	{
	case 1://when request configuration  data or hardware demand data or request live data frame is received from VM to VB
		if(RCD_flag==1)
		{
			state=2;
		}
		else if(HDD_flag==1)
		{
			state=3;
		}
		else if(RLD_flag==1)
		{
			analog_read();
			state=4;
		}
		break;


	case 2:  // to transmit transmit data1
		flag_HCD=1;

		break;

	case 3: // setting the GPIO pin of buzzer
		if((HDD_PAYLOAD[0]=='C' && HDD_PAYLOAD[0]=='O' && HDD_PAYLOAD[0]=='M' && HDD_PAYLOAD[0]=='P' && HDD_PAYLOAD[0]=='0' && HDD_PAYLOAD[0]=='0' && HDD_PAYLOAD[0]=='1' && HDD_PAYLOAD[0]=='A') && (HDD_PAYLOAD[0]=='C' && HDD_PAYLOAD[0]=='O' && HDD_PAYLOAD[0]=='M' && HDD_PAYLOAD[0]=='P' && HDD_PAYLOAD[0]=='0' && HDD_PAYLOAD[0]=='0' && HDD_PAYLOAD[0]=='1' && HDD_PAYLOAD[0]=='B'))
		{
	    HAL_GPIO_WritePin(BUZZER_CON_GPIO_Port, BUZZER_CON_Pin, GPIO_PIN_SET);
	    //HAL_GPIO_WritePin(LED_CON_GPIO_Port, LED_CON_Pin, GPIO_PIN_SET);
		HDD_flag=0;
		}
		break;

	case 4://To transmit the requested live data
		flag_RDA=1;

		break;

	}
  }
  return 0;
}

  /* USER CODE END 3 */



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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, RS_Pin|EN_Pin|D4_Pin|D5_Pin
                          |D6_Pin|D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_CON_GPIO_Port, BUZZER_CON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RS_Pin EN_Pin D4_Pin D5_Pin
                           D6_Pin D7_Pin */
  GPIO_InitStruct.Pin = RS_Pin|EN_Pin|D4_Pin|D5_Pin
                          |D6_Pin|D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_CON_Pin */
  GPIO_InitStruct.Pin = BUZZER_CON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_CON_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(&huart2, receive_data,251);

	l=strlen((char*)receive_data);


	//If Request_Config_Data Frame is received
			if(receive_data[0]=='*' && receive_data[1]=='*' && receive_data[2]=='*' && receive_data[3]=='*' && receive_data[4]=='R' && receive_data[5]=='C' && receive_data[6]=='D'  && receive_data[l-4]=='#' && receive_data[l-3]=='#' && receive_data[l-2]=='#' && receive_data[l-1]=='#')
							  {

								RCD_flag=1;

								  //RCD_TOF
							  for(i=4,j=0;i<7;i++,j++)
								  RCD_TOF[j]=receive_data[i];

							  	  //RCD_VMUID
							  for(i=7,j=0;i<11;i++,j++)
								  RCD_VMUID[j]=receive_data[i];

							  	  //Data
							  for(i=11,j=0;i<21;i++,j++)
								  RCD_PAYLOAD[j]=receive_data[i];
							  }

			//If demand_data is received
			else if(receive_data[0]=='*' && receive_data[1]=='*' && receive_data[2]=='*' && receive_data[3]=='*' && receive_data[4]=='H' && receive_data[5]=='D' && receive_data[6]=='D'  && receive_data[l-4]=='#' && receive_data[l-3]=='#' && receive_data[l-2]=='#' && receive_data[l-1]=='#')
								  {

									HDD_flag=1;
									  //HDD_TOF
								  for(i=4,j=0;i<7;i++,j++)
									  HDD_TOF[j]=receive_data[i];

								  	  //HDD_VMUID
								  for(i=7,j=0;i<11;i++,j++)
									  HDD_VMUID[j]=receive_data[i];

								  //HDD_VBUID
								  for(i=11,j=0;i<15;i++,j++)
									  HDD_VBUID[j]=receive_data[i];

								  //HDD_CID
								  for(i=15,j=0;i<19;i++,j++)
									  HDD_VBUID[j]=receive_data[i];

								  //HDD_Data ; Assuming only COMP001A(BUZZER) is received
								  for(i=19,j=0;receive_data[i]!='#';i++,j++)
									  HDD_PAYLOAD[j]=receive_data[i];

								  }

			//If Request_Live_data Frame is received
			else if(receive_data[0]=='*' && receive_data[1]=='*' && receive_data[2]=='*' && receive_data[3]=='*' && receive_data[4]=='R' && receive_data[5]=='L' && receive_data[6]=='D'  && receive_data[l-4]=='#' && receive_data[l-3]=='#' && receive_data[l-2]=='#' && receive_data[l-1]=='#')
									  {

											RLD_flag=1;
										  //RLD_TOF
									  for(i=4,j=0;i<7;i++,j++)
										  RLD_TOF[j]=receive_data[i];

									  	  //RLD_VMUID
									  for(i=7,j=0;i<11;i++,j++)
										  RLD_VMUID[j]=receive_data[i];

									  //RLD_VBUID
									  for(i=11,j=0;i<15;i++,j++)
										  RLD_VBUID[j]=receive_data[i];

									  //RLD_CID
									  for(i=15,j=0;i<19;i++,j++)
										  RLD_VBUID[j]=receive_data[i];

									  	  //RLD_Data1
									  for(i=19,j=0;i<27;i++,j++)
										  RLD_PAYLOAD_INT[j]=receive_data[i];

									  //RLD_Data2
									  for(i=27,j=0;i<40;i++,j++)
										  RLD_PAYLOAD_INFO[j]=receive_data[i];
									  }

}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(flag_HCD==1)
	{
	HAL_UART_Transmit_IT(&huart2, transmit_data1, 27);
	flag_HCD=0;
	}

	if(flag_RDA==1)
	{
	HAL_UART_Transmit_IT(&huart2, transmit_data2, 35);
	flag_RDA=0;
	}
}

void analog_read()
		{
	// Read analog value from MIC
				HAL_ADC_Start_DMA(&hadc1, raw_value, 1);



				transmit_data2[27]=raw_value[0]%10;
				live_data/=10;
				transmit_data2[28]=raw_value[0]%10;
				live_data/=10;
				transmit_data2[29]=raw_value[0]%10;
				live_data/=10;
				transmit_data2[30]=raw_value[0]%10;

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







