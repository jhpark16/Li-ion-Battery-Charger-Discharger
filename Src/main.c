
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "pcf8574.h"
#include "hd44780.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;

osThreadId defaultTaskHandle;
osThreadId Task2Handle;
osThreadId Task3Handle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define USB_TX_BUFFER_SIZE 64
#define USB_RX_BUFFER_SIZE 64
 
uint8_t usbCdcTxBuffer[USB_TX_BUFFER_SIZE];
uint8_t usbCdcTxBuffer2[USB_TX_BUFFER_SIZE];
uint8_t usbCdcRxBuffer[USB_RX_BUFFER_SIZE];
int received_data_size = 0;
int receive_total =0; 		

uint8_t toggle=0;
//PCF8574_HandleTypeDef	pcf;
LCD_PCF8574_HandleTypeDef	lcd;

/* Buffers used for displaying Time and Date */
uint32_t startCounter;
uint8_t aShowTime[50] = {0};
uint8_t aShowDate[50] = {0};
int ema_N=60;
double ema_k, ema_Vvalue=0.0, ema_Avalue=0.0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
void StartDefaultTask(void const * argument);
void vTask2(void const * argument);
void vTask3(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void RTC_CalendarConfig(void);
static void RTC_CalendarShow(uint8_t *showtime, uint8_t *showdate);
static uint32_t RTC_ReadTimeCounter(RTC_HandleTypeDef* hrtc);

#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))

#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000

struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f) {
//  if (DEMCR & TRCENA) {
//    while (ITM_Port32(0) == 0);
//    ITM_Port8(0) = ch;
//  }
  ITM_SendChar(ch);
  return(ch);
}


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	
	
	HAL_TIM_Base_Start_IT(&htim1);
	//LL_TIM_GenerateEvent_UPDATE(TIM1);
	//LCDI2C_init(0x27,20,4);
	// The I2C address is 0x20 + A2,A1,A0
	// A2, A1, A0 of PCF8574 are high (open)
	lcd.pcf8574.PCF_I2C_ADDRESS = 0x27;
	lcd.pcf8574.PCF_I2C_TIMEOUT = 1000;
	lcd.pcf8574.i2c.Instance = I2C1;
	lcd.pcf8574.i2c.Init.ClockSpeed = 400000;
	lcd.NUMBER_OF_LINES = NUMBER_OF_LINES_2; // Just use 2 lines instead of 4 
	lcd.type = TYPE0;

	if(LCD_Init(&lcd)!=LCD_OK){
		// error occured
		while(1);
	}
  
  RTC_CalendarConfig();
	startCounter = RTC_ReadTimeCounter(&hrtc);
	
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
  {
    // Calibration Error 
    Error_Handler();
  }
	
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Task2 */
  osThreadDef(Task2, vTask2, osPriorityLow, 0, 128);
  Task2Handle = osThreadCreate(osThread(Task2), NULL);

  /* definition and creation of Task3 */
  osThreadDef(Task3, vTask3, osPriorityIdle, 0, 128);
  Task3Handle = osThreadCreate(osThread(Task3), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC
                              |RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV128;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* ADC2 init function */
static void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 200000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef DateToUpdate;

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x32F2){
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initialize RTC and set the Time and Date 
    */
  sTime.Hours = 0x9;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  DateToUpdate.WeekDay = RTC_WEEKDAY_SATURDAY;
  DateToUpdate.Month = RTC_MONTH_JUNE;
  DateToUpdate.Date = 0x2;
  DateToUpdate.Year = 0x18;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR1,0x32F2);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 36000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SquareOut_GPIO_Port, SquareOut_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CurrentSourceCtrl_GPIO_Port, CurrentSourceCtrl_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SquareOut_Pin CurrentSourceCtrl_Pin */
  GPIO_InitStruct.Pin = SquareOut_Pin|CurrentSourceCtrl_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
  * @brief  Configure the current time and date.
  * @param  None
  * @retval None
  */
static void RTC_CalendarConfig(void)
{
  RTC_DateTypeDef sdatestructure;
  RTC_TimeTypeDef stimestructure;

  /*##-1- Configure the Date #################################################*/
  /* Set Date: Friday June 1st 2018 */
  sdatestructure.Year = 0x18;
  sdatestructure.Month = RTC_MONTH_JUNE;
  sdatestructure.Date = 0x1;
  sdatestructure.WeekDay = RTC_WEEKDAY_FRIDAY;
  
  if(HAL_RTC_SetDate(&hrtc,&sdatestructure,RTC_FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Configure the Time #################################################*/
  /* Set Time: 02:00:00 */
	if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x32F2)
	{
		stimestructure.Hours = 0x02;
		stimestructure.Minutes = 0x00;
		stimestructure.Seconds = 0x00;
		
		if (HAL_RTC_SetTime(&hrtc, &stimestructure, RTC_FORMAT_BCD) != HAL_OK)
		{
			/* Initialization Error */
			Error_Handler();
		}
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2);
	}
			/*##-3- Writes a data in a RTC Backup data Register1 #######################*/
}

/**
  * @brief  Display the current time and date.
  * @param  showtime : pointer to buffer
  * @param  showdate : pointer to buffer
  * @retval None
  */
static void RTC_CalendarShow(uint8_t *showtime, uint8_t *showdate)
{
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructureget;

  /* Get the RTC current Time */
  HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
  /* Display time Format : hh:mm:ss */
  sprintf((char *)showtime, "%2d:%2d:%2d", stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);
  /* Display date Format : mm-dd-yy */
  sprintf((char *)showdate, "%2d-%2d-%2d", sdatestructureget.Month, sdatestructureget.Date, 2000 + sdatestructureget.Year);
}

static uint32_t RTC_ReadTimeCounter(RTC_HandleTypeDef* hrtc)
{
  uint16_t high1 = 0U, high2 = 0U, low = 0U;
  uint32_t timecounter = 0U;

  high1 = READ_REG(hrtc->Instance->CNTH & RTC_CNTH_RTC_CNT);
  low   = READ_REG(hrtc->Instance->CNTL & RTC_CNTL_RTC_CNT);
  high2 = READ_REG(hrtc->Instance->CNTH & RTC_CNTH_RTC_CNT);

  if (high1 != high2)
  { /* In this case the counter roll over during reading of CNTL and CNTH registers, 
       read again CNTL register then return the counter value */
    timecounter = (((uint32_t) high2 << 16U) | READ_REG(hrtc->Instance->CNTL & RTC_CNTL_RTC_CNT));
  }
  else
  { /* No counter roll over during reading of CNTL and CNTH registers, counter 
       value is equal to first value of CNTL and CNTH */
    timecounter = (((uint32_t) high1 << 16U) | low);
  }

  return timecounter;
}

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 5 */
	int maxSize = 0;
  /* Infinite loop */
  for(int i=0;;i++)
  {
		printf("Task Default:%d\n",i);
		if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
		{
      USBD_CDC_SetRxBuffer(&hUsbDeviceFS, usbCdcRxBuffer);
		  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
			int size = USBD_LL_GetRxDataSize(&hUsbDeviceFS, CDC_OUT_EP);
			if(size>maxSize) maxSize = size;
  		LCD_SetLocation(&lcd,14,3);
	  	LCD_WriteNumber(&lcd,maxSize,10);
			sprintf((char*)usbCdcTxBuffer,"Test%d\n",i);
			int n = strlen((char*)usbCdcTxBuffer);
			LCD_SetLocation(&lcd, 0, 3);
			if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
			{
				USBD_CDC_SetTxBuffer(&hUsbDeviceFS, usbCdcTxBuffer, n);
				USBD_CDC_TransmitPacket(&hUsbDeviceFS);
				CDC_Transmit_FS(usbCdcTxBuffer, n);
				LCD_WriteString(&lcd,"USB connected=      ");
			}
			else
			{
				LCD_WriteString(&lcd,"USB disconnected    ");
			}
		}
    osDelay(1000);
  }
  /* USER CODE END 5 */ 
}

/* vTask2 function */
void vTask2(void const * argument)
{
  /* USER CODE BEGIN vTask2 */
	char iMode = 1; // discharge
	double cnvVFactor = 1.0/4091.0*(3.3*0.989965903)/0.327317676;
	double cnvAFactor = 1.0/4091.0*(3.3*0.989965903)*0.28714866;
	ema_k = 2.0/(ema_N+1.0);
	double voltage = 8.4;
	double current = 0.5;
	LCD_ClearDisplay(&lcd);
	LCD_SetLocation(&lcd, 0, 0);
	LCD_WriteString(&lcd, "V=      V I=0.5  A");
//	lBuffer[20]=0;
	HAL_GPIO_WritePin(CurrentSourceCtrl_GPIO_Port, CurrentSourceCtrl_Pin, GPIO_PIN_RESET);
  osDelay(100);
	if (HAL_ADC_Start(&hadc1) != HAL_OK) {
		/* ADC conversion start error */
		Error_Handler();
	}
	if (HAL_ADC_Start(&hadc2) != HAL_OK) {
		/* ADC conversion start error */
		Error_Handler();
	}
	if (HAL_ADC_PollForConversion(&hadc1, 50) != HAL_OK) {
		/* ADC polling error */
		Error_Handler();
	}
	if (HAL_ADC_PollForConversion(&hadc2, 50) != HAL_OK)	{
		/* ADC polling error */
		Error_Handler();
	}
	uint32_t iVValue = HAL_ADC_GetValue(&hadc1);
	ema_Vvalue =  iVValue;
	uint32_t iAValue = HAL_ADC_GetValue(&hadc2);
	ema_Avalue =  iAValue;
  // Infinite loop 
  for(int i=0;;i++)
  {
		printf("Task2:%d\n",i);
		if (HAL_ADC_Start(&hadc1) != HAL_OK) {
			/* ADC conversion start error */
			Error_Handler();
		}
		if (HAL_ADC_Start(&hadc2) != HAL_OK) {
			/* ADC conversion start error */
			Error_Handler();
		}
		if (HAL_ADC_PollForConversion(&hadc1, 50) != HAL_OK) {
			/* ADC polling error */
			Error_Handler();
		}
		if (HAL_ADC_PollForConversion(&hadc2, 50) != HAL_OK) {
			/* ADC polling error */
			Error_Handler();
		}
		iVValue = HAL_ADC_GetValue(&hadc1);
		ema_Vvalue = iVValue*ema_k+ema_Vvalue*(1.0-ema_k);
		iAValue = HAL_ADC_GetValue(&hadc2);
		ema_Avalue = iAValue*ema_k+ema_Avalue*(1.0-ema_k);
		voltage = ema_Vvalue*cnvVFactor;
		current = ema_Avalue*cnvAFactor;
		// STM32 cannot reliably measure voltage less than 0.1V
		if (current <0.03) current = 0.0; 
		LCD_SetLocation(&lcd, 0, 0);
		sprintf((char*)usbCdcTxBuffer,"V=%6.4fV I=%5.3fA",voltage,current);
		LCD_WriteString(&lcd,(char*) usbCdcTxBuffer);
		RTC_CalendarShow(aShowTime, aShowDate);
		LCD_SetLocation(&lcd, 0, 2);
		LCD_WriteString(&lcd,(char*) aShowTime);
		uint32_t currentCounter = RTC_ReadTimeCounter(&hrtc);
		currentCounter -= startCounter;
		if(iMode){
			LCD_SetLocation(&lcd,0,1);
			sprintf((char*)usbCdcTxBuffer,"%5u sec %6.1f mAh",(unsigned int)currentCounter,currentCounter/3.6*0.5);
			LCD_WriteString(&lcd,(char*) usbCdcTxBuffer);
			if(currentCounter >= 30 && iMode == 1 && voltage <6.00) {
				HAL_GPIO_WritePin(CurrentSourceCtrl_GPIO_Port, CurrentSourceCtrl_Pin, GPIO_PIN_SET);
				iMode = 0;
			}
			else if(currentCounter >= 30 && iMode == 2 && voltage >8.46) { // with 8.7V supply voltage
				HAL_GPIO_WritePin(CurrentSourceCtrl_GPIO_Port, CurrentSourceCtrl_Pin, GPIO_PIN_SET);
				iMode = 0;
			}
		}
		voltage = iVValue*cnvVFactor;
		current = iAValue*cnvAFactor;
		LCD_SetLocation(&lcd, 0, 3);
		sprintf((char*)usbCdcTxBuffer,"%6.4fV, %5.3fA",voltage,current);
		LCD_WriteString(&lcd,(char*) usbCdcTxBuffer);
    osDelay(490);
  }
  /* USER CODE END vTask2 */
}

/* vTask3 function */
void vTask3(void const * argument)
{
  /* USER CODE BEGIN vTask3 */
  // Infinite loop 
  for(int i=0;;i++)
  {
		printf("Task3 LED blinking:%d\n",i);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    osDelay(500);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    osDelay(500);
  }
  /* USER CODE END vTask3 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
