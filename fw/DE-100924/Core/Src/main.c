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
#include "rfid.h"
#include "logger.h"
#include "eeprom.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	BUZZ_OFF = 0,
	BUZZ_CARD_VALID,
	BUZZ_CARD_INVALID,
	BUZZ_ERROR
} buzz_tdf;

typedef enum
{
	LED_OFF = 0,
	LED_CARD_VALID,
	LED_CARD_INVALID,
	LED_CARD_BLOCKED,
	LED_ERROR
} led_tdf;

typedef enum
{
	SYS_OK = 0,
	SYS_TXRX,
	SYS_ERR
} status_tdf;

typedef enum
{
	CLOSED = 0,
	OPEN
} doorlock_tdf;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t sysfl;
RTC_TimeTypeDef rtime;
RTC_DateTypeDef rdate;
static uint8_t rsadd;
static uint8_t rsbuf[64], rscnt;
static uint32_t door_time;
static uint16_t card_cnt = 0;
static uint8_t buzz_cnt = 0, sta_cnt = 0, door_cnt = 0, led_cnt = 0;
static uint32_t buzz_tmr, sta_tmr, door_tmr, led_tmr, rstmr, rstmr2;
static uint8_t raw_data_buffer[12], rdcnt = 0;
static uint8_t card_id_buffer[6] = {0}, card_id_buffer2[6] = {0}, card_id_buffer3[6] = {0};
static uint8_t bit_cnt;
static uint8_t byte_cnt;
static uint8_t if_flag;
static uint8_t shift;
static doorlock_tdf door = CLOSED;
static status_tdf state = SYS_OK;
static led_tdf led = LED_OFF;
static buzz_tdf buzz = BUZZ_OFF;
const uint16_t max_card_number = 250;
static enum
{
	RECEIVING = 0U, PROCESSING, CHECKING,
} RFID_State = RECEIVING;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */
static void init(void);
static void rs485(void);
static void save_flags(void);
static void card_valid(void);
static void card_invalid(void);
static void card_blocked(void);
static void save_door_tmr(void);
static void enable_rfid_receiver(void);
static void enable_rs485_receiver(void);
static void new_event(const uint8_t* bcd_card);
static uint8_t find_card(const uint8_t* bcd_card);
static uint8_t write_card(const uint8_t* bcd_card);
static uint8_t delete_card(const uint8_t* bcd_card);
static uint8_t delete_all_cards(void);
static uint8_t delete_all_events(void);
static uint8_t ParityCheck(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

	/* USER CODE BEGIN 1 */
	uint8_t c = 0, d = 0;
	uint8_t even_p = 0, odd_p = 0;
	uint32_t rec_tmr = 0, rec_tmout = 0;
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
	MX_IWDG_Init();
	MX_USART1_UART_Init();
	MX_CRC_Init();
	MX_RTC_Init();
	MX_TIM14_Init();
	/* USER CODE BEGIN 2 */
	init();
	LOGGER_Init();
	rfid_process();
	rstmr2 = HAL_GetTick(); // load restart timer
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while(1)
	{
		rs485();
		rfid_process();
		LOGGER_Service();

		switch(RFID_State)
		{
			case RECEIVING:
				if(HAL_GetTick() - rec_tmr >= 200)
				{
					rec_tmr = HAL_GetTick();
					rfid_read_tag();
				}
				if(rec_tmout && (HAL_GetTick() - rec_tmout >= 300)) // reset receiving buffer after timeout
				{
					rdcnt = 0;
					rec_tmout = 0;
					ZEROFILL(card_id_buffer,  COUNTOF(card_id_buffer));
					ZEROFILL(card_id_buffer2, COUNTOF(card_id_buffer2));
					ZEROFILL(card_id_buffer3, COUNTOF(card_id_buffer3));
				}
				break;

			case PROCESSING:
				if(ParityCheck() == 0)
				{
					for(c = 1; c < 10; c += 2)
					{
						d = (raw_data_buffer[c] & 0x0f) << 4;
						d += (raw_data_buffer[c + 1] & 0x0f);
						card_id_buffer[c / 2] = d;
					}
					even_p = 0;
					for(c = 0; c < 8; c++)
					{
						if(card_id_buffer[2] & (1 << c))
							++even_p;
					}
					for(c = 4; c < 8; c++)
					{
						if(card_id_buffer[3] & (1 << c))
							++even_p;
					}
					if(even_p & 0x01)
						even_p = 1;
					else
						even_p = 0;
					odd_p = 0;
					for(c = 0; c < 4; c++)
					{
						if(card_id_buffer[3] & (1 << c))
							++odd_p;
					}
					for(c = 0; c < 8; c++)
					{
						if(card_id_buffer[4] & (1 << c))
							++odd_p;
					}
					if(odd_p & 0x01)
						odd_p = 0;
					else
						odd_p = 1;
					RFID_State = CHECKING;
				}
				else
				{
					RFID_State = RECEIVING;
				}
				c = 2;
				d = 8;
				break;

			case CHECKING:
				card_id_buffer[5] = '\0';
				uint32_t tmp = card_id_buffer[1] + card_id_buffer[2] + card_id_buffer[3] + card_id_buffer[4];
				if(tmp && (tmp < (4 * 0xFF)))
				{

					if(rdcnt == 2)
					{
						rdcnt = 0;
						if((mem_comp(&card_id_buffer[1], &card_id_buffer2[1], 4U) == 0U) &&
						        (mem_comp(&card_id_buffer2[1], &card_id_buffer3[1], 4U) == 0U))
						{
							ZEROFILL(card_id_buffer2, COUNTOF(card_id_buffer2));
							ZEROFILL(card_id_buffer3, COUNTOF(card_id_buffer3));
							if(HAL_TIM_Base_Stop_IT(&htim14) != HAL_OK)
							{
								Error_Handler();
							}
							HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
							if(find_card(&card_id_buffer[1]) == 0) card_valid();
							else  card_invalid();
							new_event(&card_id_buffer[1]);
						}
					}
					else if(rdcnt == 1)
					{
						rdcnt = 2;
						mem_copy(&card_id_buffer3[1], &card_id_buffer[1], 4);
					}
					else
					{
						rdcnt = 1;
						mem_copy(&card_id_buffer2[1], &card_id_buffer[1], 4);
					}
				}
				c = 5;
				d = 8;
				shift = 0;
				odd_p = 0xff;
				even_p = 0xff;
				RFID_State = RECEIVING;
				rec_tmout = HAL_GetTick(); // set receiving timeout
				ZEROFILL(card_id_buffer, COUNTOF(card_id_buffer));
				break;
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		// restart reader after 10min inactiv communication and not reading any card
		if(rstmr2)
		{
			if((HAL_GetTick() - rstmr2) >= 600000U)
			{
				Error_Handler();
			}
		}
#ifdef	USE_WATCHDOG
		HAL_IWDG_Refresh(&hiwdg);
#endif
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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
	                              | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_I2C1
	                                     | RCC_PERIPHCLK_RTC;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

	/* USER CODE BEGIN CRC_Init 0 */

	/* USER CODE END CRC_Init 0 */

	/* USER CODE BEGIN CRC_Init 1 */

	/* USER CODE END CRC_Init 1 */
	hcrc.Instance = CRC;
	hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
	hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
	hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
	hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
	if(HAL_CRC_Init(&hcrc) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CRC_Init 2 */

	/* USER CODE END CRC_Init 2 */

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
	hi2c1.Init.Timing = 0x00100001;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if(HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Analogue filter
	*/
	if(HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Digital filter
	*/
	if(HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	{
		Error_Handler();
	}

	/** I2C Fast mode Plus enable
	*/
	__HAL_SYSCFG_FASTMODEPLUS_ENABLE(I2C_FASTMODEPLUS_I2C1);
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

	/* USER CODE BEGIN IWDG_Init 0 */
#ifdef USE_WATCHDOG
	/* USER CODE END IWDG_Init 0 */

	/* USER CODE BEGIN IWDG_Init 1 */

	/* USER CODE END IWDG_Init 1 */
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
	hiwdg.Init.Window = 4095;
	hiwdg.Init.Reload = 4095;
	if(HAL_IWDG_Init(&hiwdg) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN IWDG_Init 2 */
#endif
	/* USER CODE END IWDG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */

	/** Initialize RTC Only
	*/
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if(HAL_RTC_Init(&hrtc) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

	/* USER CODE BEGIN TIM14_Init 0 */

	/* USER CODE END TIM14_Init 0 */

	/* USER CODE BEGIN TIM14_Init 1 */

	/* USER CODE END TIM14_Init 1 */
	htim14.Instance = TIM14;
	htim14.Init.Prescaler = 0;
	htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim14.Init.Period = 12287;
	htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if(HAL_TIM_Base_Init(&htim14) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM14_Init 2 */
	if(HAL_TIM_Base_Start_IT(&htim14) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE END TIM14_Init 2 */

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
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if(HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */
	rscnt = 0;
	ZEROFILL(rsbuf, COUNTOF(rsbuf));
	HAL_UART_Receive_IT(&huart1, rsbuf, 1);
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
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LED_RED_Pin | DOORLOCK_Pin | SPI1_SCK_Pin | SPI1_MOSI_Pin
	                  | LED_GREEN_Pin | WGNDT0_Pin | WGNDT0A12_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : LED_RED_Pin DOORLOCK_Pin LED_GREEN_Pin WGNDT0_Pin
	                         WGNDT0A12_Pin */
	GPIO_InitStruct.Pin = LED_RED_Pin | DOORLOCK_Pin | LED_GREEN_Pin | WGNDT0_Pin
	                      | WGNDT0A12_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : SPI1_SCK_Pin SPI1_MOSI_Pin */
	GPIO_InitStruct.Pin = SPI1_SCK_Pin | SPI1_MOSI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : SPI1_MISO_Pin */
	GPIO_InitStruct.Pin = SPI1_MISO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SPI1_MISO_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : ADDRESS0_Pin ADDRESS1_Pin ADDRESS2_Pin ADDRESS3_Pin
	                         ADDRESS4_Pin ADDRESS5_Pin */
	GPIO_InitStruct.Pin = ADDRESS0_Pin | ADDRESS1_Pin | ADDRESS2_Pin | ADDRESS3_Pin
	                      | ADDRESS4_Pin | ADDRESS5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : BUZZER_Pin */
	GPIO_InitStruct.Pin = BUZZER_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  init app
  * @param  none
  * @retval none
  */
static void init(void)
{
	static uint8_t buf[LOG_DSIZE];
	static uint32_t add;

#ifdef CLEAR_EEPROM
	uint8_t clr[EE_PGSIZE + 2];
	uint32_t iad;
	ZEROFILL(clr, COUNTOF(clr));
	for(iad = 0; iad <= (EE_ENDADDR - EE_PGSIZE); iad += EE_PGSIZE)
	{
		clr[0] = iad >> 8;
		clr[1] = iad & 0xFFU;
		if(HAL_I2C_IsDeviceReady(&hi2c1, EE_ADDR, DRV_TRIAL, DRV_TOUT)         != HAL_OK)  Error_Handler();
		if(HAL_I2C_Master_Transmit(&hi2c1, EE_ADDR, clr, EE_PGSIZE + 2, DRV_TOUT) != HAL_OK)  Error_Handler();
		HAL_Delay(10);
	}
	door_time = 5;
	DoorlockEnable();
	BuzzerEnable();
	buf[0] = 0U;
	buf[1] = 0U;
	buf[2] = sysfl;
	buf[3] = door_time;
	if(HAL_I2C_IsDeviceReady(&hi2c1, EE_ADDR, DRV_TRIAL, DRV_TOUT) != HAL_OK)   Error_Handler();
	if(HAL_I2C_Master_Transmit(&hi2c1, EE_ADDR, buf, 4, DRV_TOUT)  != HAL_OK)   Error_Handler();
	HAL_Delay(10);
#endif

	rsadd = 0;
	if(HAL_GPIO_ReadPin(ADDRESS0_GPIO_Port, ADDRESS0_Pin) == GPIO_PIN_RESET) rsadd |= 0x01U;
	if(HAL_GPIO_ReadPin(ADDRESS1_GPIO_Port, ADDRESS1_Pin) == GPIO_PIN_RESET) rsadd |= 0x02U;
	if(HAL_GPIO_ReadPin(ADDRESS2_GPIO_Port, ADDRESS2_Pin) == GPIO_PIN_RESET) rsadd |= 0x04U;
	if(HAL_GPIO_ReadPin(ADDRESS3_GPIO_Port, ADDRESS3_Pin) == GPIO_PIN_RESET) rsadd |= 0x08U;
	if(HAL_GPIO_ReadPin(ADDRESS4_GPIO_Port, ADDRESS4_Pin) == GPIO_PIN_RESET) rsadd |= 0x10U;
	if(HAL_GPIO_ReadPin(ADDRESS5_GPIO_Port, ADDRESS5_Pin) == GPIO_PIN_RESET) rsadd |= 0x20U;

	buf[0] = 0U;
	buf[1] = 0U;
	if(HAL_I2C_IsDeviceReady(&hi2c1, EE_ADDR, DRV_TRIAL, DRV_TOUT) != HAL_OK)  Error_Handler();
	if(HAL_I2C_Master_Transmit(&hi2c1, EE_ADDR, buf, 2U, DRV_TOUT) != HAL_OK)  Error_Handler();
	if(HAL_I2C_Master_Receive(&hi2c1, EE_ADDR, buf, LOG_DSIZE, DRV_TOUT)   != HAL_OK) Error_Handler();
	sysfl = buf[0];
	door_time = (uint32_t) buf[1] * 1000U;
	if(!IsRFIDEnabled()) // omoguci rfid uvijek nakon restarta, ovo je samo interni flag
	{
		RfidEnable();
		save_flags();
	}

	for(add = EE_CARD_LIST_START_ADDR; add <= (EE_CARD_LIST_START_ADDR + (max_card_number * 0x4U)); add += 4U)
	{
		buf[0] = add >> 8U;
		buf[1] = add & 0xFFU;
		if(HAL_I2C_IsDeviceReady(&hi2c1, 0xA0, DRV_TRIAL, DRV_TOUT) != HAL_OK)  Error_Handler();
		if(HAL_I2C_Master_Transmit(&hi2c1, 0xA0, buf, 2U, DRV_TOUT) != HAL_OK)  Error_Handler();
		if(HAL_I2C_Master_Receive(&hi2c1, 0xA0, buf, 4, DRV_TOUT)  != HAL_OK)  Error_Handler();
		if((buf[0] | buf[1] | buf[2] | buf[3]) != 0U)
		{
			++card_cnt;
		}
	}
}
/**
  * @brief  send new card event to pc app
            if pc app not acknowledge receiving event
            event is loged
  * @param  pointer to card received
  * @retval none
  */
static void new_event(const uint8_t* bcd_card)
{
	uint16_t crc = 0, i, t = 0;
	uint8_t buf[16];
	HAL_UART_AbortReceive(&huart1);
	ZEROFILL(buf, COUNTOF(buf));
	buf[0] = SOH;
	buf[1] = 0xFE;
	buf[2] = rsadd;
	buf[3] = 0x6U;
	buf[4] = GET_EVENT_LAST;
	buf[5] = led;
	mem_copy(&buf[6], bcd_card, 4);
	for(i = 0; i < 10; i++)
	{
		crc += buf[i];
	}
	buf[10] = crc >> 8;
	buf[11] = crc & 0xFFU;
	buf[12] = EOT;
	rscnt = 0;
	ZEROFILL(rsbuf, COUNTOF(rsbuf));
	HAL_UART_Transmit(&huart1, buf, 13U, RESP_TOUT);
	__HAL_UART_CLEAR_PEFLAG(&huart1);
	__HAL_UART_CLEAR_FEFLAG(&huart1);
	__HAL_UART_CLEAR_NEFLAG(&huart1);
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	__HAL_UART_CLEAR_OREFLAG(&huart1);
	HAL_UART_AbortReceive(&huart1);
	HAL_UART_Receive(&huart1, rsbuf, 8, 100);
	if((rsbuf[0] == SOH)   && (rsbuf[1] ==  rsadd) && \
	        (rsbuf[2] == 0xFEU) && (rsbuf[3] ==  0x1U) && \
	        (rsbuf[4] == ACK)   && (rsbuf[7] ==  EOT))
	{
		crc = 0;
		for(i = 0; i < (rsbuf[3] + 4); i++)
		{
			crc += rsbuf[i];
		}
		if(((rsbuf[5]) == (crc >> 8U)) && ((rsbuf[6]) == (crc & 0xFFU))) t = 1;
	}
	if(t == 0)
	{
		logger.log_event = led;
		mem_copy(&logger.log_card_id[0], bcd_card, 4);
		LOGGER_Write();
	}
	enable_rs485_receiver();
}
/**
  * @brief process received rs485 data and response
  * @param

    BAJT 0 = SOH (standardni ASCII "start of header" kontrolni char, 1 decimalno, 0x01 hexadecimalno)
    BAJT 1 = ADRESA RECEIVERA (adresa CITACA ili u odgovoru 0xFE adresa PC aplikacije) ili 0xFF brodkast adresa
    BAJT 2 = ADRESA TRANSMITERA (0xFE adresa PC aplikacije ili u odgovoru adresa CITACA)
    BAJT 3 = DUŽINA PAKETA (broj bajta korisnog dijela ovog paketa koji saljemo prijemniku pocevsi od i ukljucujuci slijedeceg)
    BAJT 4 = KOMANDA
                        GET_SYS_FLAG        0xA0 // traži flagove sistema
                        GET_CARD_CNT        0xA1 // broj kartica u eepromu
                        GET_CARD_PRESENT    0xA2 // traži kartice u eepromu
                        GET_EVENT_CNT       0xA3 // broj dogadaja u eepromu
                        GET_EVENT_LAST      0xA4 // traži zadnji dogadaj iz eeproma

                        SET_SYS_TIME        0xB0 // podesi vrijeme u BCD formatu: dan/datum/mjesec/godina/sat/minuta/sekunda
                        SET_SYS_RESTART     0xB1 // softwerski restart applikacije
                        SET_CARD_ONE        0xB2 // dodaj novu karticu u eeprom
                        SET_DOOR_OPEN       0xB3 // aktiviraj bravu
                        SET_DOOR_TIME       0xB4 // podesi vrijeme brave
                        SET_DOOR_ENABLE     0xB5 // omoguci kontrolu brave
                        SET_DOOR_DISABLE    0xB6 // onemoguci kontrolu brave
                        SET_BUZZER_ENABLE   0xB7 // omoguci buzzer
                        SET_BUZZER_DISABLE  0xB8 // onemoguci buzzer

                        DELETE_CARD_ONE     0xC0 // obrisi karticu iz eeproma
                        DELETE_CARD_ALL     0xC1 // obrisi sve kartice iz eeproma
                        DELETE_EVENT_LAST   0xC2 // obrisi dogadaj prema datom indexu iz eeproma
                        DELETE_EVENT_ALL    0xC3 // obrisi sve dogadaje iz eeproma

    BAJT 5,6... = BROJ KARTICE ILI VRIJEME (ako se šalje broj kartice ili vrijeme)

    BAJT ZADNJI - 2 = MSB CEKSUMA KOMANDE (16 bitni zbir svih bajta paketa, do prije ovog bajta)
    BAJT ZADNJI - 1 = LSB CEKSUMA KOMANDE
    BAJT ZADNJI		= EOT (standardni ASCII "end of transmission" kontrolni char, 4 decimalno, 0x04 hexadecimalno)

  * @retval
  */
static void rs485(void)
{
	static uint8_t i, resp[32];
	uint16_t crc = 0;
	if((rsbuf[0] == SOH) && (rsbuf[1] ==  rsadd) && (rsbuf[rsbuf[3] + 6U] == EOT))
	{
		HAL_UART_AbortReceive(&huart1);

		for(i = 0; i < (rsbuf[3] + 4); i++)
		{
			crc += rsbuf[i];
		}

		if(((rsbuf[rsbuf[3] + 4U]) == (crc >> 8U)) && ((rsbuf[rsbuf[3] + 5U]) == (crc & 0xFFU)))
		{
			resp[0] = SOH;
			resp[1] = 0xFEU;
			resp[2] = rsadd;
			resp[3] = 2U;
			resp[4] = rsbuf[4];
			resp[5] = ACK;

			switch(rsbuf[4])
			{
				case GET_SYS_FLAG:
					resp[5] = sysfl;
					break;

				case GET_CARD_CNT:
					resp[3] = 3U;
					resp[5] = card_cnt >> 8;
					resp[6] = card_cnt & 0xFFU;
					break;

				case GET_CARD_PRESENT:
					if(find_card(&rsbuf[5]) != 0) resp[5] = NAK;
					break;

				case GET_EVENT_CNT:
					resp[3] = 3U;
					resp[5] = logger_list_count >> 8;
					resp[6] = logger_list_count & 0xFFU;
					break;

				case GET_EVENT_LAST:
					resp[3] = 17U;
					if(LOGGER_Read(&resp[5]) == LOGGER_EMPTY)
					{
						resp[3] = 2U;
						resp[5] = LOGGER_EMPTY;
					}
					break;

				case SET_SYS_TIME:
					Str2Hex((const char*)&rsbuf[5], &rdate.Date, 2);
					Str2Hex((const char*)&rsbuf[7], &rdate.Month, 2);
					Str2Hex((const char*)&rsbuf[9], &rdate.Year, 2);
					Str2Hex((const char*)&rsbuf[11], &rtime.Hours, 2);
					Str2Hex((const char*)&rsbuf[13], &rtime.Minutes, 2);
					Str2Hex((const char*)&rsbuf[15], &rtime.Seconds, 2);
					HAL_RTC_SetTime(&hrtc, &rtime, RTC_FORMAT_BCD);
					HAL_RTC_SetDate(&hrtc, &rdate, RTC_FORMAT_BCD);
					break;

				case SET_SYS_RESTART:
					Error_Handler();
					break;

				case SET_CARD_ONE:
					if(find_card(&rsbuf[5]) != 0)
					{
						if(write_card(&rsbuf[5]) != 0) resp[5] = NAK;
					}
					break;

				case SET_DOOR_OPEN:
//					door = OPEN;
//					DoorlockEnable();
					card_valid();
					break;

				case SET_DOOR_TIME:
					door_time = (uint32_t) rsbuf[5] * 1000U;
					save_door_tmr();
					break;

				case SET_DOOR_ENABLE:
					DoorlockEnable();
					save_flags();
					break;

				case SET_DOOR_DISABLE:
					DoorlockDisable();
					save_flags();
					break;

				case SET_BUZZER_ENABLE:
					BuzzerEnable();
					save_flags();
					break;

				case SET_BUZZER_DISABLE:
					BuzzerDisable();
					save_flags();
					break;

				case DELETE_CARD_ONE:
					if(delete_card(&rsbuf[5]) != 0) resp[5] = NAK;
					break;

				case DELETE_CARD_ALL:
					delete_all_cards();
					break;

				case DELETE_EVENT_ALL:
					delete_all_events();
					break;

				case DELETE_EVENT_LAST:
					resp[5] = LOGGER_Delete();
					break;

				default:
					resp[3] = 0;
					break;
			}

			if(resp[3])
			{
				crc = 0;
				for(i = 0; i < (resp[3] + 4); i++)
				{
					crc += resp[i];
				}
				resp[resp[3] + 4U] = (crc >> 8U);
				resp[resp[3] + 5U] = (crc & 0xFFU);
				resp[resp[3] + 6U] = EOT;
				HAL_UART_Transmit(&huart1, resp, resp[3] + 7U, RESP_TOUT);
			}
		}
		enable_rs485_receiver();
	}
	// resetuj receiver 100 ms nakon zadnje primljenog bajta
	if(rstmr)
	{
		if((HAL_GetTick() - rstmr) >= 100U)
		{
			enable_rs485_receiver();
		}
	}
}
/**
  * @brief  buzzer, led, doorlock
  * @param
  * @retval
  */
void status(void)
{
	//********************* BUZZER SIGNAL *********************//
	if(IsBuzzerEnabled())
	{
		switch(buzz)
		{
			case BUZZ_CARD_VALID:
				switch(buzz_cnt)
				{
					case 0:
						BuzzerOn();
						buzz_tmr = HAL_GetTick();
						++buzz_cnt;
						break;
					case 1:
					default:
						if((HAL_GetTick() - buzz_tmr) >= 50U) buzz = BUZZ_OFF;
						break;
				}
				break;
			case BUZZ_CARD_INVALID:
				if((HAL_GetTick() - buzz_tmr) >= 50U)
				{
					if(IsBuzzerOn()) BuzzerOff();
					else BuzzerOn();
					buzz_tmr = HAL_GetTick();
					if(++buzz_cnt >= 8U) buzz = BUZZ_OFF;
				}
				break;
			case BUZZ_ERROR:
				if(((buzz_cnt & 0x1U) == 0U) && ((HAL_GetTick() - buzz_tmr) >= 5000U))
				{
					BuzzerOn();
					buzz_tmr = HAL_GetTick();
					++buzz_cnt;
				}
				else if((HAL_GetTick() - buzz_tmr) >= 100U)
				{
					BuzzerOff();
					buzz_tmr = HAL_GetTick();
					++buzz_cnt;
				}
				break;
			default:
			case BUZZ_OFF:
				buzz_cnt = 0;
				buzz_tmr = 0;
				BuzzerOff();
				break;
		}
	}
	else
	{
		buzz_cnt = 0;
		buzz_tmr = 0;
		BuzzerOff();
	}
	//********************* STATUS LED SIGNAL *********************//
	switch(state)
	{
		case SYS_OK:
			if(((sta_cnt & 0x1U) == 0U) && ((HAL_GetTick() - sta_tmr) >= 1000U))
			{
//				StatusLedOn();
				sta_tmr = HAL_GetTick();
				++sta_cnt;
			}
			else if((sta_cnt & 0x1U)  && ((HAL_GetTick() - sta_tmr) >= 100U))
			{
//				StatusLedOff();
				sta_tmr = HAL_GetTick();
				sta_cnt = 0;
			}
			break;

		case SYS_TXRX:
			if((HAL_GetTick() - sta_tmr) >= 100U)
			{
//				if(IsStatusLedOn()) StatusLedOff();
//				else StatusLedOn();
				sta_tmr = HAL_GetTick();
				if(++sta_cnt >= 10U) state = SYS_OK;
			}
			break;

		default:
		case SYS_ERR:
			if((HAL_GetTick() - sta_tmr) >= 100U)
			{
//				if(IsStatusLedOn()) StatusLedOff();
//				else StatusLedOn();
				sta_tmr = HAL_GetTick();
			}
			break;
	}
	//********************* USER LED SIGNAL *********************//
	switch(led)
	{

		case LED_CARD_VALID:
		case LED_CARD_BLOCKED:
			switch(led_cnt)
			{
				case 0:
					if(led == LED_CARD_VALID) GreenLedOn(), RedLedOff();
					else if(led == LED_CARD_BLOCKED) GreenLedOn(), RedLedOn();
					led_tmr = HAL_GetTick();
					++led_cnt;
					break;
				case 1:
				default:
					if((HAL_GetTick() - led_tmr) >= 2000U)
					{
						led_cnt = 0;
						led = LED_OFF;
						enable_rfid_receiver();
					}
					break;
			}
			break;
		case LED_CARD_INVALID:
			if((HAL_GetTick() - led_tmr) >= 50U)
			{
				if(IsRedLedOn()) RedLedOff();
				else RedLedOn();
				led_tmr = HAL_GetTick();
				if(++led_cnt >= 8U)
				{
					led_cnt = 0;
					led = LED_OFF;
					enable_rfid_receiver();
				}
			}
			break;
		case LED_ERROR:
			if(((led_cnt & 0x1U) == 0U) && ((HAL_GetTick() - led_tmr) >= 200U))
			{
				RedLedOn();
				GreenLedOn();
				led_tmr = HAL_GetTick();
				++led_cnt;
			}
			else if((led_cnt & 0x1U) && ((HAL_GetTick() - led_tmr) >= 100U))
			{
				RedLedOff();
				GreenLedOff();
				led_tmr = HAL_GetTick();
				led_cnt = 0;
			}
			break;
		default:
		case LED_OFF:
			RedLedOn();
			GreenLedOff();
			break;

	}
	//********************* DOORLOCK OUTPUT *********************//
	if(IsDoorlockEnabled())
	{
		switch(door)
		{
			case OPEN:
				switch(door_cnt)
				{
					case 0:
						DoorlockOn();
						door_tmr = HAL_GetTick();
						++door_cnt;
						break;
					case 1:
					default:
						if((HAL_GetTick() - door_tmr) >= door_time) door = CLOSED;
						break;
				}
				break;

			default:
			case CLOSED:
				DoorlockOff();
				door_cnt = 0;
				door_tmr = 0;
				break;
		}
	}
	else
	{
		DoorlockOff();
		door_cnt = 0;
		door_tmr = 0;
	}

}
/**
  * @brief  reset and start receivin data from rf485
  * @param  none
  * @retval none
  */
static void enable_rs485_receiver(void)
{
	__HAL_UART_CLEAR_PEFLAG(&huart1);
	__HAL_UART_CLEAR_FEFLAG(&huart1);
	__HAL_UART_CLEAR_NEFLAG(&huart1);
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	__HAL_UART_CLEAR_OREFLAG(&huart1);
	HAL_UART_AbortReceive(&huart1);
	rscnt = 0;
	rstmr = 0;
	ZEROFILL(rsbuf, COUNTOF(rsbuf));
	HAL_UART_Receive_IT(&huart1, rsbuf, 1);
}
/**
  * @brief  start receivin data from rfid reader
  * @param  none
  * @retval none
  */
static void enable_rfid_receiver(void)
{
	HAL_TIM_Base_Start_IT(&htim14);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}
/**
  * @brief  set signalin for card valid event
  * @param  none
  * @retval none
  */
static void card_valid(void)
{
	if(IsDoorlockEnabled())
	{
		led_cnt = 0;
		door_cnt = 0;
		buzz_cnt = 0;
		buzz_tmr = 0;
		door_tmr = 0;
		led_tmr = 0;
		BuzzerOff();
		RedLedOff();
		GreenLedOff();
		door = OPEN;
		led = LED_CARD_VALID;
		buzz = BUZZ_CARD_VALID;
	}
	else card_blocked();
	rstmr2 = HAL_GetTick(); // relod restart timer
}
/**
  * @brief  set signalin for card invalid event
  * @param  none
  * @retval none
  */
static void card_invalid(void)
{
	led_cnt = 0;
	door_cnt = 0;
	buzz_cnt = 0;
	buzz_tmr = 0;
	door_tmr = 0;
	led_tmr = 0;
	BuzzerOff();
	RedLedOff();
	GreenLedOff();
	door = CLOSED;
	led = LED_CARD_INVALID;
	buzz = BUZZ_CARD_INVALID;
	rstmr2 = HAL_GetTick(); //  reload restart timer
}
/**
  * @brief  set signalin for reader doorlock blocked event
  * @param  none
  * @retval none
  */
static void card_blocked(void)
{
	led_cnt = 0;
	door_cnt = 0;
	buzz_cnt = 0;
	buzz_tmr = 0;
	door_tmr = 0;
	led_tmr = 0;
	BuzzerOff();
	RedLedOff();
	GreenLedOff();
	door = CLOSED;
	led = LED_CARD_BLOCKED;
	buzz = BUZZ_CARD_INVALID;
	rstmr2 = HAL_GetTick(); // reload restart timer
}
/**
  * @brief  find in eeprom card id
  * @param  pointer to card id in 5 byte bcd format (10 digit)
  * @retval 0 = OK or 1 = NOT FOUND
  */
static uint8_t find_card(const uint8_t* bcd_card)
{
	uint8_t buf[4];
	uint32_t add = EE_CARD_LIST_START_ADDR;

	for(add = EE_CARD_LIST_START_ADDR; add <= (EE_CARD_LIST_START_ADDR + (max_card_number * 0x4U)); add += 0x4U)
	{
		buf[0] = add >> 8U;
		buf[1] = add & 0xFFU;
		if(HAL_I2C_IsDeviceReady(&hi2c1, EE_ADDR, DRV_TRIAL, DRV_TOUT) != HAL_OK)  Error_Handler();
		if(HAL_I2C_Master_Transmit(&hi2c1, EE_ADDR, buf, 2U, DRV_TOUT) != HAL_OK)  Error_Handler();
		if(HAL_I2C_Master_Receive(&hi2c1, EE_ADDR, buf, 4U, DRV_TOUT)  != HAL_OK)  Error_Handler();
		if(mem_comp(bcd_card, buf, 4U) == 0U) return 0U;
	}
	return (1U);
}
/**
  * @brief  write card on first available eeprom address
  * @param  pointer to card id in 5 byte bcd format
  * @retval 0 = OK or 1 = WRITE ERROR
  */
static uint8_t write_card(const uint8_t* bcd_card)
{
	uint8_t buf[6];
	uint32_t add = EE_CARD_LIST_START_ADDR;

	for(add = EE_CARD_LIST_START_ADDR; add <= (EE_CARD_LIST_START_ADDR + (max_card_number * 0x4U)); add += 4U)
	{
		buf[0] = add >> 8U;
		buf[1] = add & 0xFFU;
		if(HAL_I2C_IsDeviceReady(&hi2c1, EE_ADDR, DRV_TRIAL, DRV_TOUT) != HAL_OK)  Error_Handler();
		if(HAL_I2C_Master_Transmit(&hi2c1, EE_ADDR, buf, 2U, DRV_TOUT) != HAL_OK)  Error_Handler();
		if(HAL_I2C_Master_Receive(&hi2c1, EE_ADDR, buf, 4U, DRV_TOUT)  != HAL_OK)  Error_Handler();
		if((buf[0] | buf[1] | buf[2] | buf[3]) == 0U)
		{
			buf[0] = add >> 8U;
			buf[1] = add & 0xFFU;
			mem_copy(&buf[2], bcd_card, 4);
			if(HAL_I2C_IsDeviceReady(&hi2c1, EE_ADDR, DRV_TRIAL, DRV_TOUT) != HAL_OK)     Error_Handler();
			if(HAL_I2C_Master_Transmit(&hi2c1, EE_ADDR, buf, 0x6U, DRV_TOUT) != HAL_OK)   Error_Handler();
			HAL_Delay(10);
			++card_cnt;
			return 0U;
		}
	}
	return (1U);
}
/**
  * @brief  delete card from eeprom
  * @param  pointer to card id in 5 byte bcd format
  * @retval 0 = OK or 1 = NOT FOUND
  */
static uint8_t delete_card(const uint8_t* bcd_card)
{
	uint8_t buf[6];
	uint32_t add = EE_CARD_LIST_START_ADDR;
	for(add = EE_CARD_LIST_START_ADDR; add <= (EE_CARD_LIST_START_ADDR + (max_card_number * 0x4U)); add += 4U)
	{
		buf[0] = add >> 8U;
		buf[1] = add & 0xFFU;
		if(HAL_I2C_IsDeviceReady(&hi2c1, EE_ADDR, DRV_TRIAL, DRV_TOUT) != HAL_OK)   Error_Handler();
		if(HAL_I2C_Master_Transmit(&hi2c1, EE_ADDR, buf, 2U, DRV_TOUT) != HAL_OK)   Error_Handler();
		if(HAL_I2C_Master_Receive(&hi2c1, EE_ADDR, buf, 4U, DRV_TOUT) != HAL_OK)   Error_Handler();
		if(mem_comp(bcd_card, buf, 4U) == 0U)
		{
			ZEROFILL(buf, COUNTOF(buf));
			buf[0] = add >> 8U;
			buf[1] = add & 0xFFU;
			if(HAL_I2C_IsDeviceReady(&hi2c1, EE_ADDR, DRV_TRIAL, DRV_TOUT) != HAL_OK)      Error_Handler();
			if(HAL_I2C_Master_Transmit(&hi2c1, EE_ADDR, buf, 0x6U, DRV_TOUT) != HAL_OK)   Error_Handler();
			HAL_Delay(10);
			--card_cnt;
			return 0U;
		}
	}
	return (1U);
}
/**
  * @brief  delete card from eeprom
  * @param  pointer to card id in 5 byte bcd format
  * @retval 0 = OK or system restart
  */
static uint8_t delete_all_cards(void)
{
	uint8_t buf[EE_PGSIZE + 2];
	uint32_t add;
	ZEROFILL(buf, COUNTOF(buf));
	for(add = EE_CARD_LIST_START_ADDR; add <= (EE_CARD_LIST_END_ADDR - EE_PGSIZE); add += EE_PGSIZE)
	{
		buf[0] = add >> 8U;
		buf[1] = add & 0xFFU;
		if(HAL_I2C_IsDeviceReady(&hi2c1, EE_ADDR, DRV_TRIAL, DRV_TOUT) != HAL_OK)         Error_Handler();
		if(HAL_I2C_Master_Transmit(&hi2c1, EE_ADDR, buf, EE_PGSIZE + 2, DRV_TOUT) != HAL_OK)Error_Handler();
		HAL_Delay(10);
	}
	card_cnt = 0;
	return 0U;
}
/**
  * @brief  delete all events from eeprom
  * @param  none
  * @retval 0 = OK or system restart
  */
static uint8_t delete_all_events(void)
{
	uint8_t buf[EE_PGSIZE + 2];
	uint32_t add;
	ZEROFILL(buf, COUNTOF(buf));
	for(add = EE_LOG_LIST_START_ADDR; add <= (EE_LOG_LIST_END_ADDR - EE_PGSIZE); add += EE_PGSIZE)
	{
		buf[0] = add >> 8U;
		buf[1] = add & 0xFFU;
		if(HAL_I2C_IsDeviceReady(&hi2c1, EE_ADDR, DRV_TRIAL, DRV_TOUT)         != HAL_OK)  Error_Handler();
		if(HAL_I2C_Master_Transmit(&hi2c1, EE_ADDR, buf, EE_PGSIZE + 2, DRV_TOUT) != HAL_OK)  Error_Handler();
		HAL_Delay(10);
	}
	LOGGER_Init();
	return 0U;
}
/**
  * @brief  save door time
  * @param
  * @retval
  */
static void save_door_tmr(void)
{
	uint8_t tmp = door_time / 1000U;
	uint8_t buf[3];
	buf[0] = 0;
	buf[1] = 0x1;
	buf[2] = tmp;
	if(HAL_I2C_IsDeviceReady(&hi2c1, EE_ADDR, DRV_TRIAL, DRV_TOUT) != HAL_OK)  Error_Handler();
	if(HAL_I2C_Master_Transmit(&hi2c1, EE_ADDR, buf, 3, DRV_TOUT)  != HAL_OK)  Error_Handler();
	HAL_Delay(10);
}
/**
  * @brief  save sys flags
  * @param
  * @retval
  */
static void save_flags(void)
{
	uint8_t buf[3];
	buf[0] = 0;
	buf[1] = 0;
	buf[2] = sysfl;
	if(HAL_I2C_IsDeviceReady(&hi2c1, EE_ADDR, DRV_TRIAL, DRV_TOUT) != HAL_OK)  Error_Handler();
	if(HAL_I2C_Master_Transmit(&hi2c1, EE_ADDR, buf, 3, DRV_TOUT)  != HAL_OK)  Error_Handler();
	HAL_Delay(10);
}
/**
  * @brief
  * @param
  * @retval
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	rstmr = HAL_GetTick();
	rstmr2 = HAL_GetTick();
	if(++rscnt > sizeof(rsbuf)) rscnt = 0;
	HAL_UART_Receive_IT(&huart1, &rsbuf[rscnt], 1);
}
/**
  * @brief
  * @param
  * @retval
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
}
/**
  * @brief
  * @param
  * @retval
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	enable_rs485_receiver();
}
/**
  * @brief
  * @param
  * @retval
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == SPI1_MISO_Pin)
	{
		__HAL_TIM_SET_COUNTER(&htim14, 0);
		__HAL_TIM_SET_AUTORELOAD(&htim14, 18430); // interrupt after 3/4 bit time (384us);
		if_flag = 1;  // rising edge interrupt executed
	}
}
/**
  * @brief
  * @param
  * @retval
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(RFID_State != RECEIVING)
	{
		__HAL_TIM_SET_COUNTER(&htim14, 0);
		__HAL_TIM_SET_AUTORELOAD(&htim14, 12287); // interrupt after 1/2 bit time (256us);
	}
	else
	{
		if(HAL_GPIO_ReadPin(SPI1_MISO_GPIO_Port, SPI1_MISO_Pin) == GPIO_PIN_RESET)
		{
			__HAL_TIM_SET_AUTORELOAD(&htim14, 24574); // interrupt after 1 bit time (512us);
			if_flag = shift ^ 1;
		}
		else
		{
			__HAL_TIM_SET_AUTORELOAD(&htim14, 12287); // interrupt after 1/2 bit time (256us);
			if(if_flag)
				if_flag = shift;
		}

		shift = (shift << 1);
		shift += (if_flag & 1);

		if((if_flag > 127) && (shift == 0xff))
		{
			bit_cnt = 0xff;
			byte_cnt = 1;
			shift = 1;
		}

		if(byte_cnt)
		{
			if(++bit_cnt == 5)
			{
				bit_cnt = 0;
				raw_data_buffer[byte_cnt] = shift & 0x1f;
				if(++byte_cnt > 11)
				{
					byte_cnt = 0;
//					HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
					RFID_State = PROCESSING;
				}
			}
		}
		if_flag = 0;
	}
}

static uint8_t ParityCheck(void)
{
	uint8_t c, x, hor, ver;
	ver = 0;
	hor = 0;
	c = 0;
	x = 0;

	for(c = 1; c < 11; ++c)
	{
		ver ^= raw_data_buffer[c];
		hor = 0;
		for(x = 0x10; x > 1; x = (x >> 1))
		{
			if(raw_data_buffer[c] & x)
			{
				++hor;
			}
		}

		if((hor & 1) != (raw_data_buffer[c] & 1))
			return 1; // horisontal parity error
		raw_data_buffer[c] = (raw_data_buffer[c] >> 1);
	}

	if(raw_data_buffer[c] & 1)
		return 3;    // horisontal parity of vertical paritys not 0
	raw_data_buffer[c] = (raw_data_buffer[c] >> 1);
	ver = (ver >> 1);

	if(ver != raw_data_buffer[c])
		return 2; // vertical parity error

	return 0;   // parity ok
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
	/* USER CODE BEGIN Init */
	HAL_CRC_MspDeInit(&hcrc);
	HAL_RTC_MspDeInit(&hrtc);
	HAL_I2C_MspDeInit(&hi2c1);
	HAL_UART_MspDeInit(&huart1);
	HAL_TIM_Base_MspDeInit(&htim14);
	HAL_DeInit();
	/* USER CODE END Init */
	while(1)
	{
		HAL_NVIC_SystemReset();
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
