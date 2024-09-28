/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "common.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern CRC_HandleTypeDef hcrc;
extern I2C_HandleTypeDef hi2c1;
extern IWDG_HandleTypeDef hiwdg;
extern RTC_HandleTypeDef hrtc;
extern TIM_HandleTypeDef htim14;
extern UART_HandleTypeDef huart1;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void status(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_RED_Pin GPIO_PIN_0
#define LED_RED_GPIO_Port GPIOA
#define DOORLOCK_Pin GPIO_PIN_4
#define DOORLOCK_GPIO_Port GPIOA
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_MISO_EXTI_IRQn EXTI4_15_IRQn
#define SPI1_MOSI_Pin GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOA
#define ADDRESS0_Pin GPIO_PIN_0
#define ADDRESS0_GPIO_Port GPIOB
#define ADDRESS1_Pin GPIO_PIN_1
#define ADDRESS1_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_8
#define LED_GREEN_GPIO_Port GPIOA
#define WGNDT0_Pin GPIO_PIN_11
#define WGNDT0_GPIO_Port GPIOA
#define WGNDT0A12_Pin GPIO_PIN_12
#define WGNDT0A12_GPIO_Port GPIOA
#define ADDRESS2_Pin GPIO_PIN_3
#define ADDRESS2_GPIO_Port GPIOB
#define ADDRESS3_Pin GPIO_PIN_4
#define ADDRESS3_GPIO_Port GPIOB
#define ADDRESS4_Pin GPIO_PIN_5
#define ADDRESS4_GPIO_Port GPIOB
#define ADDRESS5_Pin GPIO_PIN_6
#define ADDRESS5_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_7
#define BUZZER_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define GET_SYS_FLAG        0xA0 // traži flagove sistema
#define GET_CARD_CNT        0xA1 // broj kartica u eepromu
#define GET_CARD_PRESENT    0xA2 // traži karticu u eepromu
#define GET_EVENT_CNT       0xA3 // broj dogadaja u eepromu
#define GET_EVENT_LAST      0xA4 // traži zadnji dogadaj iz eeproma

#define SET_SYS_TIME        0xB0 // podesi vrijeme sa param
#define SET_SYS_RESTART     0xB1 // softwerski restart applikacije
#define SET_CARD_ONE        0xB2 // dodaj novu karticu u eeprom
#define SET_DOOR_OPEN       0xB3 // aktiviraj bravu
#define SET_DOOR_TIME       0xB4 // podesi vrijeme brave
#define SET_DOOR_ENABLE     0xB5 // omoguci kontrolu brave
#define SET_DOOR_DISABLE    0xB6 // onemoguci kontrolu brave
#define SET_BUZZER_ENABLE   0xB7 // omoguci buzzer
#define SET_BUZZER_DISABLE  0xB8 // onemoguci buzzer

#define DELETE_CARD_ONE     0xC0 // obrisi karticu iz eeproma
#define DELETE_CARD_ALL     0xC1 // obrisi sve kartice iz eeproma
#define DELETE_EVENT_LAST   0xC2 // obrisi dogadaj prema datom indexu iz eeproma
#define DELETE_EVENT_ALL    0xC3 // obrisi sve dogadaje iz eeproma


extern uint8_t sysfl, spirxtx;
#define SYS_NewLogSet()             (sysfl |=  (1U<<0))
#define SYS_NewLogReset()           (sysfl &=(~(1U<<0)))
#define IsSYS_NewLogSet()           (sysfl &   (1U<<0))
#define SYS_LogListFullSet()        (sysfl |=  (1U<<1))
#define SYS_LogListFullReset()      (sysfl &=(~(1U<<1)))
#define IsSYS_LogListFullSet()      (sysfl &   (1U<<1))
#define DoorlockEnable()            (sysfl |=  (1U<<2))
#define DoorlockDisable()           (sysfl &=(~(1U<<2)))
#define IsDoorlockEnabled()         (sysfl &   (1U<<2))
#define BuzzerEnable()              (sysfl |=  (1U<<3))
#define BuzzerDisable()             (sysfl &=(~(1U<<3)))
#define IsBuzzerEnabled()           (sysfl &   (1U<<3))
#define RfidEnable()                (sysfl |=  (1U<<4))
#define RfidDisable()               (sysfl &=(~(1U<<4)))
#define IsRFIDEnabled()             (sysfl &   (1U<<4))

#define EE_EnableWR()               (1)
#define EE_DisableWR()              (0)
#define BuzzerOn()                  (HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET))
#define BuzzerOff()                 (HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET))
#define IsBuzzerOn()                (HAL_GPIO_ReadPin(BUZZER_GPIO_Port, BUZZER_Pin) == GPIO_PIN_SET)
#define RedLedOn()                  (HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET))
#define RedLedOff()                 (HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET))
#define IsRedLedOn()                (HAL_GPIO_ReadPin(LED_RED_GPIO_Port, LED_RED_Pin) == GPIO_PIN_SET)
#define GreenLedOn()                (HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET))
#define GreenLedOff()               (HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET))
#define IsGreenLedOn()              (HAL_GPIO_ReadPin(LED_GREEN_GPIO_Port, LED_GREEN_Pin) == GPIO_PIN_SET)
#define StatusLedOn()               (1)
#define StatusLedOff()              (0)
#define IsStatusLedOn()             (0)
#define DoorlockOn()                (HAL_GPIO_WritePin(DOORLOCK_GPIO_Port, DOORLOCK_Pin, GPIO_PIN_SET))
#define DoorlockOff()               (HAL_GPIO_WritePin(DOORLOCK_GPIO_Port, DOORLOCK_Pin, GPIO_PIN_RESET))
#define IsDoorlockOn()              (HAL_GPIO_ReadPin(DOORLOCK_GPIO_Port, DOORLOCK_Pin) == GPIO_PIN_SET)
extern RTC_TimeTypeDef rtime;
extern RTC_DateTypeDef rdate;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
