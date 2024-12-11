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
extern IWDG_HandleTypeDef hiwdg;
extern UART_HandleTypeDef huart1;
extern CRC_HandleTypeDef hcrc;
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
#define LED_RED_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_1
#define LED_GREEN_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_2
#define BUZZER_GPIO_Port GPIOB
#define TASTER_Pin GPIO_PIN_3
#define TASTER_GPIO_Port GPIOB
#define LED_STATUS_Pin GPIO_PIN_4
#define LED_STATUS_GPIO_Port GPIOB
#define WR_ENABLE_Pin GPIO_PIN_5
#define WR_ENABLE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define GET_SYS_FLAG        0xA0 // traži flagove sistema
#define GET_CARD_CNT        0xA1 // broj kartica u eepromu
#define GET_CARD_PRESENT    0xA2 // traži karticu u eepromu
#define GET_EVENT_CNT       0xA3 // broj događaja u eepromu
#define GET_EVENT_LAST      0xA4 // traži zadnji događaj iz eeproma

#define SET_SYS_TIME        0xB0 // podesi vrijeme sa param
#define SET_SYS_RESTART     0xB1 // softwerski restart applikacije
#define SET_CARD_ONE        0xB2 // dodaj novu karticu u eeprom
#define SET_DOOR_OPEN       0xB3 // aktiviraj bravu
#define SET_DOOR_TIME       0xB4 // podesi vrijeme brave
#define SET_DOOR_ENABLE     0xB5 // omogući kontrolu brave
#define SET_DOOR_DISABLE    0xB6 // onemogući kontrolu brave
#define SET_BUZZER_ENABLE   0xB7 // omogući buzzer
#define SET_BUZZER_DISABLE  0xB8 // onemogući buzzer

#define DELETE_CARD_ONE     0xC0 // obrisi karticu iz eeproma
#define DELETE_CARD_ALL     0xC1 // obrisi sve kartice iz eeproma
#define DELETE_EVENT_LAST   0xC2 // obrisi događaj prema datom indexu iz eeproma
#define DELETE_EVENT_ALL    0xC3 // obrisi sve događaje iz eeproma

#define RESTART_ONE         0xD0 // restartuj jednu grupu citaca
#define RESTART_ALL         0xD1 // restartuj sve citace


#define BuzzerOn()                  (HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET))
#define BuzzerOff()                 (HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET))
#define IsBuzzerOn()                (HAL_GPIO_ReadPin(BUZZER_GPIO_Port, BUZZER_Pin) == GPIO_PIN_SET)
#define RedLedOn()                  (HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET))
#define RedLedOff()                 (HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET))
#define IsRedLedOn()                (HAL_GPIO_ReadPin(LED_RED_GPIO_Port, LED_RED_Pin) == GPIO_PIN_SET)
#define GreenLedOn()                (HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET))
#define GreenLedOff()               (HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET))
#define IsGreenLedOn()              (HAL_GPIO_ReadPin(LED_GREEN_GPIO_Port, LED_GREEN_Pin) == GPIO_PIN_SET)
#define StatusLedOn()               (HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET))
#define StatusLedOff()              (HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET))
#define IsStatusLedOn()             (HAL_GPIO_ReadPin(LED_STATUS_GPIO_Port, LED_STATUS_Pin) == GPIO_PIN_SET)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
