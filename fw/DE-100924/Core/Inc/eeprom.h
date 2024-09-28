/**
 ******************************************************************************
 * File Name          : eeprom.h
 * Date               : 28/02/2016 23:16:19
 * Description        : eeprom memory manager modul header
 ******************************************************************************
*
* DISPLAY           pins    ->  STM32F103 Rubicon controller
* ----------------------------------------------------------------------------
* DISPLAY   +3V3    pin 1   ->  controller +3V3
* DISPLAY   GND     pin 2   ->  controller VSS
* DISPLAY   CS      pin 3   ->  PA8
* DISPLAY   RST     pin 4   ->  PA3
* DISPLAY   DC      pin 5   ->  PA2
* DISPLAY   MOSI    pin 6   ->  PA7 - SPI1 MOSI
* DISPLAY   SCK     pin 7   ->  PA5 - SPI1 SCK
* DISPLAY   LED     pin 8   ->  PB7 - PWM TIM4 CH2
* DISPLAY   MISO    pin 9   ->  PA6 - SPI1 MISO
* SD CARD   CS      pin 10  ->  PA4
*
*
******************************************************************************
*/
#ifndef __EEPROM_H__
#define __EEPROM_H__					    FW_BUILD	// version

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
/* Defines    ----------------------------------------------------------------*/
/* EEPROM hardware address and page size */
#define EE_PGSIZE                           64U
#define EE_PGNUM                            256U  // number of pages
#define EE_MAXSIZE                          0x4000U /* 64Kbit */
#define EE_ENDADDR                          0x3FFFU
#define EE_ADDR                             0xA0U
#define EETOUT                              1000U
#define EE_WR_TIME                          15U
#define EE_TRIALS                           10U
#define EE_MAX_TRIALS                       100U
#define EE_OK                               0U
#define EE_FAIL                             1U
#define EE_TOUT                             2U
#define EE_DEAULT_MARK                      0x55U

#define EE_SYS_CONFIG                       0x00U
#define EE_DOOR_TIME                        0x01U
#define EE_CARD_LIST_START_ADDR             0x100U
#define EE_CARD_LIST_END_ADDR               0xFFFU
#define EE_LOG_LIST_START_ADDR              0x1000U
#define EE_LOG_LIST_END_ADDR                EE_ENDADDR
#define I2C_EE_DENSITY 						16384U // available memory bytes
#define I2CEE_PGNUM				            256U    // number of pages
#define I2CEE_PGBSZ   			            64U    // number of bytes per page


/* Types  --------------------------------------------------------------------*/
/**
*   EEPROM FUNCTION RETURN STATUS
*/
typedef enum
{
	EE_FLASH_OK         = ((uint8_t)0x00U),
	EE_FLASH_ERROR      = ((uint8_t)0x01U),
	EE_FLASH_BUSY       = ((uint8_t)0x02U),
	EE_FLASH_TOUT    = ((uint8_t)0x03U)

} EEPROM_StatusTypeDef;
/* Variables  ----------------------------------------------------------------*/
/* Macros   ------------------------------------------------------------------*/
//#define FLASH_CS_Low()		(HAL_GPIO_WritePin(FLASH_CS_Port, FLASH_CS_Pin, GPIO_PIN_RESET))
//#define FLASH_CS_High()		(HAL_GPIO_WritePin(FLASH_CS_Port, FLASH_CS_Pin, GPIO_PIN_SET))
/* Function prototypes    ---------------------------------------------------*/
void EEPROM_Save(uint16_t ee_address, uint8_t* value, uint16_t size);
#endif
/************************ (C) COPYRIGHT JUBERA D.O.O Sarajevo ************************/
