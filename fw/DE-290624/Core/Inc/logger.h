/**
 ******************************************************************************
 * File Name          : logger.h
 * Date               : 08/05/2016 23:15:16
 * Description        : data logger modul header
 ******************************************************************************
 *
 *
 ******************************************************************************
 */
 
 
#ifndef __LOGGER_H__
#define __LOGGER_H__					FW_BUILD	// version


/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"


/* Exported defines    -------------------------------------------------------*/
/* Exported types  -----------------------------------------------------------*/
typedef struct
{
	uint8_t log_event;
	uint8_t log_card_id[4];
	
}LOGGER_EventTypeDef;


typedef enum
{
	LOGGER_OK       = ((uint8_t)0x00U),
	LOGGER_FULL     = ((uint8_t)0x01U),
	LOGGER_EMPTY    = ((uint8_t)0x02U),
	LOGGER_ERROR    = ((uint8_t)0x03U),
	LOGGER_WRONG_ID = ((uint8_t)0x04U),
	LOGGER_BUSY     = ((uint8_t)0x05U)
	
}LOGGER_StatusTypeDef;


/* Exported variables  -------------------------------------------------------*/
extern LOGGER_EventTypeDef logger;
extern uint32_t logger_list_count;
/* Exported macros     -------------------------------------------------------*/
/* Exported functions  -------------------------------------------------------*/
extern void LOGGER_Init(void);
extern void LOGGER_Service(void);
extern LOGGER_StatusTypeDef LOGGER_Write(void);
extern LOGGER_StatusTypeDef LOGGER_Delete(void);
extern LOGGER_StatusTypeDef LOGGER_Read(uint8_t *buff);



#endif
/************************ (C) COPYRIGHT JUBERA D.O.O Sarajevo ************************/
