/**
 ******************************************************************************
 * File Name          : logger.c
 * Date               : 28/02/2016 23:16:19
 * Description        : data logger software modul
 ******************************************************************************
 *
 *
 ******************************************************************************
 */

#if (__LOGGER_H__ != FW_BUILD)
    #error "logger header version mismatch"
#endif
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "logger.h"
#include "eeprom.h"
/* Imported Types  -----------------------------------------------------------*/
LOGGER_EventTypeDef logger;
/* Imported Variables --------------------------------------------------------*/
/* Imported Functions    -----------------------------------------------------*/
/* Private Variables  --------------------------------------------------------*/
static uint32_t logger_next_log_address;
static uint32_t logger_next_log_id;
static uint32_t logger_tmr;
uint32_t logger_list_count;
/* Private Macros    ---------------------------------------------------------*/
/* Private Prototypes    -----------------------------------------------------*/
/* Program code   ------------------------------------------------------------*/
void LOGGER_Service(void)
{
    // timer nakon čitanja zadnjeg loga blokira čitač 
    // i omogućava brisanje zadnjeg loga nakon čega
    // je brisanje onemogućeno
    if(logger_tmr) 
    {
        RfidDisable();
        if((HAL_GetTick() - logger_tmr) >= 100U) 
        {
            RfidEnable();
            logger_tmr = 0;
        }
    }
}
/**
  * @brief
  * @param
  * @retval
  */
void LOGGER_Init(void)
{
    uint8_t log_buff[LOG_DSIZE];
    SYS_LogListFullReset();
    SYS_NewLogReset();
    logger_tmr = 0;
    logger_list_count = 0U;
    logger_next_log_id = 1U;
    mem_set(&logger, 0U, sizeof(logger));
    log_buff[0] = (EE_LOG_LIST_START_ADDR >> 8U);
    log_buff[1] = (EE_LOG_LIST_START_ADDR & 0xFFU);
    logger_next_log_address = EE_LOG_LIST_START_ADDR;
    if(HAL_I2C_IsDeviceReady(&hi2c1, EE_ADDR, DRV_TRIAL, DRV_TOUT)              != HAL_OK)  Error_Handler();
    if(HAL_I2C_Master_Transmit(&hi2c1, EE_ADDR, log_buff, 2U, DRV_TOUT)         != HAL_OK)  Error_Handler();
    if(HAL_I2C_Master_Receive(&hi2c1, EE_ADDR, log_buff, LOG_DSIZE, DRV_TOUT)   != HAL_OK)  Error_Handler();
    
    while(logger_next_log_address <= (EE_LOG_LIST_END_ADDR - LOG_DSIZE))
    {	
        if((log_buff[0] == 0U) && (log_buff[1] == 0U)) break;
        logger_next_log_id = ((log_buff[0] << 8U) | log_buff[1]);
        ++logger_next_log_id;
        ++logger_list_count;
        logger_next_log_address += LOG_DSIZE;
        log_buff[0] = logger_next_log_address >> 8U;
        log_buff[1] = logger_next_log_address & 0xFFU;
        if(HAL_I2C_Master_Transmit(&hi2c1, EE_ADDR, log_buff, 2U, DRV_TOUT)       != HAL_OK)  Error_Handler();
        if(HAL_I2C_Master_Receive(&hi2c1, EE_ADDR, log_buff, LOG_DSIZE, DRV_TOUT) != HAL_OK)  Error_Handler();
    }
    /**
    *	set log list not empty 	-> system status flag
    *	set log list full  		-> system status flag
    */
    if(logger_list_count != 0U) SYS_NewLogSet();
    if(logger_next_log_address > (EE_LOG_LIST_END_ADDR - LOG_DSIZE)) SYS_LogListFullSet();
}
/**
  * @brief
  * @param
  * @retval
  */
LOGGER_StatusTypeDef LOGGER_Write(void)
{
    uint8_t log_buff[LOG_DSIZE+2];
    
    HAL_RTC_GetTime(&hrtc, &rtime, RTC_FORMAT_BCD); // should be only time update
    HAL_RTC_GetDate(&hrtc, &rdate, RTC_FORMAT_BCD); // should be only date update

	if(logger_next_log_address > (EE_LOG_LIST_END_ADDR - LOG_DSIZE))
	{
		SYS_LogListFullSet();
		return (LOGGER_FULL);
	}
    log_buff[0] = logger_next_log_address >> 8u;
    log_buff[1] = logger_next_log_address & 0xFFU;
    log_buff[2] = logger_next_log_id >> 8U;
    log_buff[3] = logger_next_log_id & 0xFFU;
    log_buff[4] = logger.log_event;
    log_buff[5] = logger.log_card_id[0];
    log_buff[6] = logger.log_card_id[1];
    log_buff[7] = logger.log_card_id[2];
    log_buff[8] = logger.log_card_id[3];
    log_buff[9] = rdate.Date;
    log_buff[10]= rdate.Month;
    log_buff[11]= rdate.Year;
    log_buff[12]= rtime.Hours;
    log_buff[13]= rtime.Minutes;
    log_buff[14]= rtime.Seconds;
    mem_set(&logger, 0U, sizeof(logger));
    EE_EnableWR();
    if (HAL_I2C_IsDeviceReady(&hi2c1, EE_ADDR, DRV_TRIAL, DRV_TOUT) != HAL_OK)              Error_Handler();
    if (HAL_I2C_Master_Transmit(&hi2c1, EE_ADDR, log_buff, LOG_DSIZE+2, DRV_TOUT) != HAL_OK)Error_Handler();
    if (HAL_I2C_IsDeviceReady(&hi2c1, EE_ADDR, DRV_TRIAL, DRV_TOUT) != HAL_OK)              Error_Handler();
    EE_DisableWR();
    logger_next_log_address += LOG_DSIZE;
    ++logger_list_count;
    ++logger_next_log_id;
    if(logger_next_log_address > (EE_LOG_LIST_END_ADDR - LOG_DSIZE))
	{
		SYS_LogListFullSet();
		return (LOGGER_FULL);
	}
    SYS_NewLogSet();
    return (LOGGER_OK);
}
/**
  * @brief
  * @param
  * @retval
  */
LOGGER_StatusTypeDef LOGGER_Read(uint8_t *buff)
{
    uint8_t log_buff[2];
    logger_tmr = HAL_GetTick();
	if (logger_list_count == 0U) return(LOGGER_EMPTY);
    log_buff[0] = ((logger_next_log_address - LOG_DSIZE) >> 8);
    log_buff[1] = ((logger_next_log_address - LOG_DSIZE) & 0xFFU);
    if (HAL_I2C_IsDeviceReady(&hi2c1, EE_ADDR, DRV_TRIAL, DRV_TOUT)       != HAL_OK)  Error_Handler();
    if (HAL_I2C_Master_Transmit(&hi2c1, EE_ADDR, log_buff, 2U, DRV_TOUT)  != HAL_OK)  Error_Handler();
    if (HAL_I2C_Master_Receive(&hi2c1, EE_ADDR, buff, LOG_DSIZE, DRV_TOUT)!= HAL_OK)  Error_Handler();	
    return(LOGGER_OK);
}
/**
  * @brief
  * @param
  * @retval
  */
LOGGER_StatusTypeDef LOGGER_Delete(void)
{
    uint8_t log_buff[LOG_DSIZE+2];
    if(!logger_tmr) return(LOGGER_ERROR);// brisanje zadnjeg loga moguće jedino unutar 100 ms nakon čitanja zadnjeg loga
	if (logger_list_count == 0U) return(LOGGER_EMPTY);
    mem_set(log_buff, 0U, sizeof(log_buff));
    log_buff[0] = ((logger_next_log_address - LOG_DSIZE) >> 8U);
    log_buff[1] = ((logger_next_log_address - LOG_DSIZE) & 0xFFU);
    EE_EnableWR();
    if (HAL_I2C_IsDeviceReady(&hi2c1, EE_ADDR, DRV_TRIAL, DRV_TOUT) != HAL_OK)              Error_Handler();
    if (HAL_I2C_Master_Transmit(&hi2c1, EE_ADDR, log_buff, LOG_DSIZE+2, DRV_TOUT) != HAL_OK)Error_Handler();
    if (HAL_I2C_IsDeviceReady(&hi2c1, EE_ADDR, DRV_TRIAL, DRV_TOUT) != HAL_OK)              Error_Handler();	
    EE_DisableWR();
    logger_next_log_address -= LOG_DSIZE;
    --logger_list_count;
    if (logger_list_count == 0U) SYS_NewLogReset();
    SYS_LogListFullReset();
    return (LOGGER_OK);
}
/************************ (C) COPYRIGHT JUBERA D.O.O Sarajevo ************************/
