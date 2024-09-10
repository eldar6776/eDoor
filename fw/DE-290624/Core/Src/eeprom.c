/**
 ******************************************************************************
 * File Name          : eeprom.c
 * Date               : 28/02/2016 23:16:19
 * Description        : eeprom memory manager modul 
 ******************************************************************************
 *
 *
 *
 *
 ******************************************************************************
 */
 
#if (__EEPROM_H__ != FW_BUILD)
    #error "eeprom header version mismatch"
#endif
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "logger.h"
#include "eeprom.h"
/* Variables  ----------------------------------------------------------------*/
static uint8_t ebuf[I2CEE_PGBSZ+2];
/* Macros   ------------------------------------------------------------------*/
/* Function prototypes    ---------------------------------------------------*/
/* Program code   ------------------------------------------------------------*/

/*************************************************************************/
/**         S A V E         V A L U E       T O     E E P R O M         **/
/*************************************************************************/
void EEPROM_Save(uint16_t ee_address, uint8_t* value, uint16_t size)
{
    uint32_t frst_page_bcnt = (uint32_t)(I2CEE_PGBSZ-(ee_address%I2CEE_PGBSZ));   // number of bytes to write to first page 
    uint32_t full_page_bcnt = (uint32_t)(size/I2CEE_PGBSZ);                       // number of full pages to write
    uint32_t last_page_bcnt = (uint32_t)((ee_address+size)%I2CEE_PGBSZ);          // number of bytes to write to last page
    uint32_t wr_address = ee_address;
    /* write to first page till page boundary*/
    if (frst_page_bcnt > size) frst_page_bcnt = size;
    ebuf[0] = wr_address >> 8;
    ebuf[1] = wr_address & 0xFF;
    mem_cpy(&ebuf[2], value, frst_page_bcnt);
    HAL_I2C_Master_Transmit  (&hi2c1, EE_ADDR, ebuf, frst_page_bcnt+2, DRV_TOUT);
    if (HAL_I2C_IsDeviceReady(&hi2c1, EE_ADDR, DRV_TRIAL, DRV_TOUT) != HAL_OK) Error_Handler();
    size -= frst_page_bcnt;
    if (size == 0) return;
    /*  write full pages */
    value += frst_page_bcnt; // data pointer offset
    wr_address += frst_page_bcnt; // write address offset
    while(full_page_bcnt)
    {
        ebuf[0] = wr_address >> 8;
        ebuf[1] = wr_address & 0xFF;
        mem_cpy(&ebuf[2], value, I2CEE_PGBSZ);
        HAL_I2C_Master_Transmit  (&hi2c1,EE_ADDR, ebuf, I2CEE_PGBSZ+2, DRV_TOUT);
        if (HAL_I2C_IsDeviceReady(&hi2c1, EE_ADDR, DRV_TRIAL, DRV_TOUT)!=HAL_OK) Error_Handler();
        wr_address += I2CEE_PGBSZ; // write address offset
        value += I2CEE_PGBSZ; // data pointer offset
        --full_page_bcnt;
        size -= I2CEE_PGBSZ;
        if (size == 0) return;
    }
    /* write to last page */
    ebuf[0] = wr_address >> 8;
    ebuf[1] = wr_address & 0xFF;
    mem_cpy(&ebuf[2], value, last_page_bcnt+2);                                   
    HAL_I2C_Master_Transmit  (&hi2c1, EE_ADDR, ebuf, last_page_bcnt+2, DRV_TOUT);
    if (HAL_I2C_IsDeviceReady(&hi2c1, EE_ADDR, DRV_TRIAL, DRV_TOUT)!=HAL_OK) Error_Handler();
}

/************************ (C) COPYRIGHT JUBERA D.O.O Sarajevo ************************/
