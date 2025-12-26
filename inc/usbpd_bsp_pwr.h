/**
  ******************************************************************************
  * @file    usbpd_bsp_pwr.h
  * @author  MCD Application Team
  * @brief   This file contains bsp interface control functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

#ifndef USBPD_BSP_PWR_H
#define USBPD_BSP_PWR_H
/* Includes ------------------------------------------------------------------*/
#include "usbpd_def.h"

/* Private typedef -----------------------------------------------------------*/
/**
  * @brief  POWER Status
  */
typedef enum
{
  PWR_OK = 0,
  PWR_ERROR
} PWR_StatusTypeDef;

/* Variable containing ADC conversions results */

#if defined(USE_UNIBOARD)
/* MB1377 is used for VBUS sourcing on uniboard setup */
#define BSP_PWR_HIGH_VBUS_THRESHOLD     3800u
#define BSP_PWR_LOW_VBUS_THRESHOLD      1000u
#elif defined(MB1397)
/* STM32G474E_EVAL board is used */
#define BSP_PWR_HIGH_VBUS_THRESHOLD     2800u
#define BSP_PWR_LOW_VBUS_THRESHOLD      1000u

#else
#warning "Please ensure returned value has been computed properly according to your board setup"
#define BSP_PWR_HIGH_VBUS_THRESHOLD     390u
#define BSP_PWR_LOW_VBUS_THRESHOLD      100u
#endif

#define BSP_PWR_DISCHARGE_MARGIN        500u
#define BSP_PWR_DISCHARGE_TIME          6u

#define BSP_PWR_VBUS_5V                 5000u
#define BSP_PWR_VBUS_9V                 9000u
#define BSP_PWR_VBUS_15V                15000u

#define BSP_PWR_PRECISION               (350u)        /* DCDC output precision set to 350mV */

#define BSP_PWR_INVALID_VALUE           0xFFFFFFFFu   /* Invalid value set during issue with voltage setting */
#define BSP_PWR_TIMEOUT_PDO             250u          /* Timeout for PDO to PDO or PDO to APDO at 250ms*/
#if _PPS
#define BSP_PWR_TIMEOUT_APDO            25u           /* Timeout for APDO to APDO at 25ms*/
#endif /* _PPS */



/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void BSP_PWR_VBUSInit(uint8_t PortCount);
void BSP_PWR_VCONNInit(uint8_t PortNum, uint8_t cc);
uint32_t BSP_PWR_VBUSGetVoltage(uint8_t PortCount);
int32_t  BSP_PWR_VBUSGetCurrent(uint8_t PortCount);
int32_t  BSP_PWR_VBUSGetFilterredVoltage(uint8_t PortCount);
PWR_StatusTypeDef BSP_PWR_VBUSSetVoltage_Fixed(uint32_t PortId,
                                               uint32_t VbusTargetInmv,
                                               uint32_t OperatingCurrent,
                                               uint32_t MaxOperatingCurrent);
PWR_StatusTypeDef BSP_PWR_VBUSSetVoltage_Variable(uint32_t PortId,
                                                  uint32_t VbusTargetMinInmv,
                                                  uint32_t VbusTargetMaxInmv,
                                                  uint32_t OperatingCurrent,
                                                  uint32_t MaxOperatingCurrent);
uint32_t BSP_PWR_VBUSSetVoltage(uint8_t PortCount, uint16_t VbusInmv, uint16_t Calibration, uint16_t Precision, uint32_t Timeout);
uint32_t BSP_PWR_VBUSDischarge(uint8_t PortCount, uint32_t VbusStopInmv, uint32_t VbusErrorInmv, uint16_t DelayInms);

PWR_StatusTypeDef   BSP_PWR_VBUSOn(uint8_t PortCount);
PWR_StatusTypeDef   BSP_PWR_VBUSOff(uint8_t PortCount);
uint8_t BSP_PWR_VBUSIsOn(uint8_t PortCount);
PWR_StatusTypeDef   BSP_PWR_VCONNOn(uint8_t PortCount, uint8_t cc);
PWR_StatusTypeDef   BSP_PWR_VCONNOff(uint8_t PortCount, uint8_t cc);
uint32_t BSP_PWR_VCONNGetVoltage(uint8_t PortCount);
uint32_t BSP_PWR_VCONNGetCurrent(uint8_t PortCount);


#endif /* USBPD_BSP_HW_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

