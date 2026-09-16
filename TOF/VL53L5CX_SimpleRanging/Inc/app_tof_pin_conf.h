/**
 ******************************************************************************
 * @file    app_tof_pin_conf.h
 * @author  IMG SW Application Team
 * @brief   This file contains definitions for TOF pins
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_TOF_PIN_CONF_H__
#define __APP_TOF_PIN_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Exported symbols ----------------------------------------------------------*/
#define TOF_INT_EXTI_PIN    (GPIO_PIN_4)
#define TOF_INT_EXTI_PORT   (GPIOA)

#ifdef __cplusplus
}
#endif

#endif /* __APP_TOF_PIN_CONF_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
