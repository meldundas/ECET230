/**
 ******************************************************************************
 * @file    lsm6dsl_settings.h
 * @author  MEMS Application Team
 * @version V7.0.0
 * @date    24-June-2019
 * @brief   This file contains definitions for the LSM6DSL settings
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#ifndef __LSM6DSL_SETTINGS_H__
#define __LSM6DSL_SETTINGS_H__

#ifdef __cplusplus
extern "C" {
#endif

#define LSM6DSL_ACC_ODR 104.0f /* ODR = 104Hz */

#define LSM6DSL_ACC_FS 2 /* FS = 2g */

#define LSM6DSL_GYRO_ODR 104.0f /* ODR = 104Hz */

#define LSM6DSL_GYRO_FS 2000 /* FS = 2000dps */

#ifdef __cplusplus
}
#endif

#endif /* __LSM6DSL_SETTINGS_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
