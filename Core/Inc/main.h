/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//电路板版本1、2、3，另外新生成代码需要注意can.c文件修改宏定义
#define Board   1
//驱动板参数定义，1为达秒科技，2为叶旧电机
#define MIT   1
//左右程序定义,1为右，0为左
#define RightBoard 1

//常量定义
#define g   9.8
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
////卡尔曼滤波器参数////BEG
//typedef struct
//{
//    float LastP;//上次估算协方�? 初始化�?�为0.02
//    float Now_P;//当前估算协方�? 初始化�?�为0
//    float out;//卡尔曼滤波器输出 初始化�?�为0
//    float Kg;//卡尔曼增�? 初始化�?�为0
//    float Q;//过程噪声协方�? 初始化�?�为0.001
//    float R;//观测噪声协方�? 初始化�?�为0.543
//}Kal;//Kalman Filter parameter
////卡尔曼滤波器参数////END

//extern Kal Kalman;
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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
