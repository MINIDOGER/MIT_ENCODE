/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */

////定义接收数据的结构体////BEG
typedef struct
{
	float p_out;
	float v_out;
	float t_out;
}Back;

extern Back val_out;
////定义接收数据的结构体////END


/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

/* USER CODE BEGIN Prototypes */

////函数声明////BEG
HAL_StatusTypeDef CAN_SetFilters();
float Climb(float x, float k, uint32_t t);
float sin_p_n(uint32_t x, float A, float w, float fai, float B, float Hz);
float S_Trajectory_Vel_T(float x0, float x, uint32_t t, uint32_t s, float flexible, float X_stretching, float X_displacement);
float S_Trajectory_Pos_T(float x0, float x, uint32_t t, uint32_t s, float X_stretching, float X_displacement);
float sin_v_n(uint32_t x, float A, float w, float fai, float B, float Hz);

float Derivative(float x, float x0);

void SPEED_MODE(uint32_t std_id);
void POSITION_MODE(uint32_t std_id);
void EnterMotorMode(uint32_t std_id);
void ExitMotorMode(uint32_t std_id);
void ResetPoint(uint32_t std_id);
void CalibrationZeroPoint(uint32_t std_id);

void cansend(uint32_t std_id, float p, float v, float kpf, float kdf, float tff);
void canread();
////函数声明////END

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

