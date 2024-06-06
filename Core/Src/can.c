/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "main.h"
#include <stdio.h>
#include "math.h"

#define PI acos(-1)

//#if (Board == 1 || Board == 2)
//	#define CANRx  GPIO_PIN_8
//	#define CANTx  GPIO_PIN_9
//	#define GPIO	GPIOB
//#else
//	#define CANRx  GPIO_PIN_11
//	#define CANTx  GPIO_PIN_12
//	#define GPIO	GPIOA
//#endif

#define CANRx  GPIO_PIN_8
#define CANTx  GPIO_PIN_9
#define GPIO	GPIOB

//#define CANRx  GPIO_PIN_11
//#define CANTx  GPIO_PIN_12
//#define GPIO	GPIOA
////定义参量////BEG

//#if (MIT == 1)
//	#define P_MIN   -25.5f//叶4pi,原4pi
//	#define P_MAX   25.5f
//	#define V_MIN   -45.0f//叶45,原30
//	#define V_MAX   45.0f
//	#define KP_MIN  0.0f
//	#define KP_MAX  500.0f//叶100,原500
//	#define KD_MIN  0.0f
//	#define KD_MAX  5.0f//叶5,原100
//	#define T_MIN   -18.0f
//	#define T_MAX   18.0f
//#else
//	#define P_MIN   -25.12f//叶4pi,原4pi
//	#define P_MAX   25.12f
//	#define V_MIN   -45.0f//叶45,原30
//	#define V_MAX   45.0f
//	#define KP_MIN  0.0f
//	#define KP_MAX  100.0f//叶100,原500
//	#define KD_MIN  0.0f
//	#define KD_MAX  5.0f//叶5,原100
//	#define T_MIN   -18.0f
//	#define T_MAX   18.0f
//#endif

#define P_MIN   -25.5f//叶4pi,原4pi
#define P_MAX   25.5f
#define V_MIN   -45.0f//叶45,原30
#define V_MAX   45.0f
#define KP_MIN  0.0f
#define KP_MAX  500.0f//叶100,原500
#define KD_MIN  0.0f
#define KD_MAX  5.0f//叶5,原100
#define T_MIN   -18.0f
#define T_MAX   18.0f

//测试结果：
//叶旧电机2022-8-23：与驱动板参数不同，源参数位置差2倍，速度较快，8pi，45较为合适，但没有精确测量
//U8电机2022-8-25：标定时电流过大在23v左右，电流约2A~2.5A电压12v左右，电流约1.5A电压8V左右，电流约1A；空载情况下，速度模式，kv参数大于14，出现问题，旋转准确性与旧电机差不多

//叶中旧电机及U8空载适用参数//
//-25.12到25.12，-45到45，0到500，0到100（叶），-18到18
//达妙科技参数//
//-12.5到12.5，-45到45，0到500，0到5，-18到18


//#define CAN_RX_FIFO0  (0X00000000U)
//#define CAN_RX_FIFO1  (0X00000001U)
////定义参量////END

////S_Trajectory_Pos()使用作为差�??////BEG
float y_error_p = 0;
////S_Trajectory_Pos()使用作为差�??////END


////声明结构体////BEG
Back val_out;
////声明结构体////END

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  //MIT:833(842) 达妙:931
  //1、2版CAN1引脚PB8 RX，PB9 TX，3版CAN1引脚PA11 RX，PA12 TX
  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 9;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 9;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = CANRx|CANTx;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIO, &GPIO_InitStruct);

  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIO, CANRx|CANTx);

  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_RX1_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */


////设定CAN过滤器，全部接收////BEG
HAL_StatusTypeDef CAN_SetFilters()
{
  CAN_FilterTypeDef canFilter;
  canFilter.FilterBank = 0;
  canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilter.FilterScale = CAN_FILTERSCALE_32BIT;

  canFilter.FilterIdHigh = 0x0000;
  canFilter.FilterIdLow = 0x0000;
  canFilter.FilterMaskIdHigh = 0x0000;
  canFilter.FilterMaskIdLow = 0x0000;

  canFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
  canFilter.FilterActivation = ENABLE;
  canFilter.SlaveStartFilterBank = 14;
  HAL_StatusTypeDef result = HAL_CAN_ConfigFilter(&hcan1, &canFilter);
  return result;

}
////设定CAN过滤器，全部接收////END


////取较大�??////BEG
float fmaxf(float x, float y){
    /// Returns maximum of x, y ///
    return (((x)>(y))?(x):(y));
    }
////取较大�??////END


////取较小�??////BEG
float fminf(float x, float y){
    /// Returns minimum of x, y ///
    return (((x)<(y))?(x):(y));
    }
////取较小�??////END


////测试：p位置爬升计算////BEG
float Climb(float x, float k, uint32_t t){
	float y = 0;
	if(k >0){
		y = fminf(0.01*k*t, x);
	}
	else if(k < 0){
		y = fmaxf(0.01*k*t, x);
	}

	return y;
}
////测试：p位置爬升计算////END

////测试：p位置sin规划////BEG
float sin_p_n(uint32_t x, float A, float w, float fai, float B, float Hz){
	float y = 0;
	if(Hz == 0){
		//y = A*sin(w*x*0.01 + fai) + B;
		y = A*sin(w*x*0.00005 + fai) + B; //乐聚关节性能测试�?20221109
	}
	else{
		//y = A*sin(Hz*2*PI*x*0.01 + fai) + B;
		y = A*sin(Hz*2*PI*x*0.00005 + fai) + B; //乐聚关节性能测试�?20221109
	}
	return y;
}
////测试：p位置sin规划////END


////测试：S曲线速度计算(t时间�?始减�?)////BEG
float S_Trajectory_Vel_T(float x0, float x, uint32_t t, uint32_t s, float flexible, float X_stretching, float X_displacement){

	//y = x0+(x-x0)/(1+e^(flexible*(t*a-b)))
	//x0:调节纵轴平移，初�?
	//x:调节纵轴拉伸，终�?
	//t:自变量，时间
	//s:自变量，持续时间
	//flexible:横轴整体改变,<0
	//X_stretching:调节横轴拉伸,a
	//X_stretching:调节横轴平移,b

	float yv = 0;
	if(s != 0){
		if(t*0.01 <= s){
			float x_error = 0;
			float index = 0;
			x_error = x - x0;
			index = flexible*(0.01*t*X_stretching - X_displacement);//0.01是转换为�?
			yv = x0 + x_error/(1 + exp(index));
		}
		else if(t*0.01 > s){
			float x_error = 0;
			float index = 0;
			x_error = x0 - x;
			index = flexible*((0.01*t-s)*X_stretching - X_displacement);//0.01是转换为�?
			yv = x + x_error/(1 + exp(index));
		}
	}
	else if(s == 0){
		float x_error = 0;
		float index = 0;
		x_error = x - x0;
		index = flexible*(0.01*t*X_stretching - X_displacement);//0.01是转换为�?
		yv = x0 + x_error/(1 + exp(index));
	}
	return yv;
}
////测试：S曲线速度计算(t时间�?始减�?)////END


////测试：S曲线位置计算////BEG
float S_Trajectory_Pos_T(float x0, float x, uint32_t t, uint32_t s, float X_stretching, float X_displacement){
	float yp = 0;
	if(0.01*t <= s){
		float x_error_p = 0;
		float logarithm_p = 0;
		x_error_p = x - x0;
		logarithm_p = exp((0.01*t)*X_stretching) + exp(X_displacement);
		yp = x*0.01*t - x_error_p*log(logarithm_p)/X_stretching;
		y_error_p = yp;
	}
	else if(0.01*t > s){
		float x_error_p = 0;
		float logarithm_p = 0;
		x_error_p = x0 - x;
		logarithm_p = exp((0.01*t-s)*X_stretching) + exp(X_displacement);
		yp = y_error_p + x0*(0.01*t-s) - x_error_p*log(logarithm_p)/X_stretching;
	}
	return yp;
}
////测试：S曲线位置计算////END

////测试：sin速度规划////BEG
float sin_v_n(uint32_t x, float A, float w, float fai, float B, float Hz){
	float y = 0;
	if(Hz == 0){
		y = A*sin(w*x*0.01 + fai) + B;
	}
	else{
		y = A*sin(Hz*2*3.1415926*x*0.01 + fai) + B;
	}
	return y;
}
////测试：sin速度规划////END

////测试：求�?////BEG
float Derivative(float x, float x0){
	//1毫秒
	float y;
	y = (x - x0)/0.01;
	return y;
}
////测试：求�?////END


////float转unit，应该是浮点型转整型////BEG
int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }
////float转unit，应该是浮点型转整型////END


////unit转float，应该是整型转浮点型////BEG
float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }
////unit转float，应该是整型转浮点型////END/


////进入速度模式指令////BEG 驱动板已删除
void SPEED_MODE(uint32_t std_id){
	uint8_t TxData[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFA};
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId=std_id;
	TxHeader.TransmitGlobalTime = DISABLE;
	TxHeader.DLC = 8;

  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {
    /* Transmission request Error */
    Error_Handler();
  }

  //if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  //{
    /* Transmission request Error */
    //Error_Handler();
  //}
}
////进入速度模式指令////END


////进入位置模式指令////BEG 驱动板已删除
void POSITION_MODE(uint32_t std_id){
	uint8_t TxData[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFB};
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId=std_id;
	TxHeader.TransmitGlobalTime = DISABLE;
	TxHeader.DLC = 8;

  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {
    /* Transmission request Error */
    Error_Handler();
  }

  //if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  //{
    /* Transmission request Error */
    //Error_Handler();
  //}
}
////进入位置模式指令////END


////进入电机模式指令////BEG
void EnterMotorMode(uint32_t std_id){
	uint8_t TxData[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = std_id;
	TxHeader.TransmitGlobalTime = DISABLE;
	TxHeader.DLC = 8;

  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {
    /* Transmission request Error */
    Error_Handler();
    printf("error_EnterMotorMode\r\n");
  }

  //if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  //{
    /* Transmission request Error */
    //Error_Handler();
  //}
}
////进入电机模式指令////END

////�?出电机模式指�?////BEG
void ExitMotorMode(uint32_t std_id){
	uint8_t TxData[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId=std_id;
	TxHeader.TransmitGlobalTime = DISABLE;
	TxHeader.DLC = 8;

  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {
    /* Transmission request Error */
    Error_Handler();
    printf("error_ExitMotorMode\r\n");
  }

  //if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  //{
    /* Transmission request Error */
    //Error_Handler();
  //}
}
////�?出电机模式指�?////END

////恢复位置圈数指令////BEG 修改版中使用,驱动板有问题，不建议使用
void ResetPoint(uint32_t std_id){
	uint8_t TxData[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId=std_id;
	TxHeader.TransmitGlobalTime = DISABLE;
	TxHeader.DLC = 8;

  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {
    /* Transmission request Error */
    Error_Handler();
  }

  //if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  //{
    /* Transmission request Error */
    //Error_Handler();
  //}
}
////恢复位置圈数指令////END

////标定零点位置指令////BEG 修改版中使用,驱动板有问题，不建议使用
void CalibrationZeroPoint(uint32_t std_id){
	uint8_t TxData[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId=std_id;
	TxHeader.TransmitGlobalTime = DISABLE;
	TxHeader.DLC = 8;

  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {
    /* Transmission request Error */
    Error_Handler();
  }

  //if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  //{
    /* Transmission request Error */
    //Error_Handler();
  //}
}
////标定零点位置指令////END

////CAN数据帧发送函函数////BEG
void cansend(uint32_t std_id, float p, float v, float kpf, float kdf, float tff)
{
  CAN_TxHeaderTypeDef TxHeader;
  uint8_t TxData[8];
  uint32_t TxMailbox;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;            
  TxHeader.StdId = std_id;
  TxHeader.TransmitGlobalTime = DISABLE;
  TxHeader.DLC = 8;

  /// limit data to be within bounds///
  float p_des = fminf(fmaxf(P_MIN, p), P_MAX);
  float v_des = fminf(fmaxf(V_MIN, v), V_MAX);
  float kp = fminf(fmaxf(KP_MIN, kpf), KP_MAX);
  float kd = fminf(fmaxf(KD_MIN, kdf), KD_MAX);
  float t_ff = fminf(fmaxf(T_MIN, tff), T_MAX);
  /// convert floats to unsigned ints///
  uint16_t p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  uint16_t v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  uint16_t kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  uint16_t kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  uint16_t t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
  /// pack ints into the can buffer///
  TxData[0] = p_int>>8;
  TxData[1] = p_int&0xFF;
  TxData[2] = v_int>>4;
  TxData[3] = ((v_int&0xF)<<4)|(kp_int>>8);
  TxData[4] = kp_int&0xFF;
  TxData[5] = kd_int>>4;
  TxData[6] = ((kd_int&0xF)<<4)|(t_int>>8);
  TxData[7] = t_int&0xff;

  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {
    /* Transmission request Error */
	//HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
	  printf("error\r\n");
	  //cansend(01, 1, 2, 3, 4, 5);
      Error_Handler();
  }

  //if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  //{
    /* Transmission request Error */
    //Error_Handler();
  //}
}
////CAN数据帧发送函数////END


////CAN数据帧读取函数////BEG
void canread()
{
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[6];

  /*RxHeader.RTR = CAN_RTR_DATA;
  RxHeader.IDE = CAN_ID_STD;            
  RxHeader.StdId=std_id;
  RxHeader.DLC = 8;*/

  if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {

  }

  uint16_t id = RxData[0];
  uint16_t p_int = (RxData[1]<<8)|RxData[2];
  uint16_t v_int = (RxData[3]<<4)|(RxData[4]>>4);
  uint16_t i_int = ((RxData[4]&0xF)<<8)|RxData[5];
  /// convert uints to floats ///
  float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
  float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
  float t = uint_to_float(i_int, -T_MAX, T_MAX, 12);

  val_out.p_out = p;
  val_out.v_out = v;
  val_out.t_out = t;

  //return val;
}
////CAN数据帧读取函函数////END


/* USER CODE END 1 */
