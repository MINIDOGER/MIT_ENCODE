#include "spi.h"
#include "AS5048A.h"
#include "math.h"

////AS5048A编码器声明参量////BEG

/*
#define AS_NOP					0xc000
#define AS_Read_Angle   		0xffff
#define AS_Clear_Error_Flag		0x4001
#define AS_AGC					0x7ffd
#define AS_Magnitude			0x7ffe
#define AS_CS_Enable        	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define AS_CS_Disable        	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
*/
//uint16_t SPI_AS5048A_ReadData(void);
void SPI_AS5048A_Origin_Value(void);
uint16_t SPI_AS5048A_Algorithm_ReadData(void);

uint16_t Return_Value(void);

uint16_t command;
uint16_t angleValue;

uint16_t SPI_TX_DATA[4]={AS_Clear_Error_Flag, AS_NOP, AS_Read_Angle, AS_NOP};
uint16_t SPI_RX_DATA[10]={0};


const uint8_t op_num = 4;
uint16_t value_origin = 0;
uint8_t i;
uint16_t post_process_value=0;

POS data_ang;
////AS5048A编码器声明参量////END

////AS5048A编码器理想状态下数据读取////BEG
uint16_t SPI_AS5048A_ReadData(void)
{
  uint16_t angle_value;
  command = AS_Read_Angle;
  AS_CS_Enable;
  HAL_SPI_Transmit(&hspi1 ,(unsigned char *)&command ,1,100);
  AS_CS_Disable;
  AS_CS_Enable;
  HAL_SPI_TransmitReceive(&hspi1 ,(unsigned char *)&command ,(unsigned char *)&angleValue ,1 ,100);
  AS_CS_Disable;
  angle_value = angleValue & 0x3FFF;

  return angle_value;
}
////AS5048A编码器理想状态下数据读取////END

////测试：理想状态角度返回相对零点值_汪////BEG
uint16_t SPI_AS5048A_Relative_ReadData(POS* data){
  uint16_t angle_value_absolutely;
  command = AS_Read_Angle;
  AS_CS_Enable;
  HAL_SPI_Transmit(&hspi1 ,(unsigned char *)&command ,1,100);
  AS_CS_Disable;
  AS_CS_Enable;
  HAL_SPI_TransmitReceive(&hspi1 ,(unsigned char *)&command ,(unsigned char *)&angleValue ,1 ,100);
  AS_CS_Disable;
  angle_value_absolutely = angleValue & 0x3FFF;

  if(data->angle_state == 0){
	  data->angle_first = angle_value_absolutely;
	  data->angle_state = 1;
  }
  data->angle_now = angle_value_absolutely;

  return angle_value_absolutely;
}

uint16_t Angle_Process(POS* data){
	SPI_AS5048A_Relative_ReadData(data);
	if(data->angle_now < data->angle_first){
		data->angle_rel = 8192 - data->angle_first +data->angle_now;
	}
	else{
		data->angle_rel = data->angle_now - data->angle_first;
	}
	return data->angle_rel;
}
////测试：理想状态角度返回相对零点值_汪////END

////不设置零点位置，STM32芯片上电时先读取当前位置数据作为“零点”////BEG
void SPI_AS5048A_Origin_Value(void){
//	for (i = 0; i < op_num; i++){
//		AS_CS_Enable;
//		HAL_SPI_TransmitReceive(&hspi1, (unsigned char *)&SPI_TX_DATA[i], (unsigned char *)&SPI_RX_DATA[i], 1, 100);
//		AS_CS_Disable;
//		HAL_Delay(1);
//	}

	command = AS_Read_Angle;
	AS_CS_Enable;
	HAL_SPI_TransmitReceive(&hspi1 ,(unsigned char *)&command ,(unsigned char *)&SPI_RX_DATA[3] ,1 ,100);
	AS_CS_Disable;
	HAL_Delay(1);

	value_origin = SPI_RX_DATA[3] & 0x3fff;
}
uint16_t SPI_AS5048A_Algorithm_ReadData(void){
	uint16_t TXD = 0;
	command = AS_Read_Angle;

//	for (i = 1; i < op_num; i++){
//		AS_CS_Enable;
//		HAL_SPI_TransmitReceive(&hspi1, (unsigned char *)&SPI_TX_DATA[i], (unsigned char *)&SPI_RX_DATA[i], 1, 100);
//		AS_CS_Disable;
//		HAL_Delay(1);
//	}

	AS_CS_Enable;
	HAL_SPI_TransmitReceive(&hspi1 ,(unsigned char *)&command ,(unsigned char *)&TXD ,1 ,100);
	AS_CS_Disable;
	HAL_Delay(1);
//	TXD = SPI_RX_DATA[2];

	if((TXD & 0x3fff) >= value_origin){
		post_process_value = (TXD & 0x3fff) - value_origin;
	}
	else{
		post_process_value = 8192 - value_origin + (TXD & 0x3fff);
	}

	//TXD = (post_process_value&0xff00)>>8;
	//TXD = post_process_value&0x00ff;

	//return SPI_RX_DATA[3];
	return post_process_value;
	//return TXD;
}
////不设置零点位置，STM32芯片上电时先读取当前位置数据作为“零点”////END/


////角度360°转换////BEG
float Angle_Circle(float Origin){
	float Angle = 0;
	Angle = Origin/16384*360*2;
	return Angle;
}
////角度360°转换////END


////角度传感器绝对值处理////BEG
float Angle_absolute(float Origin){
	float Angle = 0;
	float Angle_C = 0;
	Angle_C = Angle_Circle(Origin);
	if(Angle_C > 180){
		Angle = -fabs(Angle_C - 360);
	}
	else{
		Angle = Angle_C;
	}
	return Angle;
}
////角度传感器绝对值处理////END/


////返回值////BEG
uint16_t Return_Value(void){
	return value_origin;
}
////返回值////END
