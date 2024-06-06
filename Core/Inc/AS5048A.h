/*
 * AS5048A.h
 *
 *  Created on: 2022年8月9日
 *      Author: MINI
 */

#ifndef INC_AS5048A_H_
#define INC_AS5048A_H_

#define AS_NOP					0xc000
#define AS_Read_Angle   		0xFFFF
#define AS_Clear_Error_Flag		0x4001
#define AS_CS_Enable        	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define AS_CS_Disable        	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

typedef struct{
	uint16_t angle_first;	//第一次上电读到的数
	uint16_t angle_now;		//此时读到的数
	uint16_t angle_rel;		//相对初始位置转过的角度

	uint8_t angle_state;	//标志是否为第一次上电
}POS;

extern POS data_ang;

uint16_t SPI_AS5048A_ReadData(void);

uint16_t SPI_AS5048A_Relative_ReadData(POS* data);
uint16_t Angle_Process(POS* data);

void SPI_AS5048A_Origin_Value(void);
uint16_t SPI_AS5048A_Algorithm_ReadData(void);

float Angle_Circle(float Origin);
float Angle_absolute(float Origin);

uint16_t Return_Value(void);

#endif /* INC_AS5048A_H_ */
