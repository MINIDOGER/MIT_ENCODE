/* USER CODE BEGIN Header */

//最后更新日期及平台：
//...
//2022-08-12 Linux
//2022-08-12 Windows
//2022-08-14 Windows
//2022-08-15 Linux b
//2022-08-16 Windows s
//2022-08-17 Windows b
//2022-08-18 Windows m
//2022-08-24 Windows s
//2022-08-27 Windows m
//2022-09-05 Windows m
//2022-09-29 Windows s
//2022-10-08 Windows s
//2022-10-20 Windows s
//2022-10-26 Windows s
//2022-11-01 Windows s
//2022-11-06 Windows s
//2022-11-10 Windows s
//2023-03-23 Windows m 增加对串口1的中断支持，暂用于ESP32通信，迭代版功能，旧版不支持，新代码导致远古中文注释部分乱码
//2023-03-30 Windows b 增加6串口中断，额外留出I2C，SPI，恢复部分中文注释
//2023-04-13 Windows b MPU6050模块数据解析函数封装，未测试，增加电路板版本辨识定义Board，增加达秒和旧电机辨识定义MIT，恢复部分中文注释
//2023-04-16 Windows b 增加MPU6050模块数据采集函数封装，未测试，增加拟合函数封装，但未定义足够参数，未测试，恢复并增加部分中文注释,注释基本全部恢复(除过时注释或遗漏)
//2023-04-16 Windows s 注释掉之前有关倒立摆的代码
//2023-04-16 Windows m MPU6050模块数据解析函数封装暂不可用，暂改为单独粘贴修改
//2023-05-05 Windows s 修改电路板版本导致串口中断问题
//2023-05-06 Windows s 增加三串口数据采集测试
//2023-05-15 Windows m 修改Tim1的触发规则，未测试，修改部分参数命名，尝试增加拟合数据缓存，完善注释
//2023-05-26 Windows n 功能杂，维护难，课题内容新开，芯片1：ArtificialLimb_Chip1，芯片2：ArtificialLimb_Chip2

//计划及其重要程度2023-05-21：
//1.MPU6050模块解析数据函数封装**
//2.角度数据拟合*****尝试固定轨迹后编写
//  a.行走轨迹
//  b.越障轨迹
//  c.滑倒轨迹
//3.物理开关检测***

/*
//                            _ooOoo_
//                           o8888888o
//                           88" . "88
//                           (| -_- |)
//                            O\ = /O
//                        ____/`---'\____
//                      .   ' \\| |// `.
//                       / \\||| : |||// \
//                     / _||||| -:- |||||- \
//                       | | \\\ - /// | |
//                     | \_| ''\---/'' | |
//                      \ .-\__ `-` ___/-. /
//                   ___`. .' /--.--\ `. . __
//                ."" '< `.___\_<|>_/___.' >'"".
//               | | : `- \`.;`\ _ /`;.`/ - ` : | |
//                 \ \ `-. \_ __\ /__ _/ .-` / /
//         ======`-.____`-.___\_____/___.-`____.-'======
//                            `=---='
//
//         .............................................
//                  佛祖保佑   芯片不烧   永无BUG
//          佛曰:
//                  写字楼里写字间，写字间里程序员；
//                  程序人员写程序，又拿程序换酒钱。
//                  酒醒只在网上坐，酒醉还来网下眠；
//                  酒醉酒醒日复日，网上网下年复年。
//                  但愿老死电脑间，不愿鞠躬老板前；
//                  奔驰宝马贵者趣，公交自行程序员。
//                  别人笑我忒疯癫，我笑自己命太贱；
//                  不见满街漂亮妹，哪个归得程序员？
*/

/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "can.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>

#include "stm32f4xx_hal_pwr_ex.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_rcc_ex.h"
#include "stm32f4xx_hal_flash_ex.h"

#include "AS5048A.h"
#include "math.h"
#include "Kalman_Filter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
////移至main.h文件//------------------------------------------------------------------//
////电路板版本1、2、3，另外注意修改can.c文件CAN1引脚
//#define Board   1

////常量定义
//#define g   9.8
////移至main.h文件//------------------------------------------------------------------//
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

////声明参量------------------------------------------------------------------////BEG
#pragma region
//CAN通信参数
uint32_t CANID = 0x00;
float p = 0, v = 0, kp = 0, kv = 0, tr = 0;
float PI = 3.1415926;

float k = 1;//爬坡计算斜率参数

float v0 = 0, v1 = 3.14, s_t = 0, c_v = -1, a_v = 5, b_v = 7;//S型速度曲线规划计算参数
float v0_p = 0, v1_p = 3, a_p = -5, b_p = -5;//S型速度度积分得位置规划计算参数
float A_sin = 1, w_sin = 1, f_sin = 0, B_sin = 0, Hz_sin = 0;//sin速度规划计算参数

uint8_t Tim1Flag = 0;
uint16_t ss = 0;
uint8_t s = 0, m = 0, h = 0;
uint16_t us = 0; //乐聚关节测试20221109

float dy = 0;
float dp = 0;

uint8_t p_compare = 0;//用于位置模式测试判断参数
uint8_t p_com_delay = 10;//秒
//uint8_t v_compare = 0;//用于速度模式测试判断参数
//uint8_t v_com_delay = 6;//秒
//uint32_t ss_error = 0;
uint8_t t_com = 0;
float MotorMode = 0;//用于判断电机模式的开启关闭
uint8_t Flag =0 ;//用于判断是否第一次进入电机模式
float Mode = 0;//用于速度位置模式改变
float Mode_p = 0;//用于p变化模式判断
float Mode_v = 0;//用于v变化模式判断
float Mode_Ang = 1;//用于判断角度传感器模式
float canid = 0;//用于暂存CANID的数值，float转换uint32_t
float p_crx = 0;//用于暂存p值，爬坡计算使用
float v_crx = 0;//用于暂存v值

float serial_send = 0;//用于串口调试信息分类

float Angle;
float Angle_Origin;//编码器原始数据

//字符缓存
uint8_t cRx_1;
uint8_t cRx_2;
uint8_t cRx_3;
uint8_t cRx_4;
uint8_t cRx_5;
uint8_t cRx_6;

/*
uint16_t Test_Value;//查看某个过程数据暂存变量

float KA = 0, KE = 0, KS = 0, Angle_old = 0;//PID稳摆瞎测试
float KI = 0, KD = 0, KP = 0, error = 0, error_old = 0, error_sum = 0;//PID稳摆瞎测试

float v_origin = 0, v_time = 50;//PID起摆瞎测试
int count = 0;
uint8_t state = 0;

float M = 0, B = 0, K = 0;//阻抗控制单关节参数
float x_old = 0, xDer_old = 0, xSDer_old = 0;

float Kalman_v = 0;//卡尔曼输出参数
*/


//MPU6050模块数据采集参数
struct TranBuf{
	uint8_t rxDataAcc[11]; //加速度原始数据存放
	uint8_t rxDataAngAcc[11]; //角加速度原始数据存放
	uint8_t rxDataAng[11]; //角度原始数据存放

	uint16_t Accx;
	uint16_t Accy;
	uint16_t Accz; //加速度缓存
	uint16_t AngAccx;
	uint16_t AngAccy;
	uint16_t AngAccz; //角加速度缓存
	uint16_t Angx;
	uint16_t Angy;
	uint16_t Angz; //角度缓存
}; //数据缓存

struct StateBuf{
	uint8_t counter; //计数
	uint8_t sum; //合
	uint8_t Flag; //标志
}; //状态统计

struct DataFit{
	float FitBuf1[100];
	float FitBuf2[100];
	uint8_t FitFlag1;
	uint8_t FitFlag2;
	int sizenum1;
	int sizenum2;
}; //拟合数据缓存与统计



struct Data{
	float Accx;
	float Accy;
	float Accz; //加速度
	float AngAccx;
	float AngAccy;
	float AngAccz; //角加速度
	float Angx;
	float Angy;
	float Angz; //角度
	struct TranBuf Buf; //状态
	struct StateBuf State; //缓存
	struct DataFit Fit;
}; //关节各个数据

struct joint{
	struct Data Hip;
	struct Data Knee;
	struct Data Ankle;
}; //各个关节

struct joint Right; //右关节数据组

#pragma endregion
////声明参量------------------------------------------------------------------////END

////通用函数声明------------------------------------------------------------------////BEG
#pragma region
int float_to_uint(float x, float x_min, float x_max, int bits);

void MPU6050ModDataBuf(struct Data AllData, uint8_t cRx);
void MPU6050ModData(struct Data AllData);

void polyfit(int n, double *x, double *y, int poly_n, double p[]);
void gauss_solve(int n,double A[],double x[],double b[]);
#pragma endregion
////通用函数声明------------------------------------------------------------------////END
/*/---------------------------------2022.8.9转移至AS5048A.c---------------------------------------//BEG
 *
////AS5048A编码器声明参数////BEG
#define AS_NOP					0xc000
#define AS_Read_Angle   		0xFFFF
#define AS_Clear_Error_Flag		0x4001
#define AS_CS_Enable        	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define AS_CS_Disable        	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

uint16_t SPI_AS5048A_ReadData(void);
void SPI_AS5048A_Origin_Value(void);
uint16_t SPI_AS5048A_Algorithm_ReadData(void);

uint16_t command;
uint16_t angleValue;

uint16_t SPI_TX_DATA[10]={0};
uint16_t SPI_RX_DATA[10]={0};

//函数参量定义
const uint8_t op_num = 4;
uint16_t value_origin = 0;
uint8_t i;
uint16_t post_process_value=0;
////AS5048A编码器声明参数////END
 *
//---------------------------------2022.8.9转移至AS5048A.c---------------------------------------//END*/

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */


	/*///测试：理想状态角度返回相对零点位置_汪part0------------------------------------------------------------------////BEG
	data_ang.angle_first = 0;
	data_ang.angle_now = 0;
	data_ang.angle_rel = 0;
	data_ang.angle_state = 0;
	////测试：理想状态角度返回相对零点位置_汪part0------------------------------------------------------------------////END*/


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_CAN2_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */


	////初始化printf，TIM2中断，CAN过滤器，CAN1打开------------------------------------------------------------------////BEG
	setvbuf(stdout, NULL, _IONBF, 0); //输出缓冲

	//HAL_TIM_Base_Start_IT(&htim2);
	//HAL_TIM_Base_Stop_IT(&htim2);


	if(HAL_UART_Receive_IT(&huart2, &cRx_2, 1) != HAL_OK){
		Error_Handler();
	}
	if(Board == 2 || Board == 3){
		if(HAL_UART_Receive_IT(&huart1, &cRx_1, 1) != HAL_OK){
			Error_Handler();
		}
	}
	if(Board == 3){
		if(HAL_UART_Receive_IT(&huart3, &cRx_3, 1) != HAL_OK){
			Error_Handler();
		}
		if(HAL_UART_Receive_IT(&huart4, &cRx_4, 1) != HAL_OK){
			Error_Handler();
		}
		if(HAL_UART_Receive_IT(&huart5, &cRx_5, 1) != HAL_OK){
			Error_Handler();
		}
		if(HAL_UART_Receive_IT(&huart6, &cRx_6, 1) != HAL_OK){
			Error_Handler();
		}
	}

	CAN_SetFilters();
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO1_MSG_PENDING);
	//HAL_CAN_Start(&hcan2);

	////初始化printf，TIM2中断，CAN过滤器，CAN1打开------------------------------------------------------------------////END


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


	/*///进入电机模式MIT------------------------------------------------------------------////BEG
	EnterMotorMode(0x00);
	HAL_CAN_Stop(&hcan1);
	//HAL_CAN_Stop(&hcan2);
	////进入电机模式MIT------------------------------------------------------------------////END*/


	/*////进入速度或位置模式------------------------------------------------------------------////BEG
	SPEED_MODE(0x00);
	//POSITION_MODE(0x00);
	HAL_CAN_Stop(&hcan1);
	//HAL_CAN_Stop(&hcan2);
	////进入速度或位置模式------------------------------------------------------------------////END*/


	/*///测试：理想状态角度返回相对零点位置_汪part1------------------------------------------------------------------////BEG
	SPI_AS5048A_Relative_ReadData(&data_ang);
	////测试：理想状态角度返回相对零点位置_汪part1------------------------------------------------------------------////BEG*/


	/*///通电“零点相对角度返回值part1------------------------------------------------------------------////BEG/
	SPI_AS5048A_Origin_Value();
	////通电“零点相对角度返回值part1------------------------------------------------------------------////END*/


	HAL_Delay(1);


	////参数初始化------------------------------------------------------------------////BEG
	CANID = 0x01;
	p = 6.28;
	v = 0;
	kp = 2;
	kv = 6;
	tr = 0;

	v0 = 0;
	v1 = 3.14;
	c_v = -1;
	a_v = 5;
	b_v = 7;
	v0_p = 0;
	v1_p = 3;
	a_p = -5;
	b_p = -5;

	p_crx = p;
	//v_crx = v;

	//Mode = 0;
	//Mode_p = 0;

	v_Kal.LastP = 0.02;
	v_Kal.NowP = 0;
	v_Kal.Out = 0;
	v_Kal.Kg = 0;
	v_Kal.Q = 0.001;
	v_Kal.R = 0.543;

    Right.Hip.State.counter = 0;
    Right.Hip.State.sum = 0;
    Right.Hip.State.Flag = 0;
    Right.Knee.State.counter = 0;
    Right.Knee.State.sum = 0;
    Right.Knee.State.Flag = 0;
    Right.Ankle.State.counter = 0;
    Right.Ankle.State.sum = 0;
    Right.Ankle.State.Flag = 0;

	////参数初始化------------------------------------------------------------------////END


	while (1)
	{
		/*///恢复零点------------------------------------------------------------------////BEG
		HAL_CAN_Start(&hcan1);
		cansend(CANID, 0, 0, 10, 1, 0);
		canread();
		printf("p = %.3f\r\n", val_out.p_out);
		HAL_Delay(10);
		////恢复零点------------------------------------------------------------------////END*/


//		printf("OK\r\n");
//		HAL_TIM_Base_Start_IT(&htim2);
//		HAL_CAN_Start(&hcan1);
//		CalibrationZeroPoint(CANID);
//		EnterMotorMode(CANID);
//		HAL_Delay(100);



		////主程序------------------------------------------------------------------////BEG

		if(MotorMode == 1){
			if(Flag == 1){
				//printf("[Debug info] whiling...\r\n");
				if(Mode == 0){
					//printf("[Debug info] Mode_MIT");
				}
				else if(Mode == 1){
					//printf("[Debug info] Mode_v");
					if(Mode_v == 0){
						p = 0;
						kp = 0;
						//printf("[Debug info] Mode_v_n");
					}
					else if(Mode_v == 1){
						p = 0;
						kp = 0;
						v = S_Trajectory_Vel_T(v0, v1, ss, s_t, c_v, a_v, b_v);
						//printf("[Debug info] Mode_v_s");
					}
					else if(Mode_v == 2){
						p = 0;
						kp = 0;
						v = sin_v_n(ss, A_sin, w_sin, f_sin, B_sin, Hz_sin);
					}
				}
				else if(Mode == 2){
					//printf("[Debug info] Mode_p");
					if(Mode_p == 0){
						v = 0;
						//printf("[Debug info] Mode_p_n");
					}
					else if(Mode_p == 1){
						v = 0;
						p = Climb(p_crx, k, ss);
						//printf("[Debug info] Mode_p_c");
					}
					else if(Mode_p == 2){
						v = 0;
						//p = sin_p_n(ss, A_sin, w_sin, f_sin, B_sin, Hz_sin);
						//p = sin_p_n(us, A_sin, w_sin, f_sin, B_sin, Hz_sin); //乐聚关节性能测试�?20221109
					}
				}
				else{
					Mode = 0;
				}
				//HAL_Delay(1);
			}
			else if(Flag != 1){
				if(Tim1Flag == 1){
					HAL_TIM_Base_Start_IT(&htim2);
				}
				//HAL_TIM_Base_Start_IT(&htim3); //乐聚关节性能测试用20221109
				//ResetPoint(CANID);
				HAL_CAN_Start(&hcan1);
				EnterMotorMode(CANID);
				HAL_CAN_Stop(&hcan1);
				if(Mode_Ang != 0 || Mode_Ang !=3){
					SPI_AS5048A_Origin_Value();
				}
				Flag = 1;
				//printf("[Debug info] first\r\n");
				HAL_Delay(5);
			}


			HAL_CAN_Start(&hcan1);
			cansend(CANID, p, v, kp, kv, tr);
			canread();

//			KalmanFilter(&v_Kal, val_out.v_out);
//			dy = Derivative(val_out.p_out, dp);
//			dp = val_out.p_out;
//			printf("%.3f,%.3f,%.3f,%.3f,%.3f\r\n", val_out.p_out, val_out.v_out, v_Kal.Out, dy, v);

//			printf("%.3f,%.3f,%.3f,%.3f,%.3f\r\n", val_out.p_out, val_out.v_out, val_out.t_out, p, v);

//			Angle_Origin = SPI_AS5048A_Algorithm_ReadData();
//			if(Mode_Ang == 1 || Mode_Ang == 0){
//				Angle = Angle_Circle(Angle_Origin);
//			}
//			else if(Mode_Ang == 2){
//				Angle = Angle_absolute(Angle_Origin);
//			}

			if(Mode_Ang == 3){
				printf("%.2f,%.2f,%.2f,%.2f,%.2f\r\n", val_out.p_out, val_out.v_out, val_out.t_out, p, v);
			}
			else{
				Angle_Origin = SPI_AS5048A_Algorithm_ReadData();
				if(Mode_Ang == 1 || Mode_Ang == 0){
					Angle = Angle_Circle(Angle_Origin);
				}
				else if(Mode_Ang == 2){
					Angle = Angle_absolute(Angle_Origin);
				}
				printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", val_out.p_out, val_out.v_out, val_out.t_out, p, v, Angle);
			}
			HAL_Delay(5);
			//printf("%d",us);
			//HAL_Delay_us(50); //乐聚关节性能测试用20221109
		}

		else if(MotorMode == 2){
			////测试模式
			if(Flag == 1){

				/*//测试：S模式同时规划速度与位置///BEG
				p = S_Trajectory_Pos_T(0, 3, ss, 4, -5, -7);
				v = S_Trajectory_Vel_T(0, 3, ss, 4, -1, 5, 7);

				HAL_CAN_Start(&hcan1);
				cansend(CANID, p, v, kp, kv, tr);
				canread();
				printf("%.3f,%.3f,%.3f,%.3f,%.3f\r\n", val_out.p_out, val_out.v_out, val_out.t_out, p, v);
				//printf("%.3f,%.3f\r\n", p, v);
				//printf("%.3f,%.3f,%.3f\r\n", val_out.p_out, val_out.v_out, v);
				HAL_Delay(10);
				///测试：S模式同时规划速度与位置///END*/


				/*//测试：sin速度规划///BEG
				HAL_CAN_Start(&hcan1);
				p = 0;
				kp = 0;
				tr = 0;
				v = sin_v_n(ss, 6.28, 0, 0, 0, 1);
				Angle_Origin = SPI_AS5048A_Algorithm_ReadData();
				Angle = Angle_absolute(Angle_Origin);
				//Angle = Angle_Circle(Angle_Origin);
				//Test_Value = Return_Value();
				//printf("Origin = %f, Angle = %f\n\r", Angle_Origin, Angle);
				//printf("look = %d\n\r", Test_Value);

//				cansend(CANID, p, v, kp, kv, tr);
//				canread();
				printf("%.3f,%.3f\r\n", v, Angle);
				///测试：sin速度规划///END*/

				/*//测试：无数学模型指导瞎尝试稳摆QAQ_1///BEG
				HAL_CAN_Start(&hcan1);
				Angle_Origin = SPI_AS5048A_Algorithm_ReadData();
				Angle = Angle_absolute(Angle_Origin);
				if(fabs(Angle) <= 5){
					v = Angle*1.2;
				}
				else{
					v = 0;
				}
				cansend(CANID, 0, v, 0, kv, 0);
				canread();
				printf("%.3f,%.3f,%.3f\r\n", val_out.v_out, v, Angle);
				HAL_Delay(1);
				///测试：无数学模型指导瞎尝试稳摆QAQ_1///END*/

				/*//测试：无数学模型指导瞎尝试稳摆QAQ_2///BEG
				HAL_CAN_Start(&hcan1);
				Angle_Origin = SPI_AS5048A_Algorithm_ReadData();
				Angle = Angle_absolute(Angle_Origin);
				if(fabs(Angle) <=10){
					v = KA*Angle + KE*(Angle - Angle_old);
					if(fabs(v) > 15){
						v = 0;
					}
				}
				else{
					v = 0;
				}
				Angle_old = Angle;
				cansend(CANID, 0, v, 0, kv, 0);
				canread();
				printf("%.3f,%.3f,%.3f\r\n", val_out.v_out, v, Angle);
				HAL_Delay(5);
				///测试：无数学模型指导瞎尝试稳摆QAQ_2///END*/

				/*//测试：无数学模型指导瞎尝试稳摆QAQ_3///BEG未测试
				HAL_CAN_Start(&hcan1);
				Angle_Origin = SPI_AS5048A_Algorithm_ReadData();
				Angle = Angle_absolute(Angle_Origin);
				if(Angle <=170 && Angle >=-170){
					v = sin_v_n(ss, A_sin, w_sin, f_sin, B_sin, Hz_sin);
				}
				else if(Angle > 170 && Angle < 180){
					Angle = Angle - 180;
					v = KA*Angle + KE*(Angle - Angle_old);
					Angle_old = Angle;
				}
				else if(Angle >= -180 && Angle < -170){
					Angle = Angle + 180;
					v = KA*Angle + KE*(Angle - Angle_old);
					Angle_old = Angle;
				}
				cansend(CANID, 0, v, 0, kv, 0);
				canread();
				printf("%.3f,%.3f,%.3f\r\n", val_out.v_out, v, Angle);
				HAL_Delay(5);
				///测试：无数学模型指导瞎尝试稳摆QAQ_3///END*/


				/*//测试：无数学模型指导瞎尝试稳摆QAQ_4///BEG
				HAL_CAN_Start(&hcan1);
				Angle_Origin = SPI_AS5048A_Algorithm_ReadData();
				Angle = Angle_absolute(Angle_Origin);
				if(fabs(Angle) <=10){
					error_sum += Angle;
					//error = 0 - val_out.p_out;
					//error = error*0.8;
					//error += error_old*0.2;
					//error_sum += error;
					v = KA*Angle + KS*error_sum + KE*(Angle - Angle_old) ;
					error_old = error;
					if(fabs(v) > 15){
						v = 0;
					}
				}
				else{
					v = 0;
				}
				Angle_old = Angle;

				cansend(CANID, 0, v, 0, kv, 0);
				canread();
				printf("%.3f,%.3f,%.3f\r\n", val_out.v_out, v, Angle);
				HAL_Delay(5);
				///测试：无数学模型指导瞎尝试稳摆QAQ_4///END*/

				/*//测试：无数学模型指导瞎尝试起摆QAQ///BEG
				HAL_CAN_Start(&hcan1);
				Angle_Origin = SPI_AS5048A_Algorithm_ReadData();
				Angle = Angle_absolute(Angle_Origin);

				if(state == 0){
					if(Angle <= 5 && Angle >= -5){
						count = count + 1;
						ss = 0;
					}
					else if(Angle > 170){
						state = 1;
					}

					if(fabs(v_origin) <= 15 && ss <= v_time){
						if(count % 2 == 0){
							cansend(CANID, 0, v_origin, 0, kv, 0);
						}
						else if(count % 2 == 1){
							cansend(CANID, 0, -v_origin, 0, kv, 0);
						}
					}
					else{
						cansend(CANID, 0, 0, 0, kv, 0);
					}
				}

				else if(state == 1){
					Angle = 180 - fabs(Angle);
					if(fabs(Angle) <=10){
						error_sum += Angle;
						//error = 0 - val_out.p_out;
						//error = error*0.8;
						//error += error_old*0.2;
						//error_sum += error;
						v = KA*Angle + KS*error_sum + KE*(Angle - Angle_old) ;
						error_old = error;
						if(fabs(v) > 15){
							v = 0;
						}
					}
					else{
						v = 0;
					}
					Angle_old = Angle;

					cansend(CANID, 0, v, 0, kv, 0);
					canread();
					printf("%.3f,%.3f,%.3f\r\n", val_out.v_out, v, Angle);
					HAL_Delay(5);
				}

				//cansend(CANID, 0, v, 0, kv, 0);
				canread();
				printf("%.3f,%.3f,%.3f\r\n", val_out.v_out, v, Angle);
				HAL_Delay(10);
				///测试：无数学模型指导瞎尝试起摆QAQ///END*/

				/*//测试：无数学模型指导瞎尝试起摆_2QAQ///BEG
				HAL_CAN_Start(CANID);
				Angle_Origin = SPI_AS5048A_Algorithm_ReadData();
				Angle = Angle_absolute(Angle_Origin);
				if(fabs(Angle) > 170){
					state = 1;
					A_sin = B_sin;
					w_sin = f_sin;
					ss = 0;
				}
				else if(fabs(Angle) <= 170){
					Angle_old = 0;
					state = 0;
				}
				if(state == 0){
					v = A_sin*sin(w_sin*ss*0.01);
					if(A_sin < 15){
						A_sin = A_sin + p*0.01;
					}
					else{
						A_sin = 15;
					}
					if(w_sin < 15){
						w_sin = w_sin + kp*0.01;
					}
					else{
						w_sin = 15;
					}
				}
				else if(state == 1){
					if(Angle > 0){
						Angle = 180 - fabs(Angle);
					}
					else if(Angle < 0){
						Angle = fabs(Angle) - 180;
					}
					if(fabs(Angle) <= 10){
//						if(ss < 30){
//							error_sum += Angle;
//							v = KA*Angle + KS*error_sum + KE*(Angle - Angle_old);
//							error_old = error;
//						}
//						else{
//							error_sum += Angle;
//							v = 1.2*Angle + 0.2*error_sum + 0.8*(Angle - Angle_old);
//							error_old = error;
//						}


						error_sum += Angle;
						v = KA*Angle + KS*error_sum + KE*(Angle - Angle_old);
						error_old = error;

						if(fabs(v) > 15){
							v = 15;
						}
					}
					else{
						v = 0;
					}
					Angle_old = Angle;
				}

				cansend(CANID, 0, v, 0, kv, 0);
				canread();
				printf("%.3f,%.3f\r\n", val_out.v_out, Angle);
				HAL_Delay(10);
	 			///测试：无数学模型指导瞎尝试起摆_2QAQ///END*/


				/*//达妙科技电机模式测试///BEG
				HAL_CAN_Start(&hcan1);
				v = S_Trajectory_Vel_T(v0, v1, ss, s_t, c_v, a_v, b_v);
				cansend(CANID, 0, v, 0, kv, 0);
				canread();
				printf("%.3f,%.3f\r\n", val_out.p_out, val_out.v_out);
				HAL_Delay(10);

				//注意2022-10-8：CAN通信固定波特率1M,45M-9-3-1

				///达妙科技电机模式测试///END*/

				/*//阻抗控制测试///BEG未写完
				HAL_CAN_Start(&hcan1);
				canread();

				tr = K*val_out.p_out + B*(val_out.p_out - x_old) + M*(val_out.p_out - x_old - xDer_old);
				tr = round(tr*10)/10;
				xDer_old = val_out.p_out - x_old;
				x_old = val_out.p_out;

				cansend(CANID, 0, 0, 0, 0, tr);

				HAL_Delay(5);

				//结果2022-10-15：跑飞了
				//结果2022-10-17：0.2nm飞转，需负载
				///阻抗控制测试///END*/

				/*//卡尔曼滤波测试///BEG未写完
				HAL_CAN_Start(&hcan1);
				canread();

				v = sin_v_n(ss, A_sin, w_sin, f_sin, B_sin, Hz_sin);
				cansend(CANID, 0, v, 0, kv, 0);

				KalmanFilter(&v_Kal, val_out.v_out);
				dy = Derivative(val_out.p_out, dp);
				dp = val_out.p_out;

				printf("%.3f,%.3f,%.3f,%.3f\r\n", v, val_out.v_out, v_Kal.Out, dy);
				HAL_Delay(1);
				///卡尔曼滤波测试///END*/

				///MPU6050模块测试///BEG
				 printf("%.2f,%.2f,%.2f\r\n",Right.Hip.Angx, Right.Knee.Angx-Right.Hip.Angx, Right.Ankle.Angx-Right.Knee.Angx);
				 //printf("%.2f,%.2f,%.2f\r\n",Right.Hip.Angx, Right.Knee.Angx, Right.Ankle.Angx);
				 HAL_Delay(10);
				 ///MPU6050模块测试///END

			}
			else if(Flag != 1){
				if(Tim1Flag == 1){
					HAL_TIM_Base_Start_IT(&htim2);
				}
				HAL_CAN_Start(&hcan1);
				//CalibrationZeroPoint(CANID);
				EnterMotorMode(CANID);
				Flag = 1;
				if(Mode_Ang != 0){
					SPI_AS5048A_Origin_Value();
					HAL_Delay(1);
					SPI_AS5048A_Origin_Value();
				}
				HAL_Delay(2);
			}
		}

		else if(MotorMode == 0){
			HAL_CAN_Start(&hcan1);
			ExitMotorMode(CANID);
			HAL_Delay(2);
			HAL_CAN_Stop(&hcan1);
			HAL_TIM_Base_Stop_IT(&htim2);
			ss = 0;
			Flag = 0;
			p_compare = 0;
			//Angle_old = 0;
			//v_compare = 0;y
			printf("[Debug info] stop\r\n");
			while(1){
				if(MotorMode != 0)break;
			}
		}

		////主程序------------------------------------------------------------------////END/


		/*///测试：速度模式------------------------------------------------------------------////BEG
		HAL_CAN_Start(&hcan1);
		cansend(0x00, 0, 6.28, 0, 1, 0);
		canread();
		printf("pos = %.3f, vel = %.3f\r\n", val_out.p_out, val_out.v_out);
		HAL_Delay(10);

		//测试结果：
		//叶旧电机2022-8-11：kv为较小值时运行比较稳定，较大容易嗡嗡响或产生剧烈抖动；v较小且kv较小，容易在卡顿处停下，嗡嗡响

		////测试：速度模式------------------------------------------------------------------////END*/


		/*///测试：位置模式------------------------------------------------------------------////BEG
		HAL_CAN_Start(&hcan1);
		//cansend(0x00, 0, 2, 0, 1, 0);
		cansend(0x00, PI, 2, 0, 1, 0);
		canread();
		printf("pos = %.3f, vel = %.3f\r\n", val_out.p_out, val_out.v_out);
		HAL_Delay(10);
		////测试：位置模式------------------------------------------------------------------////END*/


		/*////测试：位于3.14与0之间5秒循环,输出位置速度时间------------------------------------------------------------------////BEG
		HAL_CAN_Start(&hcan1);
		//HAL_CAN_Start(&hcan2);
		if(p_compare > 4){
		  cansend(CANID, p, v, kp, kv, tr);
		}
		else{
		  cansend(CANID, 0, v, kp, kv, tr);
		}
		canread();
		printf("p = %.3f, v = %.3f, s = %d, ss = %d, compare = %d\r\n", val_out.p_out, val_out.v_out, s, ss, compare);
		HAL_Delay(10);
		//HAL_CAN_Stop(&hcan1);
		//HAL_CAN_Stop(&hcan2);

		//测试结果：
		//U8电机1：2022-8-10：第一次标定嗡嗡响，转动也响；第二次标定正常，转动依然响

		////测试：位于3.14与0之间5秒循环,输出位置速度时间------------------------------------------------------------------////END*/


		/*///测试：p爬升变化------------------------------------------------------------------////
		//注意注掉ss = 0;
		HAL_CAN_Start(&hcan1);
		p = Climb(4*PI, 1, ss);
		cansend(CANID, p, v, kp, kv, tr);
		canread();
		printf("p = %.3f, v = %.3f, s = %d, ss = %d, p_int = %.3f\r\n", val_out.p_out, val_out.v_out, s, ss, p);
		HAL_Delay(10);

		//测试结果：
		//叶旧电机2022-8-6：速度抖动，电机本身安装也有问题，基本趋势与仿真相同，理论上没错

		////测试：p爬升变化------------------------------------------------------------------////END*/


		/*///测试：p爬升较大变化------------------------------------------------------------------////BEG
		//注意ss = 0;
		HAL_CAN_Start(&hcan1);
		if(s >= 0&&s < 5){
		  p = 1;
		  cansend(0x00, 1, v, kp, kv, tr);
		}
		else if(s >= 5 && s < 10){
		  p = 2;
		  cansend(0x00, 2, v, kp, kv, tr);
		}
		else if(s >= 10 && s < 15){
		  p = 3;
		  cansend(0x00, 3, v, kp, kv, tr);
		}
		else{
		  p = 4;
		  cansend(0x00, 4, v, kp, kv, tr);
		}
		canread();
		printf("p = %.3f, v = %.3f, s = %d, ss = %d, p_int = %.3f\r\n", val_out.p_out, val_out.v_out, s, ss, p);
		HAL_Delay(50);

		//测试结果：
		//叶旧电机2022-8-6：间隔时间太长，速度一顿一顿的，意料之中，理论上没错

		////测试：p爬升变化------------------------------------------------------------------////END*/


		/*///测试：S型速度曲线变化------------------------------------------------------------------////BEG
		//注意注掉ss = 0;
		HAL_CAN_Start(&hcan1);
		v = S_Trajectory_Vel_T(0, 6.28, ss, 4, -1, 5, 5);

		//v = 0 + 6.28/(1 + exp(-0.05*ss+5));//37.905s-34.807s=3.098s

		cansend(0x00, 0, v, 0, 1, 0);
		canread();
		//printf("%.3f\r\n",v);
		printf("pos = %.3f, vel = %.3f, v_in = %.3f\r\n", val_out.p_out, val_out.v_out, v);
		HAL_Delay(10);

		//测试结果：
		//叶旧电机2022-8-16：基本无抖动，速度整体趋势与仿真相同，理论上没错

		////测试：S型速度曲线变化------------------------------------------------------------------////END*/


		/*///测试：S型曲线位置变化------------------------------------------------------------------////BEG
		//注意注掉ss = 0;
		HAL_CAN_Start(&hcan1);
		p = S_Trajectory_Pos_T(0, 3, ss, 4, -5, -7);

		cansend(0x00, p, 0, 1, 0, 0);
		canread();
		//printf("%.3f\r\n", p);
		printf("pos = %.3f, vel = %.3f, p_in = %.3f\r\n", val_out.p_out, val_out.v_out, p);
		HAL_Delay(10);

		//测试结果：
		//叶旧电机2022-8-16：前面有抖动（大概原因初始位置在-12左右导致前面计算值较大），位置与仿真相同，速度整体趋势与仿真看似相同，但比速度控制更加抖动（大概原因位置范围在12到-12，为了观察现象，速度较低，且有抖动，不明显），理论上没错

		////测试：S型曲线位置变化------------------------------------------------------------------////END*/


		/*///测试：S型速度规划加位置规划矫正------------------------------------------------------------------////BEG未测试
		//注意注掉ss = 0;
		HAL_CAN_Start(&hcan1);
		v = S_Trajectory_Vel_T(0, 3, ss, 4, -1, 5, 7);
		p = S_Trajectory_Pos_T(0, 3, ss, 4, -5, -7);

		cansend(0x00, p, v, 1, 2, 0);
		canread();
		printf("pos = %.3f, vel = %.3f, p_in = %.3f\r\n", val_out.p_out, val_out.v_out, p);
		HAL_Delay(10);
		////测试：S型速度规划加位置规划矫正------------------------------------------------------------------////END*/


		/*////测试：串口数据发送接收------------------------------------------------------------------////BEG
		printf("ok\r\n");
		HAL_Delay(1000);
		////测试：串口数据发送接收------------------------------------------------------------------////END*/


		/*///测试：理想状态角度返回值------------------------------------------------------------------////BEG
		Angle_Origin = (float)SPI_AS5048A_ReadData();
		Angle = Angle_Origin/16384*360*2;
		printf("Origin = %f, Angle = %f \n\r",data_ang.angle_now, Angle_Origin, Angle);
		HAL_Delay(1000);
		////测试：理想状态角度返回值------------------------------------------------------------------////END*/


		/*///测试：理想状态角度返回相对零点值_汪part2------------------------------------------------------------------////BEG
		Angle_Origin = (float)Angle_Process(&data_ang);
		Angle = Angle_Origin/16384*360*2;
		printf("raw = %d, Origin = %f, Angle = %f \n\r",data_ang.angle_now, Angle_Origin, Angle);
		HAL_Delay(1000);
		////测试：理想状态角度返回相对零点值_汪part2------------------------------------------------------------------////END*/


		/*///测试：通电“零点相对角度返回值part2------------------------------------------------------------------////BEG/
		Angle_Origin = SPI_AS5048A_Algorithm_ReadData();
		Angle = Angle_Origin/16384*360*2;
		//Test_Value = Return_Value();
		printf("Origin = %f, Angle = %f\n\r", Angle_Origin, Angle);
		//printf("look = %d\n\r", Test_Value);
		HAL_Delay(1000);
		////测试：通电“零点相对角度返回值part2------------------------------------------------------------------////BEG*/


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

////计时------------------------------------------------------------------////BEG
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == htim2.Instance){
	    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	    ss++;
	    if(ss == 100){
		//爬升或S模式下注释掉ss=0
		//ss = 0;
		//秒分时
		/*s++;
			if(s == 60){
		  		s = 0;
		  		m++;
		  		if(m == 60){
			  		m = 0;
			  		h++;
		  		}
			}
	  }*/

	  //位置模式测试判断参数，当计时值p_compare（v�? >= 设定值p_com_delay（v）时，计时值归零
		    if(ss % 10 == 0){
		    	p_compare++;
				if(p_compare >= p_com_delay){
					p_compare = 0;
				}

	//    v_compare++;
	//	if(v_compare >= v_com_delay){
	//	  v_compare = 0;
		    }
	    }
	}
	else if(htim->Instance == htim3.Instance){
		us++;
		if(us*0.00005 >= 1/Hz_sin*3){
			us = 0;
		}
	}
}
////计时------------------------------------------------------------------////END



/*////CAN中断------------------------------------------------------------------////BEG
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	canread();
	printf("p = %f\r\n", val_out.p_out);
}
////CAN中断------------------------------------------------------------------////END*/

////串口中断，先退出电机模式，再更新参数------------------------------------------------------------------////BEG

/*已过时列表未更新
 * 20220825p5
 * S=1/2/0 �?始发�?/测试/停止发�?�指�?
 * S=0�?// C=? CANID设置
 * S=0�?// p=../? p设置
 * S=0�?// v=../? v设置
 * S=0�?// P=? kp设置
 * S=0�?// V=? kv设置
 * S=0�?// t=? t设置
 * S=0�? cp? 爬坡p与p_crx设置
 * S=0�? ck? 爬坡k设置
 * S=0�? so? S速度v0设置
 * S=0�? sv? S速度v1设置
 * S=0�? st? S速度t设置
 * S=0�? sc? S速度c设置
 * S=0�? sa? S速度a设置
 * S=0�? sb? S速度b设置
 * S=0�? m=? Mode设置
 * S=0�? mv? Mode_v设置
 * S=0�? mp? Mode_p设置
 *
 * S=0�? cd0 显示�?有可设置参数
 */
#pragma region
uint8_t rxBuf_2[256];
uint32_t rxBufCursor_2 = 0;
uint8_t rxBuf_1[256];
uint32_t rxBufCursor_1 = 0;
#pragma endregion

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if(huart->Instance == USART2){
	  if(cRx_2 == '\n')
	  {
		//uint8_t operand1;
		float operand;
		char operator1, operator2;

		rxBuf_2[rxBufCursor_2] = '\0';
		if(sscanf((const char*)rxBuf_2, "%c%c%f", &operator1, &operator2, &operand) == 3)
		{
		  //printf("[Debug info] Expression receive\r\n");
		  //if(MotorMode == 0){
		  if((Mode == 0 && Mode_v == 0) || (Mode == 1 && Mode_p == 0) || MotorMode == 0){
			  switch(operator1){
			  case 'C':
				  canid = operand;
				  CANID = (uint32_t)canid;
				  if(MotorMode == 0){
					  printf("[Debug info] %ld,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", CANID, p, v, kp, kv, tr);
				  }
				  break;
			  case 'p':
				  p_crx = p;
				  switch(operator2){
					  case '=':
						  p = operand;
						  break;
					  case '+':
						  p = p + operand;
						  break;
					  case '-':
						  p = p - operand;
						  break;
					  case '*':
						  p = p*operand;
						  break;
					  case '/':
						  p = p/operand;
						  break;
					  default:
						  break;
				  }
				  if(p <= 4*PI){
					  if(MotorMode == 0){
						  printf("[Debug info] %ld,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", CANID, p, v, kp, kv, tr);
					  }
				  }
				  else if(p > 4*PI){
					  p = p_crx;
					  if(MotorMode == 0){
						  printf("[Debug info] Error\r\n");
						  printf("[Debug info] %ld,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", CANID, p, v, kp, kv, tr);
					  }
				  }
				  break;
			  case 'v':
				  v_crx = v;
				  switch(operator2){
				  case '=':
					  v = operand;
					  serial_send = 0;
					  break;
				  case '+':
					  v = v + operand;
					  serial_send = 0;
					  break;
				  case '-':
					  v = v - operand;
					  serial_send = 0;
					  break;
				  case '*':
					  v = v*operand;
					  serial_send = 0;
					  break;
				  case '/':
					  v = v/operand;
					  serial_send = 0;
					  break;
/*
				  case 'o':
					  v_origin = operand;
					  serial_send = 1;
					  break;
				  case 't':
					  v_time = operand;
					  serial_send = 1;
					  break;
*/
				  default:
					  break;
				  }
				  if(MotorMode == 0){
					  if(serial_send == 0){
						  printf("[Debug info] %ld,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", CANID, p, v, kp, kv, tr);
					  }
/*
					  else if(serial_send == 1){
						  printf("[Debug info] %.3f,%.3f\r\n", v_origin, v_time);
					  }
*/
				  }
				  break;
			  case 'P':
				  if(operator2 == '='){
					  kp = operand;
					  if(MotorMode == 0){
						  printf("[Debug info] %ld,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", CANID, p, v, kp, kv, tr);
					  }
				  }
				  else if(operator2 != '='){
					  if(MotorMode == 0){
						  printf("[Debug info] Error\r\n");
						  printf("[Debug info] %ld,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", CANID, p, v, kp, kv, tr);
					  }
				  }
				  break;
			  case 'V':
				  if(operator2 == '='){
					  kv = operand;
					  if(MotorMode == 0){
						  printf("[Debug info] %ld,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", CANID, p, v, kp, kv, tr);
					  }
				  }
				  else if(operator2 != '='){
					  if(MotorMode == 0){
						  printf("[Debug info] Error\r\n");
						  printf("[Debug info] %ld,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", CANID, p, v, kp, kv, tr);
					  }
				  }
				  break;
			  case 't':
				  if(operator2 == '='){
					  tr = operand;
					  if(MotorMode == 0){
						  printf("[Debug info] %ld,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", CANID, p, v, kp, kv, tr);
					  }
				  }
				  else if(operator2 != '='){
					  if(MotorMode == 0){
						  printf("[Debug info] Error\r\n");
						  printf("[Debug info] %ld,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", CANID, p, v, kp, kv, tr);
					  }
				  }
				  break;
			  case 'c':
				  if(MotorMode == 0){
					  switch(operator2){
					  case 'p': //爬坡位置目标值
						  if(Mode_p == 1){
							  p = operand;
						  }
						  p_crx = p;
						  break;
					  case 'k': //爬坡斜率参数
						  k = operand;
						  break;
					  default:
						  break;
					  }
					  if(operator2 != 'd'){
						  printf("[Debug info] %.3f,%.3f\r\n", p, k);
					  }
					  else if(operator2 == 'd' && operand == 0){
						  printf("[Debug info]\r\n"
								  "MINI_V20221106a0930\r\n"
								  "C=?\tCANID:\t%ld[no]\r\n"
								  "p=../?\tp:\t%.3f[rad]\r\n"
								  "v=../?\tv:\t%.3f[rad/s]\r\n"
								  "P=?\tkp:\t%.3f[no]\r\n"
								  "V=?\tkv:\t%.3f[no]\r\n"
								  "t=?\ttr:\t%.3f[s]\r\n"
								  "cp?\tp_crx:\t%.3f[rad]\r\n"
								  "ck?\tk:\t%.3f[no]\r\n"
								  "so?\tv0:\t%.3f[rad/s]\r\n"
								  "sv?\tv1:\t%.3f[rad/s]\r\n"
								  "st?\ts_t:\t%.3f[s]\r\n"
								  "sc?\tc_v:\t%.3f[no]\r\n"
								  "sa?\ta_v:\t%.3f[no]\r\n"
								  "sb?\tb_v:\t%.3f[no]\r\n"
								  "sA?\tA_sin:\t%.3f[no]\r\n"
								  "sw?\tw_sin:\t%.3f[no]\r\n"
								  "sf?\tf_sin:\t%.3f[no]\r\n"
								  "sB?\tB_sin:\t%.3f[no]\r\n"
								  "sH?\tHz_sin:\t%.3f[no]\r\n"
								  "m=?\tMode:\t%.0f\t0:v,1:p,2:t\r\n"
								  "mv?\tMode_v:\t%.0f\t0:n,1:s,2:sin\r\n"
								  "mp?\tMode_p:\t%.0f\t0:n,1:c\r\n"
								  "ma?\tMode_a:\t%.0f\t0:no,1:zero,2:zero_abs\r\n",
								  CANID, p, v, kp, kv, tr, p_crx, k, v0, v1, s_t, c_v, a_v, b_v, A_sin, w_sin, f_sin, B_sin, Hz_sin, Mode, Mode_v, Mode_p, Mode_Ang);
					  }
					  else if(operator2 == 'd' && operand == 1){
						  printf("%.3f,%.3f,%.3f,%.3f,%.3f,"
								  "%.0f,%.0f,%.0f,%.0f\r\n",
								  p, v, kp, kv, tr,
								  Mode, Mode_v, Mode_p, Mode_Ang);
					  }
				  }
				  break;
			  case 's':
				  if(MotorMode == 0){
					  switch(operator2){
					  case 'o':
						  v0 = operand; //S速度曲线初始值
						  serial_send = 0;
						  break;
					  case 'v':
						  v1 = operand; //S速度曲线目标值
						  serial_send = 0;
						  break;
					  case 't':
						  s_t = operand; //S速度曲线持续时间
						  serial_send = 0;
						  break;
					  case 'c':
						  c_v = operand; //S速度参数1
						  serial_send = 0;
						  break;
					  case 'a':
						  a_v = operand; //S速度参数2
						  serial_send = 0;
						  break;
					  case 'b':
						  b_v = operand; //S速度参数3
						  serial_send = 0;
						  break;
					  case 'A':
						  A_sin = operand; //A*sin(w*x*0.01 + fai) + B;
						  serial_send = 1;
						  break;
					  case 'w':
						  w_sin = operand; //A*sin(w*x*0.01 + fai) + B;
						  serial_send = 1;
						  break;
					  case 'f':
						  f_sin = operand; //A*sin(w*x*0.01 + fai) + B;
						  serial_send = 1;
						  break;
					  case 'B':
						  B_sin = operand; //A*sin(w*x*0.01 + fai) + B;
						  serial_send = 1;
						  break;
					  case 'H':
						  Hz_sin = operand; //A*sin(Hz*2*3.1415926*x*0.01 + fai) + B;
						  serial_send = 1;
						  break;
					  default:
						  break;
					  }
					  if(serial_send == 0){
						  printf("[Debug info] S_Trajectory:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", v0, v1, s_t, c_v, a_v, b_v);
					  }
					  else if(serial_send == 1){
						  printf("[Debug info] Sin:%.3f,%.3f,%.3f,%.3f,%.3f\r\n", A_sin, w_sin, f_sin, B_sin, Hz_sin);
					  }
				  }
				  break;
			  case 'K':
				  if(MotorMode == 0){
					  switch(operator2){
/*					  //倒立摆PID参数
					  case 'A':
						  KA = operand;
						  serial_send = 0;
						  break;
					  case 'E':
						  KE = operand;
						  serial_send = 0;
						  break;
					  case 'S':
						  KS = operand;
						  serial_send = 0;
						  break;
					  case 'M':
						  M = operand;
						  serial_send = 1;
						  break;
					  case 'B':
						  B = operand;
						  serial_send = 1;
						  break;
					  case 'K':
						  K = operand;
						  serial_send = 1;
						  break;
*/
					  case 'Q':
						  v_Kal.Q = operand; //卡尔曼滤波过程噪声协方差
						  serial_send = 2;
						  break;
					  case 'R':
						  v_Kal.R = operand; //卡尔曼滤波观测噪声协方差
						  serial_send = 2;
						  break;
					  default:
						  break;
					  }
					  if(serial_send == 0){
						  //printf("[Debug info] %.3f,%.3f,%.3f\r\n", KA, KE, KS);
					  }
					  else if(serial_send == 1){
						  //printf("[Debug info] %.3f,%.3f,%.3f\r\n", M, B, K);
					  }
					  else if(serial_send == 2){
						  printf("[Debug info] %.3f,%.3f\r\n",v_Kal.Q, v_Kal.R);
					  }
				  }
				  break;
			  case 'm':
				  if(MotorMode == 0){
					  switch(operator2){
					  case '=':
						  Mode = operand; //模式选择
						  break;
					  case 'v':
						  Mode_v = operand; //速度模式选择
						  if(Mode_v != 0){
							  Tim1Flag = 1;
						  }
						  else{
							  Tim1Flag = 0;
						  }
						  break;
					  case 'p':
						  Mode_p = operand; //位置模式选择
						  if(Mode_p != 0){
							  Tim1Flag = 1;
						  }
						  else{
							  Tim1Flag = 0;
						  }
						  break;
					  case 'a':
						  Mode_Ang = operand; //角度模式选择
						  break;
					  default:
						  break;
					  }
					  printf("[Debug info] %.0f,%.0f,%.0f,%.0f,%d\r\n", Mode, Mode_v, Mode_p, Mode_Ang, Tim1Flag);
				  }
				  break;
			  case 'Z':
				  switch(operator2){
				  case 'e':
					  if(operand == 0 && MotorMode == 0){
						  ResetPoint(CANID);
					  }
					  break;
				  case 'E':
					  if(operand == 0 && MotorMode == 0){
						  CalibrationZeroPoint(CANID);
					  }
					  break;
				  case 'A':
					  if(operand == 0 && MotorMode == 0){
						  p = 0;
						  v = 0;
						  kp = 0;
						  kv = 0;
						  tr = 0;
						  k = 0;
						  v0 = 0;
						  v0_p = 0;
						  Mode = 0;
						  Mode_p = 0;
						  Mode_v = 0;
						  Mode_Ang = 3;
						  printf("[Debug info] Zero");
					  }
					  break;
				  default:
					  break;
				  }
				  break;
			  //case '1':
			  default:
				  break;
			  }
		  }
		  else if(MotorMode != 0 && operator1 != 'S'){
			  printf("[Debug info] Motor is run\r\n");
		  }

		  if(operator1 == 'S'){
			  if(operand == 1){
				  MotorMode = 1;
			  }
			  else if(operand == 2){
				  MotorMode = 2;
			  }

			  else if(operand == 3){
				  HAL_CAN_Start(&hcan1);
				  cansend(CANID, 1, 2, 3, 4, 5);
				  HAL_CAN_Stop(&hcan1);
				  printf("ok\r\n");
			  }

			  else{
				  MotorMode = 0;
				  Flag = 0;
			  }
		  }

		}
		else{
			MotorMode = 0;
			Flag = 0;
			printf("[Debug info] Invalid expression\r\n");
		}

		rxBufCursor_2 = 0;
	  }
	  else
	  {
		if(rxBufCursor_2 < 255)
		{
		  rxBuf_2[rxBufCursor_2++] = cRx_2;
		}
	  }

	  if(HAL_UART_Receive_IT(&huart2, &cRx_2, 1) != HAL_OK)
	  {
		Error_Handler();
	  }
  }

  if(Board == 2 || Board == 3){
	  //与ESP32相连或与STM32相连串口
	  if(huart->Instance == USART1){
		  if(cRx_1 == '\n'){
			  printf("[Debug info] test\r\n");
			  rxBufCursor_1 = 0;
		  }
		  else
		  {
			if(rxBufCursor_1 < 255)
			{
			  rxBuf_1[rxBufCursor_1++] = cRx_1;
			}
		  }

		  if(HAL_UART_Receive_IT(&huart1, &cRx_1, 1) != HAL_OK)
		  {
			Error_Handler();
		  }
	  }
  }

  if(Board == 3){
	  //传感器数据串口.暂时仅用角度
	  if(huart->Instance == USART3){
		  //髋关节数据
		  if(Right.Hip.State.counter < 1){
			  if(cRx_3 != 0x55){
				  Right.Hip.State.counter = 0;
				  Right.Hip.State.sum = 0;
			  }
			  else{
				  ++Right.Hip.State.counter;
				  Right.Hip.State.sum += cRx_3;
				  Right.Hip.Buf.rxDataAcc[Right.Hip.State.counter] = cRx_3;
				  Right.Hip.Buf.rxDataAngAcc[Right.Hip.State.counter] = cRx_3;
				  Right.Hip.Buf.rxDataAng[Right.Hip.State.counter] = cRx_3;
			  }
		  }
		  else if(Right.Hip.State.counter < 2){
			  Right.Hip.Buf.rxDataAcc[Right.Hip.State.counter] = cRx_3;
			  Right.Hip.Buf.rxDataAngAcc[Right.Hip.State.counter] = cRx_3;
			  Right.Hip.Buf.rxDataAng[Right.Hip.State.counter] = cRx_3;
			  switch(cRx_3){
			  case 0x51:
	//			  Right.Hip.State.Flag = 1;
	//			  ++Right.Hip.State.counter;
	//			  Right.Hip.State.sum += cRx_3;

				  Right.Hip.State.Flag = 0;
				  Right.Hip.State.counter = 0;
				  Right.Hip.State.sum = 0;
				  break;
			  case 0x52:
	//			  Right.Hip.State.Flag = 2;
	//			  ++Right.Hip.State.counter;
	//			  Right.Hip.State.sum += cRx_3;

				  Right.Hip.State.Flag = 0;
				  Right.Hip.State.counter = 0;
				  Right.Hip.State.sum = 0;
				  break;
			  case 0x53:
				  Right.Hip.State.Flag = 3;
				  ++Right.Hip.State.counter;
				  Right.Hip.State.sum += cRx_3;
				  break;
			  default:
				  Right.Hip.State.Flag = 0;
				  Right.Hip.State.counter = 0;
				  Right.Hip.State.sum = 0;
				  break;
			  }
		  }
		  else if(Right.Hip.State.counter < 11){
			  switch (Right.Hip.State.Flag){
			  case 1:
				  Right.Hip.Buf.rxDataAcc[ Right.Hip.State.counter] = cRx_3;
				  ++Right.Hip.State.counter;
				  Right.Hip.State.sum += cRx_3;
				  break;
			  case 2:
				  Right.Hip.Buf.rxDataAngAcc[ Right.Hip.State.counter] = cRx_3;
				  ++Right.Hip.State.counter;
				  Right.Hip.State.sum += cRx_3;
				  break;
			  case 3:
				  Right.Hip.Buf.rxDataAng[ Right.Hip.State.counter] = cRx_3;
				  ++Right.Hip.State.counter;
				  Right.Hip.State.sum += cRx_3;
				  break;
			  default:
				  Right.Hip.State.Flag = 0;
				  Right.Hip.State.counter = 0;
				  Right.Hip.State.sum = 0;
				  break;
			  }
		  }
		  else if(Right.Hip.State.counter == 11){
			  if(Right.Hip.State.sum != cRx_3){
				  Right.Hip.State.counter = 0;

			  }
			  else{
				  Right.Hip.Buf.rxDataAcc[Right.Hip.State.counter] = cRx_3;
				  Right.Hip.Buf.rxDataAngAcc[Right.Hip.State.counter] = cRx_3;
				  Right.Hip.Buf.rxDataAng[Right.Hip.State.counter] = cRx_3;
			  }
			  Right.Hip.State.counter = 0;
			  Right.Hip.State.sum = 0;

			  //
			  switch(Right.Hip.State.Flag){
			  case 1:
				  Right.Hip.Buf.Accx = (Right.Hip.Buf.rxDataAcc[3]<<8)|Right.Hip.Buf.rxDataAcc[2];
				  Right.Hip.Accx = (float) Right.Hip.Buf.Accx/32768*16*g;
				  Right.Hip.Buf.Accy = (Right.Hip.Buf.rxDataAcc[5]<<8)|Right.Hip.Buf.rxDataAcc[4];
				  Right.Hip.Accy = (float) Right.Hip.Buf.Accy/32768*16*g;
				  Right.Hip.Buf.Accz = (Right.Hip.Buf.rxDataAcc[7]<<8)|Right.Hip.Buf.rxDataAcc[6];
				  Right.Hip.Accz = (float) Right.Hip.Buf.Accz/32768*16*g;
				  break;
			  case 2:
				  Right.Hip.Buf.AngAccx = (Right.Hip.Buf.rxDataAngAcc[3]<<8)|Right.Hip.Buf.rxDataAngAcc[2];
				  Right.Hip.AngAccx = (float) Right.Hip.Buf.AngAccx/32768*2000;
				  Right.Hip.Buf.AngAccy = (Right.Hip.Buf.rxDataAngAcc[5]<<8)|Right.Hip.Buf.rxDataAngAcc[4];
				  Right.Hip.AngAccy = (float) Right.Hip.Buf.AngAccy/32768*2000;
				  Right.Hip.Buf.AngAccz = (Right.Hip.Buf.rxDataAngAcc[7]<<8)|Right.Hip.Buf.rxDataAngAcc[6];
				  Right.Hip.AngAccz = (float) Right.Hip.Buf.AngAccz/32768*2000;
				  break;
			  case 3:

				  Right.Hip.Buf.Angx = (Right.Hip.Buf.rxDataAng[3]<<8)|Right.Hip.Buf.rxDataAng[2];
				  Right.Hip.Angx = (float) Right.Hip.Buf.Angx/32768*180;
				  if(Right.Hip.Angx > 180){
					  Right.Hip.Angx = Right.Hip.Angx - 360;
				  }

				  //采集数据存储，用于拟合，暂时尝试固定轨迹20230521
//				  Right.Hip.Fit.FitBuf1[Right.Hip.Fit.sizenum1] = Right.Hip.Angx;
//				  if(){
//
//				  }
//				  Right.Hip.Fit.sizenum1++;

				  break;
			  default:
				  break;
			  }
			  //

			  Right.Hip.State.Flag = 0;
		  }

		  //MPU6050ModDataBuf(Right.Hip, cRx_3);//暂不可用，等修改

		  if(HAL_UART_Receive_IT(&huart3, &cRx_3, 1) != HAL_OK){
			  Error_Handler();
		  }
	  }
	  if(huart->Instance == UART4){
		  //膝关节数据
		  if(Right.Knee.State.counter < 1){
			  if(cRx_4 != 0x55){
				  Right.Knee.State.counter = 0;
				  Right.Knee.State.sum = 0;
			  }
			  else{
				  ++Right.Knee.State.counter;
				  Right.Knee.State.sum += cRx_4;
				  Right.Knee.Buf.rxDataAcc[Right.Knee.State.counter] = cRx_4;
				  Right.Knee.Buf.rxDataAngAcc[Right.Knee.State.counter] = cRx_4;
				  Right.Knee.Buf.rxDataAng[Right.Knee.State.counter] = cRx_4;
			  }
		  }
		  else if(Right.Knee.State.counter < 2){
			  Right.Knee.Buf.rxDataAcc[Right.Knee.State.counter] = cRx_4;
			  Right.Knee.Buf.rxDataAngAcc[Right.Knee.State.counter] = cRx_4;
			  Right.Knee.Buf.rxDataAng[Right.Knee.State.counter] = cRx_4;
			  switch(cRx_4){
			  case 0x51:
	//			  Right.Knee.State.Flag = 1;
	//			  ++Right.Knee.State.counter;
	//			  Right.Knee.State.sum += cRx_4;

				  Right.Knee.State.Flag = 0;
				  Right.Knee.State.counter = 0;
				  Right.Knee.State.sum = 0;
				  break;
			  case 0x52:
	//			  Right.Knee.State.Flag = 2;
	//			  ++Right.Knee.State.counter;
	//			  Right.Knee.State.sum += cRx_4;

				  Right.Knee.State.Flag = 0;
				  Right.Knee.State.counter = 0;
				  Right.Knee.State.sum = 0;
				  break;
			  case 0x53:
				  Right.Knee.State.Flag = 3;
				  ++Right.Knee.State.counter;
				  Right.Knee.State.sum += cRx_4;
				  break;
			  default:
				  Right.Knee.State.Flag = 0;
				  Right.Knee.State.counter = 0;
				  Right.Knee.State.sum = 0;
				  break;
			  }
		  }
		  else if(Right.Knee.State.counter < 11){
			  switch (Right.Knee.State.Flag){
			  case 1:
				  Right.Knee.Buf.rxDataAcc[ Right.Knee.State.counter] = cRx_4;
				  ++Right.Knee.State.counter;
				  Right.Knee.State.sum += cRx_4;
				  break;
			  case 2:
				  Right.Knee.Buf.rxDataAngAcc[ Right.Knee.State.counter] = cRx_4;
				  ++Right.Knee.State.counter;
				  Right.Knee.State.sum += cRx_4;
				  break;
			  case 3:
				  Right.Knee.Buf.rxDataAng[ Right.Knee.State.counter] = cRx_4;
				  ++Right.Knee.State.counter;
				  Right.Knee.State.sum += cRx_4;
				  break;
			  default:
				  Right.Knee.State.Flag = 0;
				  Right.Knee.State.counter = 0;
				  Right.Knee.State.sum = 0;
				  break;
			  }
		  }
		  else if(Right.Knee.State.counter == 11){
			  if(Right.Knee.State.sum != cRx_4){
				  Right.Knee.State.counter = 0;

			  }
			  else{
				  Right.Knee.Buf.rxDataAcc[Right.Knee.State.counter] = cRx_4;
				  Right.Knee.Buf.rxDataAngAcc[Right.Knee.State.counter] = cRx_4;
				  Right.Knee.Buf.rxDataAng[Right.Knee.State.counter] = cRx_4;
			  }
			  Right.Knee.State.counter = 0;
			  Right.Knee.State.sum = 0;

			  //
			  switch(Right.Knee.State.Flag){
			  case 1:
				  Right.Knee.Buf.Accx = (Right.Knee.Buf.rxDataAcc[3]<<8)|Right.Knee.Buf.rxDataAcc[2];
				  Right.Knee.Accx = (float) Right.Knee.Buf.Accx/32768*16*g;
				  Right.Knee.Buf.Accy = (Right.Knee.Buf.rxDataAcc[5]<<8)|Right.Knee.Buf.rxDataAcc[4];
				  Right.Knee.Accy = (float) Right.Knee.Buf.Accy/32768*16*g;
				  Right.Knee.Buf.Accz = (Right.Knee.Buf.rxDataAcc[7]<<8)|Right.Knee.Buf.rxDataAcc[6];
				  Right.Knee.Accz = (float) Right.Knee.Buf.Accz/32768*16*g;
				  break;
			  case 2:
				  Right.Knee.Buf.AngAccx = (Right.Knee.Buf.rxDataAngAcc[3]<<8)|Right.Knee.Buf.rxDataAngAcc[2];
				  Right.Knee.AngAccx = (float) Right.Knee.Buf.AngAccx/32768*2000;
				  Right.Knee.Buf.AngAccy = (Right.Knee.Buf.rxDataAngAcc[5]<<8)|Right.Knee.Buf.rxDataAngAcc[4];
				  Right.Knee.AngAccy = (float) Right.Knee.Buf.AngAccy/32768*2000;
				  Right.Knee.Buf.AngAccz = (Right.Knee.Buf.rxDataAngAcc[7]<<8)|Right.Knee.Buf.rxDataAngAcc[6];
				  Right.Knee.AngAccz = (float) Right.Knee.Buf.AngAccz/32768*2000;
				  break;
			  case 3:

				  Right.Knee.Buf.Angx = (Right.Knee.Buf.rxDataAng[3]<<8)|Right.Knee.Buf.rxDataAng[2];
				  Right.Knee.Angx = (float) Right.Knee.Buf.Angx/32768*180;
				  if(Right.Knee.Angx > 180){
					  Right.Knee.Angx = Right.Knee.Angx - 360;
				  }
				  break;
			  default:
				  break;
			  }
			  //

			  Right.Knee.State.Flag = 0;
		  }

		  //MPU6050ModDataBuf(Right.Knee, cRx_4);//暂不可用，等修改

		  if(HAL_UART_Receive_IT(&huart4, &cRx_4, 1) != HAL_OK){
			  Error_Handler();
		  }

	  }
	  if(huart->Instance == UART5){
		  //踝关节数据
		  if(Right.Ankle.State.counter < 1){
			  if(cRx_5 != 0x55){
				  Right.Ankle.State.counter = 0;
				  Right.Ankle.State.sum = 0;
			  }
			  else{
				  ++Right.Ankle.State.counter;
				  Right.Ankle.State.sum += cRx_5;
				  Right.Ankle.Buf.rxDataAcc[Right.Ankle.State.counter] = cRx_5;
				  Right.Ankle.Buf.rxDataAngAcc[Right.Ankle.State.counter] = cRx_5;
				  Right.Ankle.Buf.rxDataAng[Right.Ankle.State.counter] = cRx_5;
			  }
		  }
		  else if(Right.Ankle.State.counter < 2){
			  Right.Ankle.Buf.rxDataAcc[Right.Ankle.State.counter] = cRx_5;
			  Right.Ankle.Buf.rxDataAngAcc[Right.Ankle.State.counter] = cRx_5;
			  Right.Ankle.Buf.rxDataAng[Right.Ankle.State.counter] = cRx_5;
			  switch(cRx_5){
			  case 0x51:
	//			  Right.Ankle.State.Flag = 1;
	//			  ++Right.Ankle.State.counter;
	//			  Right.Ankle.State.sum += cRx_5;

				  Right.Ankle.State.Flag = 0;
				  Right.Ankle.State.counter = 0;
				  Right.Ankle.State.sum = 0;
				  break;
			  case 0x52:
	//			  Right.Ankle.State.Flag = 2;
	//			  ++Right.Ankle.State.counter;
	//			  Right.Ankle.State.sum += cRx_5;

				  Right.Ankle.State.Flag = 0;
				  Right.Ankle.State.counter = 0;
				  Right.Ankle.State.sum = 0;
				  break;
			  case 0x53:
				  Right.Ankle.State.Flag = 3;
				  ++Right.Ankle.State.counter;
				  Right.Ankle.State.sum += cRx_5;
				  break;
			  default:
				  Right.Ankle.State.Flag = 0;
				  Right.Ankle.State.counter = 0;
				  Right.Ankle.State.sum = 0;
				  break;
			  }
		  }
		  else if(Right.Ankle.State.counter < 11){
			  switch (Right.Ankle.State.Flag){
			  case 1:
				  Right.Ankle.Buf.rxDataAcc[ Right.Ankle.State.counter] = cRx_5;
				  ++Right.Ankle.State.counter;
				  Right.Ankle.State.sum += cRx_5;
				  break;
			  case 2:
				  Right.Ankle.Buf.rxDataAngAcc[ Right.Ankle.State.counter] = cRx_5;
				  ++Right.Ankle.State.counter;
				  Right.Ankle.State.sum += cRx_5;
				  break;
			  case 3:
				  Right.Ankle.Buf.rxDataAng[ Right.Ankle.State.counter] = cRx_5;
				  ++Right.Ankle.State.counter;
				  Right.Ankle.State.sum += cRx_5;
				  break;
			  default:
				  Right.Ankle.State.Flag = 0;
				  Right.Ankle.State.counter = 0;
				  Right.Ankle.State.sum = 0;
				  break;
			  }
		  }
		  else if(Right.Ankle.State.counter == 11){
			  if(Right.Ankle.State.sum != cRx_5){
				  Right.Ankle.State.counter = 0;

			  }
			  else{
				  Right.Ankle.Buf.rxDataAcc[Right.Ankle.State.counter] = cRx_5;
				  Right.Ankle.Buf.rxDataAngAcc[Right.Ankle.State.counter] = cRx_5;
				  Right.Ankle.Buf.rxDataAng[Right.Ankle.State.counter] = cRx_5;
			  }
			  Right.Ankle.State.counter = 0;
			  Right.Ankle.State.sum = 0;

			  //
			  switch(Right.Ankle.State.Flag){
			  case 1:
				  Right.Ankle.Buf.Accx = (Right.Ankle.Buf.rxDataAcc[3]<<8)|Right.Ankle.Buf.rxDataAcc[2];
				  Right.Ankle.Accx = (float) Right.Ankle.Buf.Accx/32768*16*g;
				  Right.Ankle.Buf.Accy = (Right.Ankle.Buf.rxDataAcc[5]<<8)|Right.Ankle.Buf.rxDataAcc[4];
				  Right.Ankle.Accy = (float) Right.Ankle.Buf.Accy/32768*16*g;
				  Right.Ankle.Buf.Accz = (Right.Ankle.Buf.rxDataAcc[7]<<8)|Right.Ankle.Buf.rxDataAcc[6];
				  Right.Ankle.Accz = (float) Right.Ankle.Buf.Accz/32768*16*g;
				  break;
			  case 2:
				  Right.Ankle.Buf.AngAccx = (Right.Ankle.Buf.rxDataAngAcc[3]<<8)|Right.Ankle.Buf.rxDataAngAcc[2];
				  Right.Ankle.AngAccx = (float) Right.Ankle.Buf.AngAccx/32768*2000;
				  Right.Ankle.Buf.AngAccy = (Right.Ankle.Buf.rxDataAngAcc[5]<<8)|Right.Ankle.Buf.rxDataAngAcc[4];
				  Right.Ankle.AngAccy = (float) Right.Ankle.Buf.AngAccy/32768*2000;
				  Right.Ankle.Buf.AngAccz = (Right.Ankle.Buf.rxDataAngAcc[7]<<8)|Right.Ankle.Buf.rxDataAngAcc[6];
				  Right.Ankle.AngAccz = (float) Right.Ankle.Buf.AngAccz/32768*2000;
				  break;
			  case 3:

				  Right.Ankle.Buf.Angx = (Right.Ankle.Buf.rxDataAng[3]<<8)|Right.Ankle.Buf.rxDataAng[2];
				  Right.Ankle.Angx = (float) Right.Ankle.Buf.Angx/32768*180;
				  if(Right.Ankle.Angx > 180){
					  Right.Ankle.Angx = Right.Ankle.Angx - 360;
				  }
				  break;
			  default:
				  break;
			  }
			  //

			  Right.Ankle.State.Flag = 0;
		  }

		  //MPU6050ModDataBuf(Right.Ankle, cRx_5);//暂不可用，等修改

		  if(HAL_UART_Receive_IT(&huart5, &cRx_5, 1) != HAL_OK){
			  Error_Handler();
		  }
	  }
	  if(huart->Instance == USART6){
		  //足底压力数据
	  }
  }
}
////串口中断，先退出电机模式，再更新参数------------------------------------------------------------------////END/

////MPU6050模块原数据采集------------------------------------------------------------------////BEG未测试
void MPU6050ModDataBuf(struct Data AllData, uint8_t cRx){
	  if(AllData.State.counter < 1){
		  if(cRx != 0x55){
			  AllData.State.counter = 0;
			  AllData.State.sum = 0;
		  }
		  else{
			  ++AllData.State.counter;
			  AllData.State.sum += cRx;
			  AllData.Buf.rxDataAcc[AllData.State.counter] = cRx;
			  AllData.Buf.rxDataAngAcc[AllData.State.counter] = cRx;
			  AllData.Buf.rxDataAng[AllData.State.counter] = cRx;
		  }
	  }
	  else if(AllData.State.counter < 2){
		  AllData.Buf.rxDataAcc[AllData.State.counter] = cRx;
		  AllData.Buf.rxDataAngAcc[AllData.State.counter] = cRx;
		  AllData.Buf.rxDataAng[AllData.State.counter] = cRx;
		  switch(cRx){
		  case 0x51:
//			  AllData.State.Flag = 1;
//			  ++AllData.State.counter;
//			  AllData.State.sum += cRx;

			  //不采集加速度
			  AllData.State.Flag = 0;
			  AllData.State.counter = 0;
			  AllData.State.sum = 0;
			  break;
		  case 0x52:
//			  AllData.State.Flag = 2;
//			  ++AllData.State.counter;
//			  AllData.State.sum += cRx;

			  //不采集角加速度
			  AllData.State.Flag = 0;
			  AllData.State.counter = 0;
			  AllData.State.sum = 0;
			  break;
		  case 0x53:
			  AllData.State.Flag = 3;
			  ++AllData.State.counter;
			  AllData.State.sum += cRx;
			  break;
		  default:
			  AllData.State.Flag = 0;
			  AllData.State.counter = 0;
			  AllData.State.sum = 0;
			  break;
		  }
	  }
	  else if(AllData.State.counter < 11){
		  switch (AllData.State.Flag){
		  case 1:
			  AllData.Buf.rxDataAcc[ AllData.State.counter] = cRx;
			  ++AllData.State.counter;
			  AllData.State.sum += cRx;
			  break;
		  case 2:
			  AllData.Buf.rxDataAngAcc[ AllData.State.counter] = cRx;
			  ++AllData.State.counter;
			  AllData.State.sum += cRx;
			  break;
		  case 3:
			  AllData.Buf.rxDataAng[ AllData.State.counter] = cRx;
			  ++AllData.State.counter;
			  AllData.State.sum += cRx;
			  break;
		  default:
			  AllData.State.Flag = 0;
			  AllData.State.counter = 0;
			  AllData.State.sum = 0;
			  break;
		  }
	  }
	  else if(AllData.State.counter == 11){
		  if(AllData.State.sum != cRx){
			  AllData.State.counter = 0;

		  }
		  else{
			  AllData.Buf.rxDataAcc[AllData.State.counter] = cRx;
			  AllData.Buf.rxDataAngAcc[AllData.State.counter] = cRx;
			  AllData.Buf.rxDataAng[AllData.State.counter] = cRx;
		  }
		  AllData.State.counter = 0;
		  AllData.State.sum = 0;

		  MPU6050ModData(AllData);//封装函数未测试
		  AllData.State.Flag = 0;
	  }
}
////MPU6050模块原数据采集------------------------------------------------------------------////END

////MPU6050模块数据解析------------------------------------------------------------------////BEG未测试
void MPU6050ModData(struct Data AllData){
	switch(AllData.State.Flag){
	case 1:
		AllData.Buf.Accx = (AllData.Buf.rxDataAcc[3]<<8)|AllData.Buf.rxDataAcc[2];
		AllData.Accx = (float) AllData.Buf.Accx/32768*16*g;
		AllData.Buf.Accy = (AllData.Buf.rxDataAcc[5]<<8)|AllData.Buf.rxDataAcc[4];
		AllData.Accy = (float) AllData.Buf.Accy/32768*16*g;
		AllData.Buf.Accz = (AllData.Buf.rxDataAcc[7]<<8)|AllData.Buf.rxDataAcc[6];
		AllData.Accz = (float) AllData.Buf.Accz/32768*16*g;
		break;
	case 2:
		AllData.Buf.AngAccx = (AllData.Buf.rxDataAngAcc[3]<<8)|AllData.Buf.rxDataAngAcc[2];
		AllData.AngAccx = (float) AllData.Buf.AngAccx/32768*2000;
		AllData.Buf.AngAccy = (AllData.Buf.rxDataAngAcc[5]<<8)|AllData.Buf.rxDataAngAcc[4];
		AllData.AngAccy = (float) AllData.Buf.AngAccy/32768*2000;
		AllData.Buf.AngAccz = (AllData.Buf.rxDataAngAcc[7]<<8)|AllData.Buf.rxDataAngAcc[6];
		AllData.AngAccz = (float) AllData.Buf.AngAccz/32768*2000;
		break;
	  case 3:
		AllData.Buf.Angx = (AllData.Buf.rxDataAng[3]<<8)|AllData.Buf.rxDataAng[2];
		AllData.Angx = (float) AllData.Buf.Angx/32768*180;
		AllData.Buf.Angy = (AllData.Buf.rxDataAng[5]<<8)|AllData.Buf.rxDataAng[4];
		AllData.Angy = (float) AllData.Buf.Angy/32768*180;
		AllData.Buf.Angz = (AllData.Buf.rxDataAng[7]<<8)|AllData.Buf.rxDataAng[6];
		AllData.Angz = (float) AllData.Buf.Angz/32768*180;
		break;
	  default:
		  break;
	}
}
////MPU6050模块数据解析------------------------------------------------------------------////END

////曲线拟合函数------------------------------------------------------------------////BEG未测试

/*==================polyfit(n,x,y,poly_n,a)===================*/
	// 拟合y=a0+a1*x+a2*x^2+……+apoly_n*x^poly_n
	// 是数据个数 xy是数据值 poly_n是多项式的项数
	// 返回a0,a1,a2,……a[poly_n]，系数比项数多一（常数项）
/*============================================================*/
void polyfit(int n,double x[],double y[],int poly_n,double p[])
{
	int i,j;
	double *tempx,*tempy,*sumxx,*sumxy,*ata;

	tempx = (double *)calloc(n , sizeof(double));
	sumxx = (double *)calloc((poly_n*2+1) , sizeof(double));
	tempy = (double *)calloc(n , sizeof(double));
	sumxy = (double *)calloc((poly_n+1) , sizeof(double));
	ata = (double *)calloc( (poly_n+1)*(poly_n+1) , sizeof(double) );
	for (i=0;i<n;i++)
	{
		tempx[i]=1;
		tempy[i]=y[i];
	}
	for (i=0;i<2*poly_n+1;i++)
	{
		for (sumxx[i]=0,j=0;j<n;j++)
		{
			sumxx[i]+=tempx[j];
			tempx[j]*=x[j];
		}
	}

	for (i=0;i<poly_n+1;i++)
	{
		for (sumxy[i]=0,j=0;j<n;j++)
		{
			sumxy[i]+=tempy[j];
			tempy[j]*=x[j];
		}
	}

	for (i=0;i<poly_n+1;i++)
	{
		for (j=0;j<poly_n+1;j++)
		{
			ata[i*(poly_n+1)+j]=sumxx[i+j];
		}
	}
	gauss_solve(poly_n+1,ata,p,sumxy);

	free(tempx);
	free(sumxx);
	free(tempy);
	free(sumxy);
	free(ata);
}

/*============================================================*/
	// 高斯消元法计算得到 n 次多项式的系数
	// n: 系数的个数
	// ata: 线性矩阵
	// sumxy: 线性方程组的Y值
	// p: 返回拟合的结果
/*============================================================*/
void gauss_solve(int n,double A[],double x[],double b[])
{
	int i,j,k,r;
	double max;
	for (k=0;k<n-1;k++)
	{
		max=fabs(A[k*n+k]);					// find maxmum
		r=k;
		for (i=k+1;i<n-1;i++)
		{
			if (max<fabs(A[i*n+i]))
			{
				max=fabs(A[i*n+i]);
				r=i;
			}
		}
		if (r!=k)
		{
			for (i=0;i<n;i++)		//change array:A[k]&A[r]
			{
				max=A[k*n+i];
				A[k*n+i]=A[r*n+i];
				A[r*n+i]=max;
			}

			max=b[k];                    //change array:b[k]&b[r]
			b[k]=b[r];
			b[r]=max;
		}

		for (i=k+1;i<n;i++)
		{
			for (j=k+1;j<n;j++)
				A[i*n+j]-=A[i*n+k]*A[k*n+j]/A[k*n+k];
			b[i]-=A[i*n+k]*b[k]/A[k*n+k];
		}
	}

	for (i=n-1;i>=0;x[i]/=A[i*n+i],i--)
	{
		for (j=i+1,x[i]=b[i];j<n;j++)
			x[i]-=A[i*n+j]*x[j];
	}
}

////曲线拟合函数------------------------------------------------------------------////END

/*/---------------------------------2022.8.9转移至AS5048A.c---------------------------------------//BEG
 *
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

////不设置零点位置，STM32芯片上电时先读取当前位置数据作为“零点”////BEG
void SPI_AS5048A_Origin_Value(void){
	SPI_TX_DATA[0] =  AS_Clear_Error_Flag;
	SPI_TX_DATA[1] =  AS_NOP;
	SPI_TX_DATA[2] =  AS_Read_Angle;
	SPI_TX_DATA[3] =  AS_NOP;

	for (i = 0; i<op_num; i++){
		AS_CS_Enable;
		HAL_SPI_TransmitReceive (&hspi1, (unsigned char *)&SPI_TX_DATA[i], (unsigned char *)&SPI_RX_DATA[i], 1, 100);
		AS_CS_Disable;
		HAL_Delay(1);
	}
	value_origin = SPI_RX_DATA[3]&0x3fff;
}
uint16_t SPI_AS5048A_Algorithm_ReadData(void){
	uint16_t TXD = 0;
	for (i = 0; i<op_num; i++){
	AS_CS_Enable;
	HAL_SPI_TransmitReceive(&hspi1, (unsigned char *)&SPI_TX_DATA[i], (unsigned char *)&SPI_RX_DATA[i], 1, 100);
	AS_CS_Disable;
	HAL_Delay(1);
	}

	if ( (SPI_RX_DATA[3]&0x3fff)>=value_origin ) post_process_value= (SPI_RX_DATA[3]&0x3fff)-value_origin;
	else post_process_value= 16384-value_origin+(SPI_RX_DATA[3]&0x3fff);

	TXD = (post_process_value&0xff00)>>8;
	//TXD = post_process_value&0x00ff;

	HAL_Delay(1);
	return post_process_value;
	//return TXD;
}
////不设置零点位置，STM32芯片上电时先读取当前位置数据作为“零点”////END
 *
//---------------------------------2022.8.9转移至AS5048A.c---------------------------------------//END*/

////printf输出------------------------------------------------------------------////BEG
#pragma region
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}
#pragma endregion
////////printf输出------------------------------------------------------------------////END

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

	//MotorMode = 0;

//  __disable_irq();
//  while (1)
//  {
//  }

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
