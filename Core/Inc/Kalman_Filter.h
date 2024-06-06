/*
 * Kalman_Filter.h
 *
 *  Created on: 2022年10月27日
 *      Author: MINI
 */

#ifndef INC_KALMAN_FILTER_H_
#define INC_KALMAN_FILTER_H_

//////卡尔曼滤波器参数////BEG
//typedef struct{
//    float LastP;//上次估算协方差 初始化值为0.02
//    float Now_P;//当前估算协方差 初始化值为0
//    float out;//卡尔曼滤波器输出 初始化值为0
//    float Kg;//卡尔曼增益 初始化值为0
//    float Q;//过程噪声协方差 初始化值为0.001 Q增大，动态响应变快，收敛稳定性变坏
//    float R;//观测噪声协方差 初始化值为0.543 R增大，动态响应变慢，收敛稳定性变好
//}Kal;//Kalman Filter parameter
//////卡尔曼滤波器参数////END
//
//extern Kal Kalman;

typedef struct{
	float LastP;//上次估算协方差 初始化值为0.02
	float NowP;//当前估算协方差 初始化值为0
	float Out;//卡尔曼滤波器输出 初始化值为0
	float Kg;//卡尔曼增益 初始化值为0
	float Q;//过程噪声协方差 初始化值为0.001
	float R;//观测噪声协方差 初始化值为0.543
}Kalman;
extern Kalman v_Kal;
extern Kalman t_Kal;

void KalmanFilter(Kalman *kfp,float input);

#endif /* INC_KALMAN_FILTER_H_ */
