#include "Kalman_Filter.h"

Kalman v_Kal;
Kalman t_Kal;

////卡尔曼滤波器////BEG
void KalmanFilter(Kalman *kfp,float input)
{
    //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
    kfp->NowP = kfp->LastP + kfp->Q;
    //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
    kfp->Kg = kfp->NowP / (kfp->NowP + kfp->R);
    //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
    kfp->Out = kfp->Out + kfp->Kg * (input -kfp->Out);//因为这一次的预测值就是上一次的输出值
    //更新协方差方程: 本次的系统协方差付给 kfp->LastP 威下一次运算准备。
    kfp->LastP = (1-kfp->Kg) * kfp->NowP;
    //return kfp->Out;
}

//float kalmanFilter(Kal kfp,float input)
//{
//    //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
//    kfp.Now_P = kfp.LastP + kfp.Q;
//    //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
//    kfp.Kg = kfp.Now_P / (kfp.Now_P + kfp.R);
//    //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
//    kfp.out = kfp.out + kfp.Kg * (input -kfp.out);//因为这一次的预测值就是上一次的输出值
//    //更新协方差方程: 本次的系统协方差付给 kfp->LastP 威下一次运算准备。
//    kfp.LastP = (1-kfp.Kg) * kfp.Now_P;
//    return kfp.out;
//}
////卡尔曼滤波器////END
