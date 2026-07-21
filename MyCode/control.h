#ifndef __CONTROL_H

#define __CONTROL_H
#include "main.h"
extern float Udc;//直流母线电压

extern float UDC;//直流母线电压测量值
extern float Ia_rms;//A想线电流有效值

extern float Uab_rms;//AB线电压有效值

extern float Iq_comp;//正序q轴无功补偿电流(A)，现场按功率表var最小标定以提高PF

void Volt_Loop_Control(float Udc_def); //单电压环控制

void SoftStart_Reset(void); //请求重新软启动（开启PWM/定时中断前调用）

#endif
