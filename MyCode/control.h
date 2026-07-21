#ifndef __CONTROL_H

#define __CONTROL_H
#include "main.h"
extern float Udc;//直流母线电压

extern float Ia_rms;//A想线电流有效值

extern float Uab_rms;//AB线电压有效值

void Volt_Loop_Control(float Udc_def); //单电压环控制

#endif
