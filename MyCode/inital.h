#ifndef __INITAL_H
#define __INITAL_H
#include "suanfa.h"
#include "svpwm.h"
#include "sample.h"

extern ST_ELEC_OBS input;//原始数据结构体

extern CLARK_STRUCT clark1;//克拉克变换结构体1（电压正负序分离）
extern CLARK_STRUCT clark2;//克拉克变换结构体2（电流正负序分离）

extern CLARK_STRUCT clark3;//反克拉克变换结构体

extern CHM_SIGNAL volt1;//电压广义二阶积分结构体1
extern CHM_SIGNAL volt2;//电压广义二阶积分结构体2

extern CHM_SIGNAL Crt1;//电流广义二阶积分结构体1
extern CHM_SIGNAL Crt2;//电流广义二阶积分结构体2

extern PARK_STRUCT volt3;//park变换结构体1（电压正负序分离）
extern PARK_STRUCT volt4;//park变换结构体2（电压正负序分离）

extern PARK_STRUCT volt5;//电压反park变换结构体1（电压正负序分离）
extern PARK_STRUCT volt6;//电压反park变换结构体2（电压正负序分离）

extern PARK_STRUCT Crt3;//电流park变换结构体1（电流正负序分离）
extern PARK_STRUCT Crt4;//电流park变换结构体2（电流正负序分离）

extern ST_PID Udc_PID;//电压环结构体
extern ST_PID PLL_PID;//锁相环结构体
extern CHM_SIGNAL PLL;//锁相环结构体

extern ST_PID Id_PID1;//电流环结构体
extern ST_PID Iq_PID1;//电流环结构体

extern ST_PID Id_PID2;//电流环结构体
extern ST_PID Iq_PID2;//电流环结构体

extern SVPWM_STRUCT SVPWM;//SVPWM参数结构体

#endif 
