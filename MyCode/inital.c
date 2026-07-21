#include "inital.h"
#include "main.h"

#define idmax 200
#define iqmax 200
#define uqmax 200
#define udmax 200

ST_ELEC_OBS input={0};//原始数据结构体
CLARK_STRUCT clark1;//克拉克变换结构体1（电压正负序分离）
CLARK_STRUCT clark2;//克拉克变换结构体2（电流正负序分离）

CHM_SIGNAL volt1={.frequence=50,.fpd=0,.fpd_1=0,.fpd_2=0,.fpU=0,.fpU_1=0,.fpU_2=0,.fpq=0,.fpq_1=0,.fpq_2=0,.theta=0};//电压
CHM_SIGNAL volt2={.frequence=50,.fpd=0,.fpd_1=0,.fpd_2=0,.fpU=0,.fpU_1=0,.fpU_2=0,.fpq=0,.fpq_1=0,.fpq_2=0,.theta=0};//电压

CHM_SIGNAL Crt1={.frequence=50,.fpd=0,.fpd_1=0,.fpd_2=0,.fpU=0,.fpU_1=0,.fpU_2=0,.fpq=0,.fpq_1=0,.fpq_2=0,.theta=0};//电压
CHM_SIGNAL Crt2={.frequence=50,.fpd=0,.fpd_1=0,.fpd_2=0,.fpU=0,.fpU_1=0,.fpU_2=0,.fpq=0,.fpq_1=0,.fpq_2=0,.theta=0};//电压	

PARK_STRUCT volt3={0};//电压park变换结构体1（电压正负序分离）
PARK_STRUCT volt4={0};//电压park变换结构体2（电压正负序分离）

PARK_STRUCT Crt3={0};//电流park变换结构体1（电流正负序分离）
PARK_STRUCT Crt4={0};//电流park变换结构体2（电流正负序分离）

PARK_STRUCT volt5={0};//电压反park变换结构体1（电压正负序分离）
PARK_STRUCT volt6={0};//电压反park变换结构体2（电压正负序分离）

CLARK_STRUCT clark3;//反克拉克变换结构体

SVPWM_STRUCT SVPWM;//SVPWM参数结构体

/*电压环结构体*/
ST_PID Udc_PID={.fpDes=0,.fpFB= 0,						 /*Des, FB*/
						  .fpKp=0.2,.fpKi=0.5, .fpKd=0,					 /*Kp, Ki, Kd*/ // Ki 1.5->0.5：抑制电压环低频极限环(电流幅值拍频包络)
						  .fpUp=0,.fpUi= 0,.fpUd= 0,					 /*Up, Ui, Ud*/
						  .fpE=0, .fpPreE=0,.fpSumE= 0,.fpU= 0,				 /*E, PreE, SumE, U*/
						  .fpUMax=200,.fpEpMax=udmax , .fpEiMax_p=udmax,.fpEiMax_n= udmax,.fpEdMax= udmax,.fpEMin= 0,
						  .fpUiMax=0, .fpDt=CTRL_DT};					 /*Dt*/

CHM_SIGNAL PLL={.frequence=50,.fpd=0,.fpd_1=0,.fpd_2=0,.fpU=0,.fpU_1=0,.fpU_2=0,.fpq=0,.fpq_1=0,.fpq_2=0,.theta=0}; ;//锁相环结构体

ST_PID PLL_PID={.fpDes=0,.fpFB= 0,						 /*Des, FB*/
						  .fpKp=0.5,.fpKi=0.35, .fpKd=0,					 /*Kp, Ki, Kd*/
						  .fpUp=0,.fpUi= 0,.fpUd= 0,					 /*Up, Ui, Ud*/
						  .fpE=0, .fpPreE=0,.fpSumE= 0,.fpU= 0,				 /*E, PreE, SumE, U*/
						  .fpUMax=200,.fpEpMax=udmax , .fpEiMax_p=udmax,.fpEiMax_n= udmax,.fpEdMax= udmax,.fpEMin= 0,
						  .fpUiMax=0, .fpDt=CTRL_DT};					 /*Dt*/

/*电流环结构体*/
ST_PID Id_PID1={.fpDes=0,.fpFB= 0,						 /*Des, FB*/
						  .fpKp=4,.fpKi=10, .fpKd=0,					 /*Kp, Ki, Kd*/
						  .fpUp=0,.fpUi= 0,.fpUd= 0,					 /*Up, Ui, Ud*/
						  .fpE=0, .fpPreE=0,.fpSumE= 0,.fpU= 0,				 /*E, PreE, SumE, U*/
						  .fpUMax=200,.fpEpMax=udmax , .fpEiMax_p=udmax,.fpEiMax_n= udmax,.fpEdMax= udmax,.fpEMin= 0,
						  .fpUiMax=0, .fpDt=CTRL_DT};					 /*Dt*/
ST_PID Iq_PID1={.fpDes=0,.fpFB= 0,						 /*Des, FB*/
						  .fpKp=4,.fpKi=10, .fpKd=0,					 /*Kp, Ki, Kd*/
						  .fpUp=0,.fpUi= 0,.fpUd= 0,					 /*Up, Ui, Ud*/
						  .fpE=0, .fpPreE=0,.fpSumE= 0,.fpU= 0,				 /*E, PreE, SumE, U*/
						  .fpUMax=200,.fpEpMax=udmax , .fpEiMax_p=udmax,.fpEiMax_n= udmax,.fpEdMax= udmax,.fpEMin= 0,
						  .fpUiMax=0, .fpDt=CTRL_DT};					 /*Dt*/
ST_PID Id_PID2={.fpDes=0,.fpFB= 0,						 /*Des, FB*/
						  .fpKp=4,.fpKi=10, .fpKd=0,					 /*Kp, Ki, Kd*/
						  .fpUp=0,.fpUi= 0,.fpUd= 0,					 /*Up, Ui, Ud*/
						  .fpE=0, .fpPreE=0,.fpSumE= 0,.fpU= 0,				 /*E, PreE, SumE, U*/
						  .fpUMax=200,.fpEpMax=udmax , .fpEiMax_p=udmax,.fpEiMax_n= udmax,.fpEdMax= udmax,.fpEMin= 0,
						  .fpUiMax=0, .fpDt=CTRL_DT};					 /*Dt*/
ST_PID Iq_PID2={.fpDes=0,.fpFB= 0,						 /*Des, FB*/
						  .fpKp=4,.fpKi=10, .fpKd=0,					 /*Kp, Ki, Kd*/
						  .fpUp=0,.fpUi= 0,.fpUd= 0,					 /*Up, Ui, Ud*/
						  .fpE=0, .fpPreE=0,.fpSumE= 0,.fpU= 0,				 /*E, PreE, SumE, U*/
						  .fpUMax=200,.fpEpMax=udmax , .fpEiMax_p=udmax,.fpEiMax_n= udmax,.fpEdMax= udmax,.fpEMin= 0,
						  .fpUiMax=0, .fpDt=CTRL_DT};					 /*Dt*/


