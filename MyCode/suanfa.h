#ifndef __SUANFA_H
#define __SUANFA_H

#define SQRT3_OVER_2 0.86602540378f // v3/2

#include "arm_math.h"

typedef float fp32;
typedef double fp64;

#define CTRL_FREQUENCY 10000   //控制频率
#define CTRL_DT (fp32)(1.0f / CTRL_FREQUENCY)//控制周期

/*PID控制器结构体*/
typedef struct
{
	fp32 fpDes; //控制变量目标值
	fp32 fpFB;	//控制变量反馈值

	fp32 fpKp; //比例系数Kp
	fp32 fpKi; //积分系数Ki
	fp32 fpKd; //微分系数Kd

	fp32 fpUp; //比例输出
	fp32 fpUi; //积分输出
	fp32 fpUd; //微分输出

	fp32 fpE;	 //本次偏差
	fp32 fpPreE; //上次偏差
	fp32 fpSumE; //总偏差
	fp32 fpU;	 //本次PID运算结果

	fp32 fpUMax;	//PID运算后输出最大值及做遇限削弱时的上限值
	fp32 fpEpMax;	//比例项输出最大值
	fp32 fpEiMax_p; //积分项输出最大值//positive > 0
	fp32 fpEiMax_n; //积分项输出最大值//negative < 0
	fp32 fpEdMax;	//微分项输出最大值
	fp32 fpEMin;	//积分上限

	fp32 fpUiMax;

	fp32 fpDt; //控制周期
}ST_PID;

//电角度结构体
typedef struct   		
{
	
	float theta;   		//电角度
	float w;       		//角频率
	float Ts;      		//采样时间
	uint16_t Fo;			//目标频率

} FixedangleGenerator ;


//克拉克变换结构体
typedef struct
{
    float ua;
    float ub;
    float uc;
    float alpha;
    float beta;
    float u0;
} CLARK_STRUCT;

//park变换结构体
typedef struct
{
    float alpha;
    float beta;
    float ud;
    float uq;
    float theta;
} PARK_STRUCT;

//abc to dq结构体
typedef struct
{
    float ua;
    float ub;
    float uc;
    float ud;
    float uq;
    float u0;
    float theta;
} ABC_DQ0_STRUCT;

typedef struct    	//广义二阶积分参数结构体
{
    // 输入
    fp32 fpU;     	// 当前输入值
    fp32 fpU_1;   	// 上一次输入值
    fp32 fpU_2;   	// 上上次输入值
                    
    // d轴输出
    fp32 fpd;     	// d轴当前输出（同相分量）
    fp32 fpd_1;   	// d轴上一次输出
    fp32 fpd_2;   	// d轴上上次输出
			              
    // q轴输出
    fp32 fpq;     	// q轴当前输出（正交分量）
    fp32 fpq_1;   	// q轴上一次输出
    fp32 fpq_2;   	// q轴上上次输出
                    
    // 滤波器系数
    fp32 fpa;     	// 系数a
    fp32 fplam;   	// 系数λ
    fp32 fpb;     	// 系数b
    fp32 fpc;     	// 系数c
    
    // 中间变量
    fp32 fpx;				
    fp32 fpy;				
    
    // 控制参数
    fp32 frequence; // 目标频率
    float Vac_q;
    float theta;
} CHM_SIGNAL;


/*结构体初始化：1.结构体类型 2.基波频率 3.采样频率*/
void fixed_angle_init(FixedangleGenerator *angle_gen,float Hz,float sample_rate);
/*更新结构体，并返回该结构体*/
float fixed_angle_update(FixedangleGenerator *angle_gen);
/*更新结构体*/
void fixed_angle_update_1(FixedangleGenerator *angle_gen);
//输出限幅
float my_clip(float fpValue, float fpMin, float fpMax);
//PI控制器
void PI_Controller(ST_PID *pstPid);
//克拉克结构体初始化
void Clark_Init(CLARK_STRUCT *p);
//克拉克变换
void Clark_Func(CLARK_STRUCT *p, float ua, float ub, float uc, int sele);
//反克拉克变换
void iClark_Func(CLARK_STRUCT *p, float alpha, float beta, float u0, int sele);
//park变换结构体初始化
void Park_Init(PARK_STRUCT *p);
//park变换
/* alpha beta 0 to dq0 */
void Park_Func(PARK_STRUCT *p, float alpha, float beta, float theta, int park_sele);
//反park变换
/* dq0 to alpha beta 0 */
void iPark_Func(PARK_STRUCT *p, float ud, float uq, float theta, int park_sele);
//abc to dq 结构体初始化
void abc_dq0_Init(ABC_DQ0_STRUCT *p);
/* abc to dq0 */
void abcTodq0_Func(ABC_DQ0_STRUCT *p, float ua, float ub, float uc, float theta, int sele);
/* dq0 to abc */
void dq0Toabc_Func(ABC_DQ0_STRUCT *p, float ud, float uq, float u0, float theta, int sele);
/*SOGI广义二阶积分*/
void SOGI(volatile CHM_SIGNAL*pll);
/*PLL锁相环*/
void PLL_Control(CHM_SIGNAL*signal);
/*坐标值合并函数*/
void fcn(float p0, float edp, float edn, float eqp, float eqn,
         float *idp, float *idn, float *iqp, float *iqn);

#endif
