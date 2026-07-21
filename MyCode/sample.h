#ifndef __SAMPLE_H	
#define __SAMPLE_H

#include "main.h"
typedef float fp32;
typedef double fp64;

#define ADC_SAMPLE_RATE  10000    // 采样率 10kHz
#define RMS_SAMPLE_COUNT 500  // 20ms周期/0.1ms采样间隔 = 200个采样点


typedef struct
{
	fp32 fpABVolt;			 //AB线电压
	fp32 fpBCVolt;			 //BC线电压
	fp32 fpACCVolt;			 //AC线电压
	
	fp32 fpPhaAVoltFB;    //A相电压反馈
	fp32 fpPhaBVoltFB;    //B相电压反馈
	fp32 fpPhaCVoltFB;    //C相电压反馈

	fp32 fpPhaAVoltRMS;    //A相电压反馈有效值
	fp32 fpPhaBVoltRMS;    //B相电压反馈有效值
	fp32 fpPhaCVoltRMS;    //C相电压反馈有效值
	
	fp32 fpPha1CrtFB;          //A相电流反馈
	fp32 fpPha2CrtFB;          //B相电流反馈
	fp32 fpPha3CrtFB;          //C相电流反馈
	
	fp32 VDCPower;			     //直流母线电压
	
} ST_ELEC_OBS;


//有效值计算结构体
typedef struct {
    float sampleBuffer[RMS_SAMPLE_COUNT];  // 存储一个周期的采样值
    uint16_t sampleIndex;                  // 当前采样位置
    float sumSquares;                      // 平方累加和
    uint8_t isBufferFull;                  // 缓冲区已满标志
	uint16_t lastFreq;  // 保存上一次频率，用于检测变化
	float lastRMS;     // 保存上一次的RMS值
} RMS_Calculator;


void RMS_Init(RMS_Calculator *calc);// 初始化RMS计算器
float RMS_Update(RMS_Calculator *calc, float newSample , uint16_t f) ;// 更新采样值并计算有效值


float Calculate_ACCurrent_RMS_A(ST_ELEC_OBS *pstM,uint16_t f);//计算A相线电流有效值

float Calculate_ACVoltage_RMS_AB(ST_ELEC_OBS *pstM,uint16_t f); //计算交流电压有效值 Uab
float Calculate_ACVoltage_RMS_BC(ST_ELEC_OBS *pstM,uint16_t f); //计算交流电压有效值 Ubc


void Cal_ACCurrent_A(ST_ELEC_OBS *pstM);//A相交流电流瞬时值
void Cal_ACCurrent_B(ST_ELEC_OBS *pstM);//B相交流电流瞬时值
void Cal_ACCurrent_C(ST_ELEC_OBS *pstM);//C相交流电流瞬时值

void Cal_ACVolt_AB(ST_ELEC_OBS *pstM);//AB线电压瞬时值采样
void Cal_ACVolt_BC(ST_ELEC_OBS *pstM);//BC线电压瞬时值采样

void Calculate_PhaseVoltage(ST_ELEC_OBS *pstM);  //根据Uab Ubc 计算相电压Ua Uc

void Cal_DCVolt(ST_ELEC_OBS *pstM);//直流母线电压瞬时值采样
	
#endif


