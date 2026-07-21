#include "sample.h"
#include "main.h"
#include "math.h"
#include "string.h"

extern uint16_t ADC1_Value[];

// 初始化RMS计算器
void RMS_Init(RMS_Calculator *calc) 
{
    memset(calc, 0, sizeof(RMS_Calculator));
}

// 更新采样值并计算有效值 (每0.1ms调用一次)
float RMS_Update(RMS_Calculator *calc, float newSample , uint16_t f) 
{
	uint16_t a = (ADC_SAMPLE_RATE + f / 2) / f;//四舍五入 
	
	if(a>=RMS_SAMPLE_COUNT)
	{
		a = RMS_SAMPLE_COUNT;
	}
    // 频率变化 → 强制重置缓冲区
    if(f != calc->lastFreq)
    {
        calc->lastFreq = f;       // 记录新频率
        calc->sumSquares = 0.0f;  // 清空平方和
        calc->sampleIndex = 0;    // 清空索引
    }
    
    // 累加平方和
    calc->sumSquares += newSample * newSample;
    calc->sampleBuffer[calc->sampleIndex] = newSample;
    calc->sampleIndex++;
    
    // 当累积到一周期的样本时，计算RMS并重置
    if(calc->sampleIndex >= a)
    {
        calc->lastRMS = sqrtf(calc->sumSquares / a);
        calc->sumSquares = 0.0f;
        calc->sampleIndex = 0;
        return calc->lastRMS;
    }
    
    // 周期未满时返回上一次的RMS值
    return calc->lastRMS;
}

float F=0;
//计算A相交流电流有效值(线电流)
float Calculate_ACCurrent_RMS_A(ST_ELEC_OBS *pstM,uint16_t f) 
{
    static RMS_Calculator rmsCalc3;
    static uint8_t isInitialized = 0;
    
    // 首次调用时初始化
    if (!isInitialized) 
	{
        RMS_Init(&rmsCalc3);
        isInitialized = 1;
    }
    
    // 获取当前瞬时值
    Cal_ACCurrent_A(pstM);
    F= pstM->fpPha1CrtFB;
    // 更新RMS计算并返回有效值
    return RMS_Update(&rmsCalc3,pstM->fpPha1CrtFB,f);
}
//交流侧电流采样A(线电流)
float offset_Current=0;
void Cal_ACCurrent_A(ST_ELEC_OBS *pstM)
{	
	static int count=0;

	if(count <=20000)//当检测有电压输入时取样20k次
	{
		offset_Current+=ADC1_Value[0]/20000;//采集的是平均值
		pstM->fpPha1CrtFB=(float)((ADC1_Value[0])*3.3f/4096.f/16.f-1.665f)/0.2f;     //原来是0.246f
		count++;
	}
	else
	{
		pstM->fpPha1CrtFB=(float)((ADC1_Value[0])*3.3f/4096.f/16.f-1.665f)/0.2f;
	}
}

//交流侧电流采样B(线电流)
float offset_Current1=0;
void Cal_ACCurrent_B(ST_ELEC_OBS *pstM)
{	
	static int count=0;

	if(count <=20000)//当检测有电压输入时取样20k次
	{
		offset_Current1+=ADC1_Value[1]/20000;//采集的是平均值
		pstM->fpPha2CrtFB=(float)((ADC1_Value[1])*3.3f/4096.f/16.f-1.66f)/0.2f;
		count++;
	}
	else
	{
		pstM->fpPha2CrtFB=(float)((ADC1_Value[1])*3.3f/4096.f/16.f-1.66f)/0.2f;
	}
}

//交流侧电流采样C(线电流)
float offset_Current2=0;
void Cal_ACCurrent_C(ST_ELEC_OBS *pstM)
{	
	static int count=0;

	if(count <=20000)//当检测有电压输入时取样20k次
	{
		offset_Current2+=ADC1_Value[2]/20000;//采集的是平均值
		pstM->fpPha3CrtFB=(float)((ADC1_Value[2])*3.3f/4096.f/16.f-1.66f)/0.2f;
		count++;
	}
	else
	{
		pstM->fpPha3CrtFB=(float)((ADC1_Value[2])*3.3f/4096.f/16.f-1.66f)/0.2f;
	}
}
//计算交流电压有效值 Uab
float Calculate_ACVoltage_RMS_AB(ST_ELEC_OBS *pstM,uint16_t f) 
{
    static RMS_Calculator rmsCalc1;
    static uint8_t isInitialized = 0;
    
    // 首次调用时初始化
    if (!isInitialized) 
	{
        RMS_Init(&rmsCalc1);
        isInitialized = 1;
    }
    
    // 获取当前瞬时值
    Cal_ACVolt_AB(pstM);
    
    // 更新RMS计算并返回有效值
    return RMS_Update(&rmsCalc1,pstM->fpABVolt,f);
}

//计算交流电压有效值 Ubc
float Calculate_ACVoltage_RMS_BC(ST_ELEC_OBS *pstM,uint16_t f) 
{
    static RMS_Calculator rmsCalc2;
    static uint8_t isInitialized = 0;
    
    // 首次调用时初始化
    if (!isInitialized) 
	{
        RMS_Init(&rmsCalc2);
        isInitialized = 1;
    }
    
    // 获取当前瞬时值
    Cal_ACVolt_BC(pstM);
    
    // 更新RMS计算并返回有效值
    return RMS_Update(&rmsCalc2,pstM->fpBCVolt,f);
}
//AB线电压	
float offset_volt=0;
void Cal_ACVolt_AB(ST_ELEC_OBS *pstM)
{
	static int count=0;

	if(count <=20000)//当检测有电压输入时取样20k次
	{
		offset_volt+=ADC2_Value[0]/20000;//采集的是平均值
		pstM->fpABVolt=(float)((ADC2_Value[0])*3.3f/4096.f/16.f-1.63f)*32.57f;
		count++;
	}
	else
	{
		pstM->fpABVolt=(float)((ADC2_Value[0])*3.3/4096.f/16.f-1.63f)*32.57f;
	}
}

//BC线电压	
float offset_volt_1=0;
void Cal_ACVolt_BC(ST_ELEC_OBS *pstM)
{
	static int count=0;

	if(count <=20000)//当检测有电压输入时取样20k次
	{
		offset_volt_1+=ADC2_Value[1]/20000;//采集的是平均值
		pstM->fpBCVolt=(float)(((ADC2_Value[1])*3.3/4096.f/16.f-1.63f)*32.5);
		count++;
	}
	else
	{
		pstM->fpBCVolt=(float)((ADC2_Value[1])*3.3/4096.f/16.f-1.63f)*32.5;
	}
}
 //根据Uab Ubc 计算相电压Ua Uc
void Calculate_PhaseVoltage(ST_ELEC_OBS *pstM)
{
	pstM->fpPhaAVoltFB = (2.f * pstM->fpABVolt + pstM->fpBCVolt)/3.f;
	pstM->fpPhaBVoltFB = (pstM->fpBCVolt - pstM->fpABVolt)/3.f;
	pstM->fpPhaCVoltFB = -(pstM->fpABVolt + 2*pstM->fpBCVolt)/3.f;
}

//直流母线线电压Vdc	
float offset_volt_2=0;
void Cal_DCVolt(ST_ELEC_OBS *pstM)
{
	static int count=0;

	if(count <=20000)//当检测有电压输入时取样20k次
	{
		offset_volt_2+=ADC1_Value[3]/20000;//采集的是平均值
		pstM->VDCPower=(float)(((ADC1_Value[3])*3.3/4096.f/16.f-1.628f)*47.5f);
		count++;
	}
	else
	{
		pstM->VDCPower=(float)((ADC1_Value[3])*3.3/4096.f/16.f-1.628f)*47.5f;
	}
}
