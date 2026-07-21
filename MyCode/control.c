#include "control.h"
#include "tim.h"
#include "main.h"
#include "arm_math.h"
#include "sample.h"
#include "inital.h"
#include "suanfa.h"


float Udc = 60.0f;//直流母线电压
float Uab_rms=0;//AB线电压有效值
float Ia_rms=0;//A相线电流有效值

float Idp=0;//正序电流d轴参考值
float Iqp=0;//正序电流q轴参考值
float Idn=0;//负序电流d轴参考值
float Iqn=0;//负序电流q轴参考值

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)  //中断回调 10k
{
	 if(htim==&htim6)
	 {
		Volt_Loop_Control(50); 
	 }
}

void Volt_Loop_Control(float Udc_def) //电压环加电流环正负序分离解耦控制
{
	 // 软启动：电压给定逐步上升到目标值，防止启动过冲
	 static float Udc_ref_ramp = 0.0f;
	 if (Udc_ref_ramp < Udc_def) {
		 Udc_ref_ramp += Udc_def / (2.0f * 10000.0f); // 2秒线性斜坡，10kHz控制频率
		 if (Udc_ref_ramp > Udc_def) Udc_ref_ramp = Udc_def; // 防止超调
	 }

	 Cal_DCVolt(&input);//获取直流电压有效值

	 Udc_PID.fpDes = Udc_ref_ramp; // 使用斜坡给定而非阶跃给定
	 Udc_PID.fpFB = input.VDCPower;
	 PI_Controller(&Udc_PID);

	 // 整流器功率参考必须 >=0，限制电压环输出下限为0，防止逆变/放电
	 float Udc_output = (Udc_PID.fpU > 0.0f) ? Udc_PID.fpU : 0.0f;
	 float P0 = Udc_ref_ramp * Udc_output; // 使用当前斜坡值计算功率
	
	 Cal_ACVolt_AB(&input);//获取AB线电压瞬时值
	 Cal_ACVolt_BC(&input);//获取BC线电压瞬时值
	 Calculate_PhaseVoltage(&input);//还原三个相电压
	
	 Cal_ACCurrent_A(&input);//获取A相电流瞬时值
	 Cal_ACCurrent_B(&input);//获取B相电流瞬时值
	 Cal_ACCurrent_C(&input);//获取B相电流瞬时值
	
	 Clark_Func(&clark1 , input.fpPhaAVoltFB , input.fpPhaBVoltFB , input.fpPhaCVoltFB , 1);//电压克拉克变换
	
	 volt1.fpU = 0.5f*clark1.alpha;
	 volt2.fpU = 0.5f*clark1.beta;
	 SOGI(&volt1);
	 SOGI(&volt2);
	
	 
	 float wt=PLL.theta;//通过PLL获取
	 Park_Func(&volt3 , volt1.fpd-volt2.fpq , volt1.fpq+volt2.fpd , wt , 1);//正序电压park变换，和A轴重合
	 Park_Func(&volt4 , volt1.fpd+volt2.fpq , volt2.fpd-volt1.fpq , -wt , 1);//负序电压Park变换，和A轴重合
	
	 PLL.Vac_q = volt3.uq;
	 PLL_Control(&PLL);//PLL锁相环
	 
	 Clark_Func(&clark2 , input.fpPha1CrtFB , input.fpPha2CrtFB , input.fpPha3CrtFB , 1);//电流克拉克变换
	 
	 Crt1.fpU = 0.5f*clark2.alpha;
	 Crt2.fpU = 0.5f*clark2.beta;
	 
	 SOGI(&Crt1);
	 SOGI(&Crt2);
	 
	 Park_Func(&Crt3 , Crt1.fpd-Crt2.fpq , Crt1.fpq+Crt2.fpd , wt , 1); //正序电流park变换，和A轴重合
	 Park_Func(&Crt4 , Crt1.fpd+Crt2.fpq , Crt2.fpd-Crt1.fpq , -wt , 1);//负序电流park变换，和A轴重合
	 
	 
	 fcn(P0 , volt3.ud , volt4.ud , volt3.uq , volt4.uq , &Idp , &Idn , &Iqp , &Iqn);//计算电流参考值
	 
	 //电流环
	 Id_PID1.fpDes = Idp ;
	 Id_PID1.fpFB = Crt3.ud;
	 PI_Controller(&Id_PID1);
	 
	 Iq_PID1.fpDes = Iqp;
	 Iq_PID1.fpFB = Crt3.uq;
	 PI_Controller(&Iq_PID1);
	 
//	 volt5.ud=volt3.ud-Id_PID1.fpU+100*PI*0.002*Crt3.uq;
//	 volt5.uq=volt3.uq-Iq_PID1.fpU-100*PI*0.002*Crt3.ud;

	 Id_PID2.fpDes = Idn;
	 Id_PID2.fpFB = Crt4.ud;
	 PI_Controller(&Id_PID2);
	 
	 Iq_PID2.fpDes = Iqn;
	 Iq_PID2.fpFB = Crt4.uq;
	 PI_Controller(&Iq_PID2);

	 // 调制电压 = 电压前馈 - 电流环PI输出 + 交叉解耦项
	 // PI误差=（参考-反馈），并网整流器控制律 Vconv = Vgrid + ωL·iq - PI，故为减号（此为正确形式）
	 iPark_Func(&volt5 , volt3.ud-Id_PID1.fpU+100.0f*PI*0.002f*Crt3.uq , volt3.uq-Iq_PID1.fpU-100.0f*PI*0.002f*Crt3.ud , wt , 1);//正序
	 iPark_Func(&volt6 , volt4.ud-Id_PID2.fpU+100.0f*PI*0.002f*Crt4.uq , volt4.uq-Iq_PID2.fpU-100.0f*PI*0.002f*Crt4.ud , -wt , 1);//负序


	 iClark_Func(&clark3 , (volt5.alpha+volt6.alpha)/Udc_def , (volt5.beta+volt6.beta)/Udc_def , 0 , 1);//反克拉克变换得到参考波
//	 printf_DMA("%f,%f",input.fpPhaAVoltFB,clark3.ua);
	 my_svpwm_calc(&SVPWM , (volt5.alpha+volt6.alpha)/Udc_def , (volt5.beta+volt6.beta)/Udc_def);
	
	
}

