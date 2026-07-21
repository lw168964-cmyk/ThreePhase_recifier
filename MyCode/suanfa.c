#include "suanfa.h"
#include "arm_math.h"
#include "inital.h"
float b=0;
float k=1.414;//广义二阶积分阻尼系数
/*结构体初始化：1.结构体类型 2.基波频率 3.采样频率*/
void fixed_angle_init(FixedangleGenerator *angle_gen,float Hz,float sample_rate)   
{
	 angle_gen->theta=0;
	 angle_gen->Ts=1.0f/sample_rate;
	 angle_gen->w=2*3.1415926f*Hz;
	 angle_gen->Fo=Hz;
}

/*更新结构体，并返回该结构体*/
float fixed_angle_update(FixedangleGenerator *angle_gen)
{
	//更新电角度
	angle_gen->theta+= angle_gen->w*angle_gen->Ts;
	
	//角度归一化
	if(angle_gen->theta>=2*3.1415926f)
	{
		angle_gen->theta-=2*3.1415926f ;
	}
	return angle_gen->theta ;
}
/*更新结构体*/
void fixed_angle_update_1(FixedangleGenerator *angle_gen)
{
	//更新电角度
	angle_gen->theta+= angle_gen->w*angle_gen->Ts;
	
	//角度归一化
	if(angle_gen->theta>=2*PI)
	{
		angle_gen->theta-=2*PI ;
	}
}

float my_clip(float fpValue, float fpMin, float fpMax)    //输出钳位
{
	if (fpValue <= fpMin)
	{
		return fpMin;
	}
	else if (fpValue >= fpMax)
	{
		return fpMax;
	}
	else
	{
		return fpValue;
	}
}

//PI控制器（带抗积分饱和）
void PI_Controller(ST_PID *pstPid)
{
	pstPid->fpE = pstPid->fpDes - pstPid->fpFB; //计算当前偏差
	pstPid->fpUp = my_clip(pstPid->fpKp * pstPid->fpE, -pstPid->fpEpMax, pstPid->fpEpMax);//Kp限幅

	// 抗积分饱和：仅当输出未饱和时才累加积分
	float sumE_new = pstPid->fpSumE + pstPid->fpE;
	float Ui_new = pstPid->fpKi * pstPid->fpDt * sumE_new;
	float U_test = pstPid->fpUp + Ui_new;

	if (U_test <= pstPid->fpUMax && U_test >= -pstPid->fpUMax) {
		pstPid->fpSumE = sumE_new; // 未饱和，允许积分累加
	}
	// 已饱和则fpSumE保持不变，停止积分

	pstPid->fpUi = pstPid->fpKi * pstPid->fpDt * pstPid->fpSumE;
	pstPid->fpU = my_clip(pstPid->fpUp + pstPid->fpUi, -pstPid->fpUMax, pstPid->fpUMax);
	pstPid->fpPreE = pstPid->fpE; //保存本次偏差
}

//克拉克变换结构体初始化
void Clark_Init(CLARK_STRUCT *p)
{
    p->ua = 0;
    p->ub = 0;
    p->uc = 0;
    p->alpha = 0;
    p->beta = 0;
    p->u0 = 0;
}
//克拉克变换
void Clark_Func(CLARK_STRUCT *p, float ua, float ub, float uc, int sele)
{
    //计算对应坐标值
    const float alpha_base = 0.6666667f*(ua - 0.5f * ub - 0.5f * uc);
    const float beta_base = 0.6666667f*(ub - uc) * SQRT3_OVER_2;

		//A和alpha重合
    if (sele == 1)
    {
        p->alpha = alpha_base;
        p->beta = beta_base;
    }
    else
    {
        p->alpha = -beta_base; 
        p->beta = alpha_base;
    }
}
//反克拉克变换
/* alpha beta 0 to abc */
void iClark_Func(CLARK_STRUCT *p, float alpha, float beta, float u0, int sele)
{
    if (sele == 1)
    {
        /* A & alpha */
        p->ua = (alpha + u0);
        p->ub = (-0.5f * alpha + 0.866f * beta + u0);
        p->uc = (-0.5f * alpha - 0.866f * beta + u0);
    }
    else
    {
        /* A & alpha - 90 */
        p->ua = (beta + u0);
        p->ub = (-0.866f * alpha - 0.5f * beta + u0);
        p->uc = (0.866f * alpha - 0.5f * beta + u0);
    }
}
//park变换结构体初始化
void Park_Init(PARK_STRUCT *p)
{
    p->alpha = 0;
    p->beta = 0;
    p->theta = 0;
    p->ud = 0;
    p->uq = 0;
}

//park变换
/* alpha beta 0 to dq0 */
void Park_Func(PARK_STRUCT *p, float alpha, float beta, float theta, int park_sele)
{

    float p_cos = arm_cos_f32(theta);
    float p_sin = arm_sin_f32(theta);
    if (park_sele == 1)
    {
        /* cos */
        p->ud = p_cos * alpha + p_sin * beta;
        p->uq = p_cos * beta - p_sin * alpha;
    }
    else
    {
        /* sin */
        p->ud = p_sin * alpha - p_cos * beta;
        p->uq = p_cos * alpha + p_sin * beta;
    }
}

//反park变换
/* dq0 to alpha beta 0 */
void iPark_Func(PARK_STRUCT *p, float ud, float uq, float theta, int park_sele)
{
    float p_cos = arm_cos_f32(theta);
    float p_sin = arm_sin_f32(theta);
    if (park_sele == 1)
    {
        /* cos */
        p->alpha = p_cos * ud - p_sin * uq;
        p->beta = p_sin * ud + p_cos * uq;
    }
    else
    {
        /* sin */
        p->alpha = p_sin * ud + p_cos * uq;
        p->beta = -p_cos * ud + p_sin * uq;
    }
}

//abc to dq 结构体初始化
void abc_dq0_Init(ABC_DQ0_STRUCT *p)
{
    p->ua = 0;
    p->ub = 0;
    p->uc = 0;
    p->ud = 0;
    p->uq = 0;
    p->u0 = 0;
    p->theta = 0;
}

/* abc to dq0 */
void abcTodq0_Func(ABC_DQ0_STRUCT *p, float ua, float ub, float uc, float theta, int sele)
{
    float p_sin0 = arm_sin_f32(theta);
    float p_sin1 = arm_sin_f32(theta - 2.0944f);
    float p_sin2 = arm_sin_f32(theta + 2.0944f);
    float p_cos0 = arm_cos_f32(theta);
    float p_cos1 = arm_cos_f32(theta - 2.0944f);
    float p_cos2 = arm_cos_f32(theta + 2.0944f);

    if (sele == 1)
    {
        /* sin */
        p->ud = 0.6667f * (p_sin0 * ua + p_sin1 * ub + p_sin2 * uc);
        p->uq = 0.6667f * (p_cos0 * ua + p_cos1 * ub + p_cos2 * uc);
        p->u0 = 0.3333f * (ua + ub + uc);
    }
    else
    {
        /* cos */
        p->ud = 0.6667f * (p_cos0 * ua + p_cos1 * ub + p_cos2 * uc);
        p->uq = -0.6667f * (p_sin0 * ua + p_sin1 * ub + p_sin2 * uc);
        p->u0 = 0.3333f * (ua + ub + uc);
    }
}

/* dq0 to abc */
void dq0Toabc_Func(ABC_DQ0_STRUCT *p, float ud, float uq, float u0, float theta, int sele)
{
    float p_sin0 = arm_sin_f32(theta);
    float p_sin1 = arm_sin_f32(theta - 2.0944f);
    float p_sin2 = arm_sin_f32(theta + 2.0944f);
    float p_cos0 = arm_cos_f32(theta);
    float p_cos1 = arm_cos_f32(theta - 2.0944f);
    float p_cos2 = arm_cos_f32(theta + 2.0944f);

    if (sele == 1)
    {
        /* sin */
        p->ua = (p_cos0 * ud - p_sin0 * uq + u0);
        p->ub = (p_cos1 * ud - p_sin1 * uq + u0);
        p->uc = (p_cos2 * ud - p_sin2 * uq + u0);
    }
    else
    {  
			  /* cos */
			  p->ua = (p_sin0 * ud + p_cos0 * uq + u0);
        p->ub = (p_sin1 * ud + p_cos1 * uq + u0);
        p->uc = (p_sin2 * ud + p_cos2 * uq + u0);
        
    }
}

/* 广义二阶积分 */
void SOGI(volatile CHM_SIGNAL*pll)
{

	float omi=2*PI*pll->frequence;//工作频率
	pll->fpx = 2*k*omi*CTRL_DT;
	pll->fpy = omi*omi*CTRL_DT*CTRL_DT;
	pll->fplam = 0.5f*omi*CTRL_DT;
	pll->fpa = pll->fpx/(pll->fpx+pll->fpy+4);
	pll->fpb = (8-2*pll->fpy)/(pll->fpx+pll->fpy+4);
	pll->fpc = (pll->fpx-pll->fpy-4)/(pll->fpx+pll->fpy+4);

	pll->fpd = pll->fpa * pll->fpU - pll->fpa * pll->fpU_2 + pll->fpb * pll->fpd_1 + pll->fpc * pll->fpd_2;
	pll->fpq = pll->fpb * pll->fpq_1 + pll->fpc * pll->fpq_2 + pll->fplam * pll->fpa * (pll->fpU + 2*pll->fpU_1 + pll->fpU_2);
	pll->fpU_2 = pll->fpU_1;
	pll->fpU_1 = pll->fpU;
	pll->fpd_2 = pll->fpd_1;
	pll->fpd_1 = pll->fpd;
	pll->fpq_2 = pll->fpq_1;
	pll->fpq_1 = pll->fpq;

}

/* PLL锁相环 */
void PLL_Control(CHM_SIGNAL*signal)
{
	    // 控制q轴为0
	    // 修正：正相序正向旋转(ABC)，theta应正向积分(+=)，且鉴相反馈符号取+Vq
	    // 推导：uq=V*sin(θ_true−θ_est)，若θ_est滞后则uq>0，需增大ω让θ追上→正反馈
	    PLL_PID.fpDes = 0.0f;
	    PLL_PID.fpFB = -signal->Vac_q;  // 反号：让fpU=Kp*(0-(-Vq))=+Kp*Vq实现正反馈
	    PI_Controller(&PLL_PID);

	    // 前馈 + PI输出得到角频率
	    float omega = 100.0f * PI + PLL_PID.fpU;
	    signal->frequence = omega / (2.0f * PI);

	    // 积分得到角度 —— 修正为正向积分，与Park的+wt一致
	    signal->theta += omega * CTRL_DT;  // 原来是 -=，导致PLL永远无法锁定

	    // 角度归一化
	    while (signal->theta > 2.0f * PI)
	    {
	        signal->theta -= 2.0f * PI;
	    }
	    while (signal->theta < 0.0f)
	    {
	        signal->theta += 2.0f * PI;
	    }
}

void fcn(float p0, float edp, float edn, float eqp, float eqn,
         float *idp, float *idn, float *iqp, float *iqn)
{
    const float K = 2*1.732f / 32.0f;  
    float k;
    float denominator;
    
    // 计算分母：edp^2 + eqp^2 - edn^2 - eqn^2
    denominator = edp*edp + eqp*eqp - edn*edn - eqn*eqn;
    
    // 计算 k = (2/3) / denominator * p0
    // 注意：需要避免除零错误
    if (fabsf(denominator) < 1e-12f) {
        // 分母接近0时的处理，可根据实际需求调整
        k = 0.0f;
    } else {
        k = (2.0f / 3.0f) / denominator * p0;
    }
    
    // 限幅：-K <= k <= K
    if (k > K) {
        k = K;
    }
    if (k < -K) {
        k = -K;
    }
    
    // 计算输出电流
    *idp = k * edp;
    *idn = -k * edn;
    *iqp = k * eqp;
    *iqn = -k * eqn;
}

