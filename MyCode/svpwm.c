#include "svpwm.h"
#include "hrtim.h"

void my_svpwm_Init(SVPWM_STRUCT *p)
{
    p->N = 0;
    p->na = 0;
    p->nb = 0;
    p->nc = 0;
    p->alpha = 0;
    p->beta = 0;
    p->ts = 0;
    p->t1 = 0;
    p->t2 = 0;
    p->t0 = 0;
    p->vta = 0;
    p->vtb = 0;
    p->vtc = 0;
}

void my_svpwm_calc(SVPWM_STRUCT *p, float alpha, float beta)
{
    p->alpha = alpha;
    p->beta = beta;

    if (p->beta > 0)
        p->na = 0;
    else
        p->na = 4;
    if (p->beta - 1.7321f * p->alpha > 0)
        p->nb = 0;
    else
        p->nb = 2;
    if (p->beta + 1.7321f * p->alpha > 0)
        p->nc = 0;
    else
        p->nc = 1;

    switch (p->na + p->nb + p->nc)
    {
    case 0:
        p->N = 2;
        break;
    case 1:
        p->N = 3;
        break;
    case 2:
        p->N = 1;
        break;
    case 3:
        p->N = 0;
        break;
    case 4:
        p->N = 0;
        break;
    case 5:
        p->N = 4;
        break;
    case 6:
        p->N = 6;
        break;
    case 7:
        p->N = 5;
        break;
    }

    float tem_alp = p->alpha * 0.866f;
    float tem_bet = p->beta * 0.5f;

    switch (p->N)
    {
    case 1:
        p->t1 = tem_alp - tem_bet;
        p->t2 = p->beta;
        p->t0 = 1 - p->t1 - p->t2;
        break;
    case 2:
        p->t1 = tem_alp + tem_bet;
        p->t2 = -tem_alp + tem_bet;
        p->t0 = 1 - p->t1 - p->t2;
        break;
    case 3:
        p->t1 = p->beta;
        p->t2 = -tem_alp - tem_bet;
        p->t0 = 1 - p->t1 - p->t2;
        break;
    case 4:
        p->t1 = -tem_alp + tem_bet;
        p->t2 = -p->beta;
        p->t0 = 1 - p->t1 - p->t2;
        break;
    case 5:
        p->t1 = -tem_alp - tem_bet;
        p->t2 = tem_alp - tem_bet;
        p->t0 = 1 - p->t1 - p->t2;
        break;
    case 6:
        p->t1 = -p->beta;
        p->t2 = tem_alp + tem_bet;
        p->t0 = 1 - p->t1 - p->t2;
        break;
    }

    switch (p->N)
    {
    case 1:
        p->vta = p->t0 * 0.5f + p->t1 + p->t2;
        p->vtb = p->t0 * 0.5f + p->t2;
        p->vtc = p->t0 * 0.5f;
        break;
    case 2:
        p->vta = p->t0 * 0.5f + p->t1;
        p->vtb = p->t0 * 0.5f + p->t1 + p->t2;
        p->vtc = p->t0 * 0.5f;
        break;
    case 3:
        p->vta = p->t0 * 0.5f;
        p->vtb = p->t0 * 0.5f + p->t1 + p->t2;
        p->vtc = p->t0 * 0.5f + p->t2;
        break;
    case 4:
        p->vta = p->t0 * 0.5f;
        p->vtb = p->t0 * 0.5f + p->t1;
        p->vtc = p->t0 * 0.5f + p->t1 + p->t2;
        break;
    case 5:
        p->vta = p->t0 * 0.5f + p->t2;
        p->vtb = p->t0 * 0.5f;
        p->vtc = p->t0 * 0.5f + p->t1 + p->t2;
        break;
    case 6:
        p->vta = p->t0 * 0.5f + p->t1 + p->t2;
        p->vtb = p->t0 * 0.5f;
        p->vtc = p->t0 * 0.5f + p->t1;
        break;
    }

    p->vta = 0.1f + (p->vta + 0.22f) * 0.8f / 1.4f;
    p->vtb = 0.1f + (p->vtb + 0.22f) * 0.8f / 1.4f;
    p->vtc = 0.1f + (p->vtc + 0.22f) * 0.8f / 1.4f;
    
    update_hrtim_duty(p->vta, p->vtb, p->vtc);
}
//更新占空比
void update_hrtim_duty(float dutyA, float dutyB, float dutyC)
{

  uint32_t period = 17000;

//    uint32_t cmpA = (uint32_t)(period * (1.0f - dutyA));
//    uint32_t cmpB = (uint32_t)(period * (1.0f - dutyB));
//    uint32_t cmpC = (uint32_t)(period * (1.0f - dutyC));


	hhrtim1.Instance->sTimerxRegs[0].CMP1xR =period * dutyA;
	hhrtim1.Instance->sTimerxRegs[1].CMP1xR =period * dutyB;
	hhrtim1.Instance->sTimerxRegs[2].CMP1xR =period * dutyC;


}
