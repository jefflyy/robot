/* TIMER */

#include "main.h"
#include "timer.h"
#include "control.h"
#include "JY901.h"
#include "pid.h"

void TimerIT_init(TIM_HandleTypeDef *htim){
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
    __HAL_TIM_SET_COUNTER(htim, 0);
    HAL_TIM_Base_Start_IT(htim);
}

extern uint8_t doPID;
extern uint16_t aimENC;
extern struct PID fwdPIDL, fwdPIDR;
extern struct Wheel wheelL, wheelR;

extern struct PID rotPID;
extern short aimANG;
extern struct SAngle angle;
extern uint8_t doangPID;

extern uint8_t dospinPID;
extern struct PID speedPIDL, speedPIDR;
extern short speedL, speedR;
extern short aimspeedL, aimspeedR;

extern uint8_t speedonly;

extern struct PID spinPID;

extern uint8_t tank;    // dont stop

extern int fwdlim;
int finallim = 4500;

extern uint8_t startmove;

void TimerIT_callback(){
    if(!startmove){
        return;
    }

    speedL = Wheel_getspeed(&wheelL);
    speedR = Wheel_getspeed(&wheelR);
    if(doPID){
        int dt = doangPID ? PID_calc(&rotPID, angle.Angle[2], aimANG) : 0;
        int base = fwdlim;
        if(!tank){
            aimspeedL = PID_calc(&fwdPIDL, Encoder_read(wheelL.encoder), aimENC);
            aimspeedR = PID_calc(&fwdPIDR, Encoder_read(wheelR.encoder), aimENC);
            int tl = PID_calc(&speedPIDL, speedL, aimspeedL);
            int tr = PID_calc(&speedPIDR, speedR, aimspeedR);
            base = clip((tl + tr) / 2, -finallim, finallim);
        }
        Wheel_set(&wheelL, base + dt);
    	Wheel_set(&wheelR, base - dt);
    }else if(dospinPID){
        if(speedonly){
            Wheel_set(&wheelL, PID_calc(&speedPIDL, speedL, aimspeedL));
            Wheel_set(&wheelR, PID_calc(&speedPIDR, speedR, aimspeedR));
        }else{
            int base = PID_calc(&spinPID, angle.Angle[2], aimANG);
            Wheel_set(&wheelL, PID_calc(&speedPIDL, speedL, base));
            Wheel_set(&wheelR, PID_calc(&speedPIDR, speedR, -base));
        }
    }else{
        Wheel_set(&wheelL, 0);
        Wheel_set(&wheelR, 0);
    }
}
