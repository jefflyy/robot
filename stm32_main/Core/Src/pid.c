/* PID */

#include "pid.h"

void PID_init(struct PID *pid, float kp, float ki, float kd){
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    PID_clear(pid);
}

void PID_clear(struct PID *pid){
    pid->las_d = 0;
    pid->sum_d = 0;
    pid->qlen = 0;
}

void PID_pushd(struct PID *pid, short d){
    #ifdef queue_integral
    if(pid->qlen < PID_QLEN){
        pid->q[pid->qlen++] = d;
    }else{
        pid->sum_d -= pid->q[0];
        for(short i = 1; i < PID_QLEN; i++){
            pid->q[i - 1] = pid->q[i];
        }
        pid->q[PID_QLEN - 1] = d;
    }
    #endif
    pid->sum_d += d;
}

float PID_calc(struct PID *pid, short now, short target){
    short d = target - now;
    PID_pushd(pid, d);
    float res =
        pid->kp * d +
        pid->ki * pid->sum_d +
        pid->kd * (d - pid->las_d);
    pid->las_d = d;
    return res;
}
