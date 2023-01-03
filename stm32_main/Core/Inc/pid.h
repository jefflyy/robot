/* PID */

#ifndef __PID_H
#define __PID_H

#define queue_integral

#define PID_QLEN 10

struct PID{
    float kp, ki, kd;
    float las_d, sum_d;
    short q[PID_QLEN], qlen;
};

void PID_init(struct PID*, float, float, float);
void PID_clear(struct PID*);
void PID_pushd(struct PID*, short);
float PID_calc(struct PID*, short, short);

#endif
