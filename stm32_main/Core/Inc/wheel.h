/* WHEEL */

#ifndef __WHEEL_H
#define __WHEEL_H

#include "main.h"
#include "encoder.h"

struct Wheel{
    TIM_HandleTypeDef *pwm, *encoder;
    GPIO_TypeDef *g1, *g2;
    uint16_t channel, p1, p2;
    uint16_t lasenc;
};

void Wheel_init(
    struct Wheel*,
    TIM_HandleTypeDef*, uint16_t,
    GPIO_TypeDef*, uint16_t,
    GPIO_TypeDef*, uint16_t,
    TIM_HandleTypeDef*);
void Wheel_set(struct Wheel*, int);
uint16_t Wheel_getspeed(struct Wheel*);

#endif
