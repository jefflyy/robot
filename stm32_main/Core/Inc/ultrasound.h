/* ULTRASOUND */

#ifndef __ULTRASOUND_H
#define __ULTRASOUND_H

#include "main.h"

struct Ultrasound;

typedef void (*retrfunc)(struct Ultrasound*);
void retr_front(struct Ultrasound*);

struct Ultrasound{
    TIM_HandleTypeDef *echo_htim;
    GPIO_TypeDef *trig_group;
    uint16_t echo_channel, trig_pin;

    uint8_t captured, up;
    uint16_t overflow, val;

    retrfunc retreat;
    uint8_t danger;
};

void Ultrasound_init(
    struct Ultrasound*,
    TIM_HandleTypeDef*, uint16_t,
    GPIO_TypeDef*, uint16_t);
void Ultrasound_trig(struct Ultrasound*);
void Ultrasound_detrig(struct Ultrasound*);
void Ultrasound_periodcallback(struct Ultrasound*);
void Ultrasound_capturecallback(struct Ultrasound*);
uint8_t Ultrasound_getval(struct Ultrasound*, uint32_t*);

#endif
