/* SERVO */

#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"

struct Servo{
	TIM_HandleTypeDef *htim;
	uint16_t channel;
};

void Servo_init(struct Servo*, TIM_HandleTypeDef*, uint16_t);
void Servo_set(struct Servo*, uint16_t);
void Paw_init();
void Paw_pick();
void Paw_drop(uint16_t);

#define PWM_DOWN 0x0580
#define PWM_UP 0x0700
#define PWM_OPEN 0x0570
#define PWM_CLOSE 0x0180

#endif
