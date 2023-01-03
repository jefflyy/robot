/* ENCODER */

#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

void Encoder_init(TIM_HandleTypeDef*);
void Encoder_start(TIM_HandleTypeDef*);
uint16_t Encoder_read(TIM_HandleTypeDef*);

#endif
