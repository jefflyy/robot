/* ENCODER */

#include "main.h"
#include "encoder.h"

void Encoder_init(TIM_HandleTypeDef *htim){
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
}

void Encoder_start(TIM_HandleTypeDef *htim){
    __HAL_TIM_SET_COUNTER(htim, 0);
}

uint16_t Encoder_read(TIM_HandleTypeDef *htim){
    return __HAL_TIM_GET_COUNTER(htim);
}
