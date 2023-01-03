/* ULTRASOUND */

#include "main.h"
#include "ultrasound.h"
#include "control.h"

extern uint32_t Ultralim;

void Ultrasound_init(
    struct Ultrasound *ultra,
    TIM_HandleTypeDef *echo_htim, uint16_t echo_channel,
    GPIO_TypeDef *trig_group, uint16_t trig_pin){
    /*
        TIM:
            prescaler = 71
            period = 65535
    */
    ultra->echo_htim = echo_htim;
    ultra->echo_channel = echo_channel;
    ultra->trig_group = trig_group;
    ultra->trig_pin = trig_pin;
    HAL_TIM_IC_Start_IT(echo_htim, echo_channel);
    __HAL_TIM_ENABLE_IT(echo_htim, TIM_IT_UPDATE);
    HAL_GPIO_WritePin(trig_group, trig_pin, GPIO_PIN_RESET);
    ultra->captured = 0;
    ultra->up = 0;
    ultra->overflow = 0;
    ultra->val = 0;
}

void Ultrasound_trig(struct Ultrasound *ultra){
    HAL_GPIO_WritePin(ultra->trig_group, ultra->trig_pin, GPIO_PIN_SET);
}

void Ultrasound_detrig(struct Ultrasound *ultra){
    HAL_GPIO_WritePin(ultra->trig_group, ultra->trig_pin, GPIO_PIN_RESET);
}

void Ultrasound_periodcallback(struct Ultrasound *ultra){
    if(ultra->captured == 0 && ultra->up){
        if(ultra->overflow == 0x3F){
            ultra->captured = 1;
            ultra->val = 0xFFFF;
        }else{
            ultra->overflow++;
        }
    }
}

void Ultrasound_capturecallback(struct Ultrasound *ultra){
    if(ultra->captured == 0){
        if(ultra->up){
            ultra->captured = 1;
            ultra->val = HAL_TIM_ReadCapturedValue(ultra->echo_htim, ultra->echo_channel);
            TIM_RESET_CAPTUREPOLARITY(ultra->echo_htim, ultra->echo_channel);
            TIM_SET_CAPTUREPOLARITY(ultra->echo_htim, ultra->echo_channel, TIM_ICPOLARITY_RISING);
            uint32_t res;
            if(Ultrasound_getval(ultra, &res) && res < Ultralim){
                ultra->retreat(ultra);
            }else{
                ultra->danger = 0;
            }
        }else{
            ultra->up = 1;
            ultra->val = 0;
            __HAL_TIM_DISABLE(ultra->echo_htim);
            __HAL_TIM_SET_COUNTER(ultra->echo_htim, 0);
            TIM_RESET_CAPTUREPOLARITY(ultra->echo_htim, ultra->echo_channel);
            TIM_SET_CAPTUREPOLARITY(ultra->echo_htim, ultra->echo_channel, TIM_ICPOLARITY_FALLING);
            __HAL_TIM_ENABLE(ultra->echo_htim);
        }
    }
}

uint8_t Ultrasound_getval(struct Ultrasound *ultra, uint32_t *val){
    if(ultra->captured){
        (*val) = ultra->overflow * 0x10000UL + ultra->val;
        ultra->captured = 0;
        ultra->up = 0;
        ultra->overflow = 0;
        return 1;
    }else{
        (*val) = 0;
        return 0;
    }
}

extern uint8_t doPID, dospinPID, speedonly;
extern struct PID speedPIDL, speedPIDR;
extern short aimspeedL, aimspeedR;

extern uint32_t force;
extern uint8_t doultra;

extern UART_HandleTypeDef huart1;

void retr_front(struct Ultrasound *ultra){
    if(doultra){
        HAL_UART_Transmit(&huart1, (uint8_t*)"retr\n", 5, 100);

        force = ultra->danger = 1;
        doPID = 0;
        PID_clear(&speedPIDL);
        PID_clear(&speedPIDR);
        speedonly = 1;
        aimspeedL = aimspeedR = -10;
        dospinPID = 1;
    }
}
