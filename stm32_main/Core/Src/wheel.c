/* WHEEL */

#include "main.h"
#include "wheel.h"
#include "utils.h"

void Wheel_init(
    struct Wheel *wheel,
    TIM_HandleTypeDef *pwm, uint16_t channel,
    GPIO_TypeDef *g1, uint16_t p1,
    GPIO_TypeDef *g2, uint16_t p2,
    TIM_HandleTypeDef *encoder){
    /*
        TIM(PWM):
            prescaler = 0
            period = 7199
        TIM(ENCODER):
            Tl1
    */
    wheel->pwm = pwm;
    wheel->channel = channel;
    wheel->g1 = g1;
    wheel->p1 = p1;
    wheel->g2 = g2;
    wheel->p2 = p2;
    wheel->encoder = encoder;
    HAL_GPIO_WritePin(g1, p1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(g2, p2, GPIO_PIN_RESET);
    HAL_TIM_PWM_Start(pwm, channel);
    Encoder_init(encoder);
    wheel->lasenc = 0;
}

extern uint16_t speedlim;

void Wheel_set(struct Wheel *wheel, int wide){
    /*
        wide: 0-7200
        way: >0=forward, <0=backward
    */
    HAL_GPIO_WritePin(wheel->g1, wheel->p1, wide > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(wheel->g2, wheel->p2, wide > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(wheel->pwm, wheel->channel, min(abs(wide), speedlim));
}

uint16_t Wheel_getspeed(struct Wheel *wheel){   // call one time per TimerIT
    uint16_t now = Encoder_read(wheel->encoder);
    uint16_t res = now - wheel->lasenc;
    wheel->lasenc = now;
    return res;
}
