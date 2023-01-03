/* SERVO */

#include "main.h"
#include "servo.h"
#include "control.h"
#include "utils.h"

void Servo_init(
	struct Servo *s,
	TIM_HandleTypeDef *htim, uint16_t channel){
	/*
		TIM:
			prescaler = 71
			period = 19999
	*/
	s->htim = htim;
	s->channel = channel;
	HAL_TIM_PWM_Start(htim, channel);
}

void Servo_set(struct Servo *s, uint16_t level){
	// level: 500-2500
	__HAL_TIM_SetCompare(s->htim, s->channel, level);
}

extern struct Servo servoB, servoS;

void Paw_init(){
	Servo_set(&servoB, PWM_DOWN);
	Servo_set(&servoS, PWM_OPEN);
}

#define PAW_DELAY 250

extern uint8_t doultra;

extern uint8_t force;

extern struct Wheel wheelL, wheelR;

extern uint8_t doPID;
extern struct PID fwdPIDL, fwdPIDR, rotPID;

extern uint8_t dospinPID, tank;
extern struct PID speedPIDL, speedPIDR, spinPID;
extern uint16_t aimENC;
extern uint8_t speedonly;
extern short aimspeedL, aimspeedR;

void Paw_pick(){
	force = 1;
    doPID = 0;
    PID_clear(&speedPIDL);
    PID_clear(&speedPIDR);
	speedonly = 1;
	aimspeedL = aimspeedR = -4;
	Servo_set(&servoS, PWM_CLOSE);
	dospinPID = 1;
	HAL_Delay(1000);
	dospinPID = 0;
	speedonly = 0;
	force = 0;
}

void Paw_drop(uint16_t step){
	doultra = 0;
	force = 1;

	dospinPID = 0;
	doPID = 0;
	Servo_set(&servoB, PWM_UP);
	aimENC = step;
    PID_clear(&fwdPIDL);
    PID_clear(&fwdPIDR);
    PID_clear(&rotPID);
    Encoder_start(wheelL.encoder);
    Encoder_start(wheelR.encoder);
    tank = 0;
    doPID = 1;

	HAL_Delay(max(2 * step, 1000));

	dospinPID = 0;
	doPID = 0;
	Servo_set(&servoS, PWM_OPEN);
    PID_clear(&speedPIDL);
    PID_clear(&speedPIDR);
	speedonly = 1;
	aimspeedL = aimspeedR = -10;
	dospinPID = 1;

	HAL_Delay(1000);

	Servo_set(&servoB, PWM_DOWN);
	dospinPID = 0;
	speedonly = 0;
	force = 0;
	doultra = 1;
}
