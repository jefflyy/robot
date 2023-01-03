/* CONTROL */

#include "main.h"
#include "pid.h"
#include "wheel.h"
#include "control.h"
#include "JY901.h"
#include "servo.h"

#include <stdio.h>
#include <string.h>

extern struct Wheel wheelL, wheelR;
extern TIM_HandleTypeDef htim1, htim4, htim8;
extern uint8_t doPID;
extern struct PID fwdPIDL, fwdPIDR;

void control_init(){
    Wheel_init(&wheelL, &htim4, TIM_CHANNEL_1, GPIOD, GPIO_PIN_0, GPIOD, GPIO_PIN_1, &htim8);	// left
    Wheel_init(&wheelR, &htim4, TIM_CHANNEL_2, GPIOB, GPIO_PIN_9, GPIOB, GPIO_PIN_8, &htim1);	// right
}

extern int fwdlim;

extern uint16_t aimENC;
extern struct SAngle angle;
extern struct PID rotPID;
extern short aimANG;
extern uint8_t doangPID;

extern uint8_t dospinPID;
extern struct PID speedPIDL, speedPIDR;
extern short speedL, speedR;
extern short aimspeedL, aimspeedR;

extern struct PID spinPID;

extern UART_HandleTypeDef huart1;

extern uint8_t opid;
extern short opnd;
uint8_t lasforward;

extern uint8_t tank;
short baseANG;

extern uint8_t force;

extern uint8_t startmove;

#define BLUESEND HAL_UART_Transmit(&huart1, (uint8_t*)txbuff, BLUETXLEN, 100);

void handleBlueRX(char *rxbuff, char *txbuff){
    startmove = 1;

    float ftmp;
    short stmp;
    switch(rxbuff[0]){
        case 'F':
            dospinPID = 0;
            sscanf(rxbuff + 1, "%hd", &stmp);
            if(stmp){
                fwdlim = stmp;
            }
            PID_clear(&rotPID);
            doangPID = 1;
            tank = 1;
            lasforward = 1;
            doPID = 1;

            sprintf(txbuff, "FORWARD\n");
            BLUESEND
            break;
        case 'g':
            force = 1;
            dospinPID = 0;
            doPID = 0;
            sscanf(rxbuff + 1, "%hu", &aimENC);
            PID_clear(&fwdPIDL);
            PID_clear(&fwdPIDR);
            PID_clear(&rotPID);
            Encoder_start(wheelL.encoder);
            Encoder_start(wheelR.encoder);
            lasforward = 0;
            tank = 0;
            doPID = 1;
            break;
        case 'l':
            sprintf(txbuff, "L%u R%u\n", Encoder_read(wheelL.encoder), Encoder_read(wheelR.encoder));
            BLUESEND
            break;
        case 'c':
            sscanf(rxbuff + 1, "%f", &ftmp);
            PID_init(&rotPID, ftmp, 0, 0);
            break;
        case 'p':
            doPID = 0;
            sscanf(rxbuff + 1, "%f", &ftmp);
            spinPID.kp = ftmp;
            PID_clear(&speedPIDL);
            PID_clear(&speedPIDR);
            PID_clear(&spinPID);
            dospinPID = 1;
            break;
        case 'a':
            force = 0;
            aimANG = angle.Angle[2];
            doangPID = 1;
            break;
        case 'b': // baseangle
            baseANG = angle.Angle[2];

            sprintf(txbuff, "baseangle%d\n", baseANG);
            BLUESEND
            break;
        case 'B':
            doPID = 0;
            sscanf(rxbuff + 1, "%hd", &stmp);
            aimANG = baseANG + stmp;
            PID_clear(&speedPIDL);
            PID_clear(&speedPIDR);
            PID_clear(&spinPID);
            dospinPID = 1;
            break;
        case 't':
            doPID = 0;
            sscanf(rxbuff + 1, "%hd", &stmp);
            sprintf(txbuff,  "aim%d now%d\n", aimANG, angle.Angle[2]);
            BLUESEND
            aimANG = angle.Angle[2] + stmp;
            if(lasforward){
                doPID = 1;
            }else{
                PID_clear(&speedPIDL);
                PID_clear(&speedPIDR);
                PID_clear(&spinPID);
                dospinPID = 1;
            }
            break;
        case 'r':
            sprintf(txbuff,  "aim%d now%d\n", aimANG, angle.Angle[2]);
            BLUESEND
            break;
        case 'P':
            Paw_pick();
            break;
        case 'D':
        	sscanf(rxbuff + 1, "%hd", &stmp);
            Paw_drop(stmp);
            break;
    }
}

void handleOpenRX(char* txbuff){
    startmove = 1;
    
    switch(opid){
        case 1: // forward
            dospinPID = 0;
            if(opnd){
                fwdlim = opnd;
            }
            PID_clear(&rotPID);
            doangPID = 1;
            tank = 1;
            lasforward = 1;
            doPID = 1;

            sprintf(txbuff, "FORWARD\n");
            BLUESEND
            break;
        case 2: // step
            if(opnd){
                doPID = 0;
                PID_clear(&fwdPIDL);
                PID_clear(&fwdPIDR);
                PID_clear(&rotPID);
                Encoder_start(wheelL.encoder);
                Encoder_start(wheelR.encoder);
                aimENC = opnd;
                tank = 0;
                doPID = 1;
            }else{
                doPID = 0;
                aimANG = angle.Angle[2];
                PID_clear(&speedPIDL);
                PID_clear(&speedPIDR);
                PID_clear(&spinPID);
                dospinPID = 1;
            }
            lasforward = 0;

            sprintf(txbuff, "step%d\n", opnd);
            BLUESEND
            break;
        case 3: // baseangle
            baseANG = angle.Angle[2];

            sprintf(txbuff, "baseangle%d\n", baseANG);
            BLUESEND
            break;
        case 4: // turn
        case 5: // baseangle turn
            doPID = 0;
            aimANG = (opid == 4 ? angle.Angle[2] : baseANG) + opnd;
            if(lasforward){
                doPID = 1;
            }else{
                PID_clear(&speedPIDL);
                PID_clear(&speedPIDR);
                PID_clear(&spinPID);
                dospinPID = 1;
            }

            sprintf(txbuff, "%s%d\n", (opid == 4 ? "turn" : "baseturn"), opnd);
            BLUESEND
            break;
        case 6: // pick
            Paw_pick();
            break;
        case 7:
            Paw_drop(opnd);
            sprintf(txbuff, "drop%d\n", opnd);
            BLUESEND
            break;
    }
}
