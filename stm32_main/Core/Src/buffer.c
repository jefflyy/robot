/* BUFFER */

#include "main.h"
#include "buffer.h"

#include <string.h>

uint8_t queue[3], top;
extern uint8_t opid;
extern short opnd;

uint8_t buffer_push(uint8_t val){
    if(top < 3){
        queue[top++] = val;
    }else{
        queue[0] = queue[1];
        queue[1] = queue[2];
        queue[2] = val;
    }
    uint8_t res = 0;
    if(top == 3 && queue[0]){
        opid = queue[0];
        memcpy(&opnd, queue + 1, 2);
        top = 0;
        res = 1;
    }
    return res;
}
