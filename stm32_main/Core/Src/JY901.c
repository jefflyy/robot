/* JY901 */

#include "main.h"
#include "JY901.h"

#include <string.h>

void JY901_init(struct JY901 *data, UART_HandleTypeDef *huart){
    /*
        UART:
            DMA
            Baudrate = 115200
    */
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    HAL_UART_Receive_DMA(huart, data->buffer, JY901_BUFFERLEN);
}

void JY901_getdata(struct JY901 *data, struct SAngle *angle){
    if(data->RXlen != JY901_BUFFERLEN)
        return;
    if(!(data->buffer[0] == 0x55 && data->buffer[1] == 0x53))
        return;
    memcpy(angle, data->buffer + 2, 8);
}
