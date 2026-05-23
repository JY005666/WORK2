#include "vision_receive.h"
#include"stdint.h"


RxFrame_t rx_frame;


void vision_receive_init(void){
    rx_frame.state = STATE_WAIT_HEADER;
    rx_frame.frame_ready = 0;
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

    uint8_t trigger_data = 0x11;   
        // 发送这个单变量，长度严格指定为 1 个字节
    HAL_UART_Transmit(&huart1, &trigger_data, 1, HAL_MAX_DELAY);
}

void vision_receive_process(RxFrame_t *rx_frame){
    switch (rx_frame->state){
        case STATE_WAIT_HEADER:
            if (rx_byte == 0xAA) {
                rx_frame->state = STATE_WAIT_CAM_ID;
            }
            break;
            
        case STATE_WAIT_CAM_ID:
            if (rx_byte >= 0x01 && rx_byte <= 0x03) {
                rx_frame->cam_id = rx_byte;
                    rx_frame->state = STATE_WAIT_LENGTH;
                } else {
                    rx_frame->state = STATE_WAIT_HEADER;
                } break;
                
        case STATE_WAIT_LENGTH:
            if (rx_byte <= MAX_PAYLOAD_LEN) {
                rx_frame->target_len = rx_byte;
                rx_frame->current_len = 0;
                if (rx_byte == 0) {
                    rx_frame->state = STATE_WAIT_TAIL;
                } else {
                    rx_frame->state = STATE_WAIT_PAYLOAD;
                }
            } else {
                rx_frame->state = STATE_WAIT_HEADER;
            } break;
            
        case STATE_WAIT_PAYLOAD:
            rx_frame->payload[rx_frame->current_len++] = rx_byte;
            if (rx_frame->current_len >= rx_frame->target_len) {
                rx_frame->state = STATE_WAIT_TAIL;
            } break;
        case STATE_WAIT_TAIL:
            if (rx_byte == 0x55) {
                rx_frame->frame_ready = 1;
            }
            rx_frame->state = STATE_WAIT_HEADER;
            break;
    }

}

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//     if(huart->Instance == USART1){
//         vision_receive_process(&rx_frame);
//         HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
//     }
// }