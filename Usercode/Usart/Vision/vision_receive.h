#ifndef VISION_RECEIVE_H
#define VISION_RECEIVE_H

#define MAX_PAYLOAD_LEN  10  

#include"usart.h"
#include"UpperState.h"

//状态机结构体
typedef enum {
    STATE_WAIT_HEADER,
    STATE_WAIT_CAM_ID,
    STATE_WAIT_LENGTH,
    STATE_WAIT_PAYLOAD,
    STATE_WAIT_TAIL
} RxState_t;


//接收数据储存结构体
typedef struct {
    RxState_t state;
    uint8_t cam_id;
    uint8_t target_len;
    uint8_t current_len;
    uint8_t payload[MAX_PAYLOAD_LEN];
    uint8_t frame_ready;
} RxFrame_t;

extern RxFrame_t rx_frame;
uint8_t rx_byte;

void vision_receive_init(void);


#endif /* VISION_RECEIVE_H */



