#ifndef UPPERSTATE_H
#define UPPERSTATE_H

#include"UpperStart.h"

typedef enum{
    YELLOW = 0,
    GREEN,
    WHITE
} BeanColor;

typedef enum{
    LEFT = 1,
    RIGHT,
    MIDDLE
} BeanPosition;

typedef enum{
    LEFT_2 = 0,
    LEFT_1,
    MIDDLE_0,
    RIGHT_1,
    RIGHT_2
} BoxPosition;

typedef struct{
    BeanColor color;
    BeanPosition position;
    uint8_t target_number;
    BoxPosition target_position;
} Bean;


typedef struct{
    BoxPosition position;
    uint8_t number;
} Box;
typedef struct{
    float target_distance;
    float degree_chassis;
    float degree_claw; 
} paramater;

//新建结构体，用来储存每个豆子和箱子对应的角度
typedef struct{
    float distance;
    float chassis;
    uint16_t claw_angle;
} Angle;

extern Bean bean[3];
extern Box box[5];
extern uint16_t stage_flag;
extern Angle bean_left;
extern Angle bean_right;
extern Angle bean_middle;
extern Angle box_left_2;
extern Angle box_left_1;
extern Angle box_middle_0;
extern Angle box_right_1;
extern Angle box_right_2;
extern float box_middle_0_chassis_cw;
extern float box_middle_0_chassis_ccw;

void Angle_Init(void);
void Bean_Init(void);
void Bean_Target_Set(void);
void init_paramater(paramater *par);

void Upper_State_Start(void);

extern paramater par;

#endif // !UPPERSTATE_H
