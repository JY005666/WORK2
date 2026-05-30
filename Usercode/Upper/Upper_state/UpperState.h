#ifndef UPPERSTATE_H
#define UPPERSTATE_H

#include"UpperStart.h"

typedef enum{
    YELLOW = 0,
    GREEN,
    WHITE
} BeanColor;

typedef enum{
    LEFT = 0,
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


extern Bean bean[3];
extern Box box[5];

void Bean_Init(void);
void Box_Init(void);
void Bean_Target_Set(void);
void init_paramater(paramater *par);

void Upper_State_Start(void);

extern paramater par;

#endif // !UPPERSTATE_H