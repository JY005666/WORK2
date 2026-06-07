#ifndef __VISION_SYNC_H
#define __VISION_SYNC_H

#include "main.h"
#include <stdint.h>

void Vision_Init(void);
void Vision_Start(void);
void Vision_Process(void);

uint8_t Vision_IsDone(void);
uint8_t Vision_HasError(void);
uint8_t Vision_GetPos(uint8_t out[3]);

void Vision_Clear(void);

#endif



