
#ifndef __KEYPAD_H__
#define __KEYPAD_H__
#include "tm4c123gh6pm.h"
#include "Delay.h"
#include "types.h"

#define  padRows  4
#define  padCols  4

void keypad_Init       (void);
uint8 keypad_getkey   (void);

#endif