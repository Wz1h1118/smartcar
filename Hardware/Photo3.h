#ifndef __PHOTO3_H
#define __PHOTO3_H

/*左右两路光电传感器(只输出0/1)*/
#define PHOTO3_LEFT   0		//左PA12
#define PHOTO3_RIGHT  1		//右PA11

void Photo3_Init(void);
uint8_t Photo3_GetState(uint8_t channel);

#define Photo3_GetLeft()  Photo3_GetState(PHOTO3_LEFT)		//读左:返回0或1
#define Photo3_GetRight() Photo3_GetState(PHOTO3_RIGHT)		//读右:返回0或1

#endif
