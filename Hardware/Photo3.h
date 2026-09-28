#ifndef __PHOTO3_H
#define __PHOTO3_H

/*三路独立光电传感器通道号(只输出0/1)*/
#define PHOTO3_LEFT   0		//PA11 左侧
#define PHOTO3_RIGHT  1		//PA12 右侧
#define PHOTO3_BACK   2		//PB10 后侧

void Photo3_Init(void);
uint8_t Photo3_GetState(uint8_t channel);

#define Photo3_GetLeft()  Photo3_GetState(PHOTO3_LEFT)		//读左:返回0或1
#define Photo3_GetRight() Photo3_GetState(PHOTO3_RIGHT)		//读右:返回0或1
#define Photo3_GetBack()  Photo3_GetState(PHOTO3_BACK)		//读后:返回0或1

#endif
