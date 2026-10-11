#include "stm32f10x.h"                  // Device header
#include "TB6612.h"

void SmartCar_Init(void)
{
	TB6612_Init();
}

void Move_Stop(void)
{
	Motor_SetLeftPWM(0);
	Motor_SetRightPWM(0);
}

void Move_SetSpeed(int16_t left, int16_t right)
{
	if(left > 999)  left  = 999;		//限幅到-999~999
	if(left < -999) left  = -999;
	if(right > 999)  right = 999;
	if(right < -999) right = -999;

	Motor_SetLeftPWM(left);
	Motor_SetRightPWM(right);
}
