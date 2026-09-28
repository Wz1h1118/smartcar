#include "stm32f10x.h"                  // Device header
#include "Photo3.h"

void Photo3_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;			//光电模块输出0/1数字量,上拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;	//PA11左 PA12右
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;				//PB10后
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}

uint8_t Photo3_GetState(uint8_t channel)
{
	switch(channel)
	{
		case PHOTO3_LEFT : return GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11);
		case PHOTO3_RIGHT: return GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12);
		case PHOTO3_BACK : return GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10);
		default          : return 0;
	}
}
