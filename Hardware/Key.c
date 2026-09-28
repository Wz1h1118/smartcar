#include "stm32f10x.h"                  // Device header
#include "Delay.h"

uint8_t Key_Num;

void Key_Init(void)
{
	/*释放PC13的RTC侵入检测(tamper)功能,当作普通GPIO按键使用*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	PWR_BackupAccessCmd(ENABLE);
	BKP_TamperPinCmd(DISABLE);
	BKP_ITConfig(DISABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;	//PC13按键1 PC15按键2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

uint8_t Key_GetNum(void)
{
	uint8_t Temp;
	if (Key_Num)
	{
		Temp = Key_Num;
		Key_Num = 0;
		return Temp;
	}
	return 0;
}

uint8_t Key_GetState(void)
{
	if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == 0)
	{
		return 1;
	}
	if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15) == 0)
	{
		return 2;
	}
	return 0;
}

void Key_Tick(void)
{
	static uint8_t Count;
	static uint8_t CurrState, PrevState;
	
	Count ++;
	if (Count >= 20)
	{
		Count = 0;
		
		PrevState = CurrState;
		CurrState = Key_GetState();
		
		if (CurrState == 0 && PrevState != 0)
		{
			Key_Num = PrevState;
		}
	}
}
