#include "stm32f10x.h"                  // Device header
#include "Gray8.h"

const int8_t Weight[8] = {-7,-5,-3,-1,1,3,5,7};
volatile int8_t Location[8];
float Weight_Sum;
extern uint8_t Track_Count;

void Track_Init(void)
{
	Gray8_Init();
}

float Track_GetState(void)
{
	Track_Count = 0;
	Weight_Sum = 0;		//每次调用先清零,防止累加失真
	for(int i = 0;i < 8;i++)
			{
				Location[i] = Gray8_GetState(i);
			}
	for(int i = 0;i < 8;i++)
			{
				if(Location[i] == 1)
				{
					Weight_Sum += Weight[i];
					Track_Count++;
				}
			}
	if(Track_Count == 0)
	{
		return 0;
	}
	else{
	return Weight_Sum /Track_Count;	
	}		
}
