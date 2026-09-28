#include "stm32f10x.h"                  // Device header
#include "Timer.h"
#include "OLED.h"
#include "Encoder.h"
#include "Track.h"
#include "SmartCar.h"
#include "Serial.h"
#include "Parking.h"
#include "LineFollow.h"

int main(void)
{
	OLED_Init();
	SmartCar_Init();
	Track_Init();
	Parking_Init();
	Encoder_Init();
	Timer_Init();
	Serial_Init();

	while(1)
	{
		Parking_Show();			/*第1行SidePark,第2行停车阶段,第3行光电电平*/
		LineFollow_Show();		/*第3、4行巡线调试信息+串口输出*/
	}
}

/*TIM1更新中断,每1ms触发一次;20ms一拍执行巡线与停车逻辑*/
void TIM1_UP_IRQHandler(void)
{
	static int count;
	if(TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		Parking_Scan();			/*每1ms:高速采样侧面光电,触发侧方停车*/
		count++;
		if(count >= 20)
		{
			count = 0;
			Parking_Tick();		/*每20ms:侧方停车状态机*/
			LineFollow_Tick();	/*每20ms:巡线+直角弯(停车执行中自动让位)*/
		}
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}
