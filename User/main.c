#include "stm32f10x.h"                  // Device header
#include "Timer.h"
#include "OLED.h"
#include "Encoder.h"
#include "Track.h"
#include "SmartCar.h"
#include "Serial.h"
#include "Parking.h"
#include "LineFollow.h"
#include "Ultrasonic.h"

int main(void)
{
	OLED_Init();
	SmartCar_Init();
	Track_Init();
	Parking_Init();
	Encoder_Init();
	Timer_Init();
	Serial_Init();
	Ultrasonic_Init();

	while(1)
	{
		Parking_Show();			/*第1行SidePark,第2行停车阶段,第3行光电电平*/
		LineFollow_Show();		/*第3、4行巡线调试信息+串口输出*/
		OLED_ShowNum(4,13,Ultrasonic_GetDistance(),3);	/*超声波距离(cm)*/
	}
}

/*TIM1更新中断,每1ms触发一次;20ms一拍执行巡线与停车逻辑*/
void TIM1_UP_IRQHandler(void)
{
	static int count;
	static uint8_t us_tick;
	if(TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		Parking_Scan();			/*每1ms:高速采样侧面光电,触发侧方停车*/
		count++;
		if(count >= 20)
		{
			count = 0;
			Parking_Tick();		/*每20ms:侧方停车状态机*/
			LineFollow_Tick();	/*每20ms:巡线+直角弯(停车执行中自动让位)*/
			if(++us_tick >= 5)	/*每100ms:超声波测距一次*/
			{
				us_tick = 0;
				Ultrasonic_Trigger();
			}
		}
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}
