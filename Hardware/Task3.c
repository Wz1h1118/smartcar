#include "stm32f10x.h"                  // Device header
#include "Task3.h"
#include "LineFollow.h"
#include "SmartCar.h"
#include "Serial.h"

extern volatile uint8_t Track_Count;

/*可调参数(1拍=20ms)*/
#define T3_FWD_SPEED   100	//出库直行PWM
#define T3_TURN_SPEED  150	//出库转弯PWM
#define T3_TURN_TICKS  12	//转90°拍数
#define T3_DIR_SWAP    0	//方向反了改1
#define T3_FWD_TIMEOUT 100	//直行找线超时拍数,超时停车

static volatile uint8_t T3Hold = 1;		//1=保持停车(未识别到)
static volatile uint8_t T3Phase = 0;	//0=直行出库 1=原地转 2=交给巡线
static volatile uint16_t T3Tick = 0;
static volatile uint8_t T3Dir = 0;		//转弯方向:0=左 1=右

void Task3_Reset(void)
{
	T3Hold = 1;
	T3Phase = 0;
	T3Tick = 0;
}

/*PC15发车时调用:按识别字母设置停车弯数和出库转弯方向*/
void Task3_Launch(void)
{
	uint8_t r = VisionResult;
	if(r == 1 || r == 3 || r == 5 || r == 7)
	{
		LineFollow_SetTargetTurns((r == 1 || r == 7) ? 1 : 0);	//A/D=第2个弯停 B/C=第1个弯停
		T3Dir = ((r <= 3) ? 0 : 1) ^ T3_DIR_SWAP;				//A/B=左转 C/D=右转
		T3Phase = 0;
		T3Tick = 0;
		T3Hold = 0;
	}
	else
	{
		T3Hold = 1;	//没识别到:保持停车
	}
}

/*每20ms:出库直行→压线原地转→转完交给巡线*/
void Task3_Tick(void)
{
	if(T3Phase == 0)		//直行出库,等前方8路压线
	{
		Move_SetSpeed(T3_FWD_SPEED, T3_FWD_SPEED);
		if(Track_Count > 0) { T3Tick = 0; T3Phase = 1; }
		else if(++T3Tick > T3_FWD_TIMEOUT) { T3Hold = 1; }	//超时保持停
	}
	else if(T3Phase == 1)	//压线:原地转90°(A/B左 C/D右)
	{
		Move_SetSpeed(T3Dir == 0 ? -T3_TURN_SPEED : T3_TURN_SPEED,
		              T3Dir == 0 ?  T3_TURN_SPEED : -T3_TURN_SPEED);
		if(++T3Tick >= T3_TURN_TICKS) { T3Phase = 2; }		//转完交给巡线
	}
}

uint8_t Task3_Running(void)
{
	return T3Hold || (T3Phase < 2);	//未识别/直行/转弯时接管,转完交还巡线
}
