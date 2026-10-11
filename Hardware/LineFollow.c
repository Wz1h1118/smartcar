#include "stm32f10x.h"                  // Device header
#include "LineFollow.h"
#include "Track.h"
#include "Encoder.h"
#include "PID.h"
#include "SmartCar.h"
#include "OLED.h"
#include "Serial.h"
#include "Parking.h"

#define BASE_SPEED    60.0		//巡线基础速度(速度环目标:编码器脉冲增量/20ms)
#define TURN_SPEED    300		//直角弯原地转向PWM
#define LOST_TIMEOUT  75		//丢线超时(75×20ms≈1.5s),超时停车
#define LOSS_DEBOUNCE 6			//丢线去抖:连续6×20ms=120ms才判为真弯道

static volatile uint8_t TargetTurns = 3;	//停车弯数:第(TargetTurns+1)个弯停
static volatile uint8_t AutoStopEn = 1;		//1=按弯数自动停 0=不停(发挥一用)

/*定速PID参数*/
static PID_t Left_Speed_PID  = {.Kp=3.50f,.Ki=0.75f,.Kd=0.20f,.OutMax=1000.0f,.OutMin=-1000.0f,.ErrorIntMax=1000.0f};
static PID_t Right_Speed_PID = {.Kp=3.50f,.Ki=0.75f,.Kd=0.20f,.OutMax=1000.0f,.OutMin=-1000.0f,.ErrorIntMax=1000.0f};

/*循迹PID参数*/
static PID_t Line = {.Tar=0,.Kp=4,.Ki=0,.Kd=18,.OutMax=50,.OutMin=-50,.ErrorIntMax=75};

static float Left_Speed,Right_Speed,DifSpeed;
static int16_t LeftPulse,RightPulse;

volatile uint8_t Track_Count,Stop_Count;	//Track_Count供Parking.c判断前方压线
volatile uint8_t CornerEvent = 0;			//确认一个直角弯时置1,任务模块消费后清零
static volatile uint16_t LostCount = 0;		//连续丢线拍数(看到线即清零)
static volatile uint8_t Turn_Done = 0;		//本次丢线是否已计过弯
static volatile uint8_t Finished = 0;		//已走完目标弯数,停车

void LineFollow_Tick(void)
{
	/*先读传感器:停车流程判断压线也要用Track_Count,所以让位之前必须刷新*/
	LeftPulse = Encoder_GetLeft();
	RightPulse = Encoder_GetRight();
	Line.Act = Track_GetState();

	if(!Parking_Go()) { Move_Stop(); return; }		//未发车
	if(Parking_Running()) return;					//停车流程接管电机,巡线让位

	if(Finished)									//目标弯数已走完
	{
		Move_Stop();
	}
	else if(Track_Count > 0)						//正常巡线
	{
		LostCount = 0;
		Turn_Done = 0;

		PID_Update(&Line);
		DifSpeed = Line.Out;
		Left_Speed  = BASE_SPEED - DifSpeed;
		Right_Speed = BASE_SPEED + DifSpeed;

		Left_Speed_PID.Act  = LeftPulse;			//速度环:编码器实测值
		Right_Speed_PID.Act = RightPulse;
		Left_Speed_PID.Tar  = Left_Speed;			//速度环:循迹环给出的目标
		Right_Speed_PID.Tar = Right_Speed;
		PID_Update(&Left_Speed_PID);
		PID_Update(&Right_Speed_PID);
		Move_SetSpeed(Left_Speed_PID.Out,Right_Speed_PID.Out);
	}
	else											//8路全丢线
	{
		Line.ErrorInt = 0;
		LostCount++;

		if(LostCount < LOSS_DEBOUNCE)
		{
			/*去抖期内不发新指令,电机保持上一拍PWM,避免瞬时丢线被误判成弯*/
		}
		else if(LostCount < LOST_TIMEOUT)			//确认是直角弯:原地转
		{
			if(!Turn_Done)							//一次丢线只计一个弯
			{
				Turn_Done = 1;
				Stop_Count++;
				CornerEvent = 1;
				if(AutoStopEn && Stop_Count > TargetTurns) Finished = 1;
			}

			if(Finished) Move_Stop();
			else if(Line.Error0 < 0) Move_SetSpeed(TURN_SPEED,-TURN_SPEED);		//线在右侧:右转
			else                     Move_SetSpeed(-TURN_SPEED,TURN_SPEED);		//线在左侧:左转
		}
		else
		{
			Move_Stop();							//单次丢线超时,防止无限打转
		}
	}
}

void LineFollow_Show(void)
{
	Serial_Printf("%f,%f,%f,%f\n",Line.Act,Line.Out,Line.Error0,Line.ErrorInt);
}

/*任务三:设置停车弯数并复位计数,发车时调用*/
void LineFollow_SetTargetTurns(uint8_t turns)
{
	TargetTurns = turns;
	AutoStopEn = 1;
	Stop_Count = 0;
	Finished = 0;
	LostCount = 0;
	Turn_Done = 0;
	CornerEvent = 0;	//清残留,防止被任务模块误消费
}

/*发挥一:不按弯数停车,一直巡线直到停车流程接管*/
void LineFollow_DisableAutoStop(void)
{
	AutoStopEn = 0;
	Stop_Count = 0;
	Finished = 0;
	LostCount = 0;
	Turn_Done = 0;
	CornerEvent = 0;
}
