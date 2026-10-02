#include "stm32f10x.h"                  // Device header
#include "LineFollow.h"
#include "Track.h"
#include "Encoder.h"
#include "PID.h"
#include "SmartCar.h"
#include "OLED.h"
#include "Serial.h"
#include "Parking.h"

#define BASE_SPEED    60.0				//基础速度(速度环目标:编码器脉冲增量/20ms)
#define TURN_SPEED    300				//直角弯原地转向PWM占空比
#define LOST_TIMEOUT  75				//单次丢线超时,75×20ms≈1.5s
#define LOSS_DEBOUNCE 6					//丢线去抖:连续6×20ms=120ms才判定为真弯道

static volatile uint8_t TargetTurns = 3;	//停车弯数:第(TargetTurns+1)个弯停,直行模式=3,任务三按字母改
static volatile uint8_t AutoStopEn = 1;		//1=按弯数自动停车 0=不停(发挥一用)

/*定速PID参数*/
static PID_t Left_Speed_PID = {
	.Kp = 3.50f,
	.Ki = 0.75f,
	.Kd = 0.20f,
	.OutMax = 1000.0f,
	.OutMin = -1000.0f,
	.ErrorIntMax = 1000.0f
	};

static PID_t Right_Speed_PID = {
	.Kp = 3.50f,
	.Ki = 0.75f,
	.Kd = 0.20f,
	.OutMax = 1000.0f,
	.OutMin = -1000.0f,
	.ErrorIntMax = 1000.0f
	};

/*循迹PID参数*/
static PID_t Line = {
	.Tar = 0,
	.Kp = 4,
	.Ki = 0,
	.Kd = 18,
	.OutMax = 50,
	.OutMin = -50,
	.ErrorIntMax = 75
	};

static float Left_Speed,Right_Speed;
static int16_t LeftPulse,RightPulse;
static float DifSpeed;

/*状态量*/
volatile uint8_t Track_Count,Stop_Count;		//Track_Count供Parking.c判断前方压线
volatile uint8_t CornerEvent = 0;				//每确认一个直角弯置1,由任务模块消费后清零
static volatile uint16_t LostCount = 0;			//本次连续丢线已持续的20ms周期数(看到线即清零)
static volatile uint8_t Turn_Done = 0;			//本次丢线是否已计过弯,保证一个弯只计一次
static volatile uint8_t Finished = 0;			//已走完目标弯数,永久停车

void LineFollow_Tick(void)
{
	/*每20ms读一次编码器和前方灰度(停车流程判断压线也要用Track_Count,先读再让位)*/
	LeftPulse = Encoder_GetLeft();
	RightPulse = Encoder_GetRight();
	Line.Act = Track_GetState();

	if(!Parking_Go())			/*未发车:停车等待,电机不转*/
	{
		Move_Stop();
		return;
	}

	if(Parking_Running()) return;	/*侧方停车执行中:电机由停车状态机接管,巡线让位*/

	if(Finished)							/*已走完目标弯数,永久停车*/
	{
		Move_Stop();
	}
	else if(Track_Count > 0)				/*正常巡线*/
	{
		LostCount = 0;						/*重新看到线:丢线计数清零*/
		Turn_Done = 0;						/*解除本次丢线的已计弯标记*/

		PID_Update(&Line);
		DifSpeed = Line.Out;

		Left_Speed = BASE_SPEED - DifSpeed;
		Right_Speed = BASE_SPEED + DifSpeed;

		/*PID定速调控*/
		Left_Speed_PID.Act = LeftPulse;
		Right_Speed_PID.Act = RightPulse;

		Left_Speed_PID.Tar = Left_Speed;
		Right_Speed_PID.Tar = Right_Speed;

		PID_Update(&Left_Speed_PID);
		PID_Update(&Right_Speed_PID);
		Move_SetSpeed(Left_Speed_PID.Out,Right_Speed_PID.Out);
	}

	else							/*8路全丢线*/
	{
		Line.ErrorInt = 0; /*丢线期间清零*/
		LostCount++;

		if(LostCount < LOSS_DEBOUNCE)		/*去抖:刚丢线期间维持原速冲过去*/
		{
			/*不发新指令,电机保持上一周期PWM,避免瞬时丢线被误判成直角弯*/
		}
		else if(LostCount < LOST_TIMEOUT)	/*确认是直角弯,开始原地转向*/
		{
			if(!Turn_Done)					/*边沿触发:一次丢线只计一个弯*/
			{
				Turn_Done = 1;
				Stop_Count++;
				CornerEvent = 1;			/*通知任务模块:确认了一个直角弯*/
				if(AutoStopEn && Stop_Count > TargetTurns)	/*到目标弯数:停车*/
				{
					Finished = 1;
				}
			}

			if(Finished)
			{
				Move_Stop();				/*第4个弯:停车*/
			}
			else if(Line.Error0 < 0)		/*线最后出现在右侧*/
			{
				Move_SetSpeed(TURN_SPEED,-TURN_SPEED);	/*原地右转*/
			}
			else							/*线最后出现在左侧*/
			{
				Move_SetSpeed(-TURN_SPEED,TURN_SPEED);	/*原地左转*/
			}
		}
		else
		{
			Move_Stop();  /*单次丢线超时,防止无限打转*/
		}
	}
}

/*OLED第3、4行+串口:巡线调试信息*/
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
	CornerEvent = 0;		/*清掉上次运行残留的弯道事件,防止被任务模块误消费*/
}

/*发挥一:不按弯数自动停车,一直巡线直到停车流程接管*/
void LineFollow_DisableAutoStop(void)
{
	AutoStopEn = 0;
	Stop_Count = 0;
	Finished = 0;
	LostCount = 0;
	Turn_Done = 0;
	CornerEvent = 0;		/*同上:发车瞬间清残留,否则G1会立刻误判已到目标边*/
}
