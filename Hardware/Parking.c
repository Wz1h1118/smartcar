#include "stm32f10x.h"                  // Device header
#include "Parking.h"
#include "Key.h"
#include "Photo3.h"
#include "OLED.h"
#include "SmartCar.h"
#include "Ultrasonic.h"
#include "Task3.h"

extern volatile uint8_t Track_Count;	//前方8路当前压线的传感器个数(LineFollow.c)

/*========== 模式(PC13循环:直行→侧方→倒库→自动→任务三) ==========*/
#define MODE_LINE  0	//直行(默认)
#define MODE_SIDE  1	//侧方
#define MODE_BACK  2	//倒库
#define MODE_AUTO  3	//自动:右光电+超声波距离决定
#define MODE_TASK3 4	//任务三:罗马数字识别定点停车
#define MODE_COUNT  5

/*自动模式距离窗口(cm)*/
#define AUTO_SIDE_D_MIN  18
#define AUTO_SIDE_D_MAX  24
#define AUTO_BACK_D_MIN  28
#define AUTO_BACK_D_MAX  34

/*========== 可调参数(1拍=20ms) ==========*/
#define TRIG_LEVEL          1	//光电:白=0 黑=1
#define SIDE_DIR_SWAP       0	//方向又反了再改1

#define SIDE_SLOW_SPEED     30		//侧方:降速直行PWM
#define SIDE_SLOW_TICKS     5		//降速拍数
#define SIDE_TURN_SPEED     150		//原地转向PWM
#define SIDE_TURN1_TICKS    12		//拐向车位拍数
#define SIDE_STRAIGHT_SPEED 200		//直行找边线PWM
#define SIDE_STRAIGHT_TIMEOUT 75	//找边线超时,防止冲出车位
#define SIDE_TURN2_TICKS    15		//顺直拍数

#define BACK_SLOW_SPEED     200		//倒库:减速直行PWM
#define BACK_BACKUP_SPEED   150		//转弯前倒退PWM
#define BACK_BACKUP_TICKS   20		//倒退拍数
#define BACK_TURN_SPEED     100		//原地转90°PWM
#define BACK_TURN_TICKS     50		//转90°拍数
#define BACK_REV_SPEED      100		//倒车PWM
#define BACK_REV_TIMEOUT    400		//找底线超时
#define BACK_FWD_SPEED      150		//前挪PWM
#define BACK_FWD_TICKS      20		//前挪拍数

/*阶段*/
typedef enum { SIDE_IDLE, SIDE_SLOW, SIDE_TURN1, SIDE_STRAIGHT, SIDE_TURN2, SIDE_DONE } SideState_t;
typedef enum { BACK_IDLE, BACK_SLOW, BACK_BACKUP, BACK_TURN, BACK_REV, BACK_FWD, BACK_DONE } BackState_t;

static volatile uint8_t ParkMode = MODE_LINE;
static volatile uint8_t Start = 0;		//0=待发车 1=已发车(PC15)
static volatile uint8_t AutoArmed = 0;	//自动标志:右光电压线后置1,永久锁存

static volatile SideState_t SideState = SIDE_IDLE;
static volatile BackState_t BackState = BACK_IDLE;
static volatile uint8_t SideDir = 0;		//0=左 1=右(侧方/倒库共用)
static volatile uint8_t SideLost = 0;		//侧方直行阶段:车头已离开线
static volatile uint8_t SideArmed = 0;		//手动模式武装:两侧空闲过一次
static volatile uint16_t Tick = 0;			//侧方拍数
static volatile uint16_t BackTick = 0;		//倒库拍数
static volatile uint8_t BackCount = 0;		//倒库:已数到的边线数
static volatile uint8_t EdgeLatched = 0;	//手动倒库:本次压线已计数
static uint8_t TrigCand = 0;				//手动触发候选方向(1左2右)
static uint8_t TrigCnt = 0;					//手动触发去抖
static uint8_t AutoCnt = 0;					//自动标志去抖
static uint8_t BackEdgeCnt = 0;				//自动倒库第二条边线去抖
static uint8_t BackSawIdle = 0;				//自动倒库:已离开线,可数下一条

/*原地转:dir=0左转 1右转*/
static void Turn(uint8_t dir, int16_t speed)
{
	Move_SetSpeed(dir == 0 ? -speed : speed, dir == 0 ? speed : -speed);
}

/*复位两个停车状态机(AutoArmed永久锁存,故意不清)*/
static void ResetParking(void)
{
	SideState = SIDE_IDLE;
	BackState = BACK_IDLE;
	SideLost = 0;
	Tick = 0;
	BackTick = 0;
	BackCount = 0;
	EdgeLatched = 0;
}

void Parking_Init(void)
{
	Key_Init();
	Photo3_Init();
}

uint8_t Parking_Running(void)
{
	if(ParkMode == MODE_LINE) return 0;
	if(ParkMode == MODE_TASK3) return Task3_Running();
	return (SideState != SIDE_IDLE) || (BackState != BACK_IDLE);
}

uint8_t Parking_Go(void)
{
	return Start;
}

void Parking_Scan(void)
{
	/*PC13切模式(停车复位) PC15发车*/
	Key_Tick();
	uint8_t key = Key_GetNum();
	if(key == 1)
	{
		ParkMode = (ParkMode + 1) % MODE_COUNT;
		Start = 0;
		ResetParking();
		Task3_Reset();
		Move_Stop();
	}
	else if(key == 2)
	{
		Start = 1;
		ResetParking();
		if(ParkMode == MODE_TASK3) Task3_Launch();	//任务三:发车时锁存视觉结果
	}

	if(!Start) return;	//未发车

	if(ParkMode == MODE_TASK3) return;	//任务三:无触发检测,由Task3状态机接管

	/*自动模式:右光电压线2ms即置标志;倒库减速期间数第二条边线*/
	if(ParkMode == MODE_AUTO)
	{
		if(BackState == BACK_SLOW)
		{
			if(Photo3_GetRight() != TRIG_LEVEL) { BackEdgeCnt = 0; BackSawIdle = 1; }
			else if(BackSawIdle && ++BackEdgeCnt >= 2)
			{
				BackSawIdle = 0;
				if(++BackCount >= 2) { BackTick = 0; BackState = BACK_BACKUP; }
			}
			return;
		}

		if(SideState != SIDE_IDLE || BackState != BACK_IDLE) return;

		if(Photo3_GetRight() == TRIG_LEVEL)
		{
			if(Track_Count > 0 && ++AutoCnt >= 2)	//仅直线行驶(前方在线)时置标志,转弯碰线不算
			{
				AutoArmed = 1;
				SideDir = 1 ^ SIDE_DIR_SWAP;
			}
		}
		else AutoCnt = 0;
		return;
	}

	/*手动模式公共触发管线*/
	if(ParkMode == MODE_LINE) return;
	if(ParkMode == MODE_SIDE && SideState != SIDE_IDLE) return;
	if(ParkMode == MODE_BACK && BackState != BACK_IDLE && BackState != BACK_SLOW) return;

	uint8_t left  = Photo3_GetLeft();
	uint8_t right = Photo3_GetRight();

	if(left != TRIG_LEVEL && right != TRIG_LEVEL)	//两侧空闲:武装
	{
		TrigCand = 0;
		TrigCnt = 0;
		SideArmed = 1;
		EdgeLatched = 0;
		return;
	}

	uint8_t dir = 0;
	if(left == TRIG_LEVEL && right != TRIG_LEVEL)      dir = 1;
	else if(right == TRIG_LEVEL && left != TRIG_LEVEL) dir = 2;
	else { TrigCand = 0; TrigCnt = 0; return; }			//两侧同时变:忽略

	if(TrigCand == dir) { if(TrigCnt < 2) TrigCnt++; }
	else { TrigCand = dir; TrigCnt = 1; }

	if(TrigCnt < 2 || !SideArmed || Track_Count == 0) return;	//去抖+武装+前方在线

	SideDir = (dir - 1) ^ SIDE_DIR_SWAP;
	SideArmed = 0;

	if(ParkMode == MODE_SIDE)	//侧方:第一次检测即触发
	{
		SideLost = 0;
		Tick = 0;
		SideState = SIDE_SLOW;
	}
	else if(!EdgeLatched)		//倒库:第1条减速,第2条入库
	{
		EdgeLatched = 1;
		if(++BackCount >= 2) { BackTick = 0; BackState = BACK_BACKUP; }
		else                 { BackTick = 0; BackState = BACK_SLOW; }
	}
}

void Parking_Tick(void)
{
	if(!Start || ParkMode == MODE_LINE) return;

	if(ParkMode == MODE_TASK3) { Task3_Tick(); return; }

	/*自动:标志已置,距离进窗口→启动对应停车*/
	if(ParkMode == MODE_AUTO && AutoArmed && SideState == SIDE_IDLE && BackState == BACK_IDLE)
	{
		uint16_t dist = Ultrasonic_GetDistance();
		if(dist >= AUTO_SIDE_D_MIN && dist <= AUTO_SIDE_D_MAX)
		{
			SideLost = 0;
			Tick = 0;
			SideState = SIDE_SLOW;
		}
		else if(dist >= AUTO_BACK_D_MIN && dist <= AUTO_BACK_D_MAX)
		{
			BackCount = 1;		//锁标志的边线算第1条
			BackSawIdle = 0;
			BackTick = 0;
			BackState = BACK_SLOW;
		}
	}

	/*侧方状态机*/
	if(SideState != SIDE_IDLE && (ParkMode == MODE_SIDE || ParkMode == MODE_AUTO))
	{
		switch(SideState)
		{
			case SIDE_SLOW:
				Move_SetSpeed(SIDE_SLOW_SPEED, SIDE_SLOW_SPEED);
				if(++Tick >= SIDE_SLOW_TICKS) { Tick = 0; SideState = SIDE_TURN1; }
				break;

			case SIDE_TURN1:
				Turn(SideDir, SIDE_TURN_SPEED);
				if(++Tick >= SIDE_TURN1_TICKS) { Tick = 0; SideState = SIDE_STRAIGHT; }
				break;

			case SIDE_STRAIGHT:
				Move_SetSpeed(SIDE_STRAIGHT_SPEED, SIDE_STRAIGHT_SPEED);
				if(Track_Count == 0) SideLost = 1;					//先等车头离开线
				if(Track_Count > 0 && SideLost) { Tick = 0; SideState = SIDE_TURN2; }	//再压线
				else if(++Tick > SIDE_STRAIGHT_TIMEOUT) { SideState = SIDE_DONE; }
				break;

			case SIDE_TURN2:
				Turn(SideDir ^ 1, SIDE_TURN_SPEED);
				if(++Tick >= SIDE_TURN2_TICKS) { Tick = 0; SideState = SIDE_DONE; }
				break;

			case SIDE_DONE:
				Move_Stop();
				break;

			default:
				SideState = SIDE_IDLE;
				break;
		}
	}

	/*倒库状态机*/
	if(BackState != BACK_IDLE && (ParkMode == MODE_BACK || ParkMode == MODE_AUTO))
	{
		switch(BackState)
		{
			case BACK_SLOW:
				Move_SetSpeed(BACK_SLOW_SPEED, BACK_SLOW_SPEED);
				break;

			case BACK_BACKUP:
				Move_SetSpeed(-BACK_BACKUP_SPEED, -BACK_BACKUP_SPEED);
				if(++BackTick >= BACK_BACKUP_TICKS) { BackTick = 0; BackState = BACK_TURN; }
				break;

			case BACK_TURN:
				Turn(SideDir ^ 1, BACK_TURN_SPEED);
				if(++BackTick >= BACK_TURN_TICKS) { BackTick = 0; BackState = BACK_REV; }
				break;

			case BACK_REV:
				Move_SetSpeed(-BACK_REV_SPEED, -BACK_REV_SPEED);
				if(Photo3_GetBack() == TRIG_LEVEL) { BackTick = 0; BackState = BACK_FWD; }
				else if(++BackTick > BACK_REV_TIMEOUT) { BackState = BACK_DONE; }
				break;

			case BACK_FWD:
				Move_SetSpeed(BACK_FWD_SPEED, BACK_FWD_SPEED);
				if(++BackTick >= BACK_FWD_TICKS) { BackTick = 0; BackState = BACK_DONE; }
				break;

			case BACK_DONE:
				Move_Stop();
				break;

			default:
				BackState = BACK_IDLE;
				break;
		}
	}
}

void Parking_Show(void)
{
	static const char *ModeName[MODE_COUNT] = {"Line    ", "SidePark", "BackPark", "Auto    ", "Task3   "};
	OLED_ShowString(1,1,(char *)ModeName[ParkMode]);
	OLED_ShowString(3,1,"L");
	OLED_ShowNum(3,2,(uint32_t)Photo3_GetLeft(),1);
	OLED_ShowString(3,4,"R");
	OLED_ShowNum(3,5,(uint32_t)Photo3_GetRight(),1);
	OLED_ShowString(3,7,"B");
	OLED_ShowNum(3,8,(uint32_t)Photo3_GetBack(),1);
	OLED_ShowChar(3,10,AutoArmed ? 'F' : 'f');
}
