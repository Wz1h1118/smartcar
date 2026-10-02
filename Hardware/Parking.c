#include "stm32f10x.h"                  // Device header
#include "Parking.h"
#include "Key.h"
#include "Photo3.h"
#include "OLED.h"
#include "SmartCar.h"
#include "Ultrasonic.h"
#include "Task3.h"
#include "G1.h"
#include "LineFollow.h"	//线数停车/巡线控制
#include "Serial.h"		//VisionResult/GarageLoc/GarageAction

extern volatile uint8_t Track_Count;	//前方8路当前压线的传感器个数(LineFollow.c)

/*========== 模式(PC13循环:直行→侧方→倒库→自动→任务三→发挥一) ==========*/
#define MODE_LINE  0	//直行(默认)
#define MODE_SIDE  1	//侧方
#define MODE_BACK  2	//倒库
#define MODE_AUTO  3	//自动:右光电+超声波距离决定
#define MODE_TASK3 4	//任务三:罗马数字识别定点停车
#define MODE_G1    5	//发挥一:卡片识别动态车库
#define MODE_COUNT  6

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

/*倒库减速期间:数第二条边线,数到就进入倒退
  useBoth=1左右任一触发都算,=0只看右光电(自动模式)*/
static void BackSlowCount(uint8_t useBoth)
{
	uint8_t trig = useBoth ? ((Photo3_GetLeft() == TRIG_LEVEL) || (Photo3_GetRight() == TRIG_LEVEL))
	                       : (Photo3_GetRight() == TRIG_LEVEL);
	if(!trig) { BackEdgeCnt = 0; BackSawIdle = 1; }
	else if(BackSawIdle && ++BackEdgeCnt >= 2)
	{
		BackSawIdle = 0;
		if(++BackCount >= 2) { BackTick = 0; BackState = BACK_BACKUP; }
	}
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
	/*PC13:短按在Task3模式循环切换模拟停车点A→B→C→D,其他模式切模式;长按1秒切模式
	  PC15:发车*/
	Key_Tick();
	uint8_t key = Key_GetNum();
	uint8_t keyLong = Key_GetLong();
	if(key == 1 && (ParkMode == MODE_TASK3 || ParkMode == MODE_G1))	//短按循环模拟视觉结果
	{
		/*10档循环:4个基础停车点 + 发挥一6种卡片组合*/
		static uint8_t SimIdx = 0;
		SimIdx = (SimIdx + 1) % 10;
		if(SimIdx < 4)							//基础任务:PA~PD
		{
			static const uint8_t Num[4] = {1, 3, 5, 7};
			VisionResult = Num[SimIdx];
			GarageLoc = 0;
			GarageAction = 0;
		}
		else									//发挥一:2色×3形=6种组合
		{
			/*红=倒库(REV) 黄=侧停(PAR);圆=BC 三角=CD 方=AD*/
			static const uint8_t Loc[6] = {1, 1, 2, 2, 3, 3};	//BC BC CD CD AD AD
			static const uint8_t Act[6] = {1, 2, 1, 2, 1, 2};	//REV PAR REV PAR REV PAR
			GarageLoc = Loc[SimIdx - 4];
			GarageAction = Act[SimIdx - 4];
			VisionResult = 0;
		}
	}
	else if(key == 1 || keyLong == 1)			//短按(其他模式)或长按:切换模式
	{
		ParkMode = (ParkMode + 1) % MODE_COUNT;
		Start = 0;
		ResetParking();
		Task3_Reset();
		G1_Reset();
		LineFollow_SetTargetTurns(3);	//恢复默认:按弯数自动停车
		Move_Stop();
	}
	else if(key == 2)
	{
		ResetParking();
		if(ParkMode == MODE_TASK3)					//任务三:发车时锁存视觉结果
		{
			Start = 1;
			Task3_Launch();
		}
		else if(ParkMode == MODE_G1)				//发挥一:没收到车库指令不发车
		{
			if(G1_Launch())
			{
				Start = 1;
				LineFollow_DisableAutoStop();		//一直巡线,停车由停车流程接管
			}
		}
		else
		{
			Start = 1;
		}
	}

	if(!Start) return;	//未发车

	if(ParkMode == MODE_TASK3) return;	//任务三:无触发检测,由Task3状态机接管

	/*自动模式:右光电压线2ms即置标志;倒库减速期间数第二条边线*/
	if(ParkMode == MODE_AUTO)
	{
		if(BackState == BACK_SLOW) { BackSlowCount(0); return; }

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

	/*发挥一:数弯由G1负责,只有到达目标边后才启用下面的库边检测*/
	if(ParkMode == MODE_G1)
	{
		if(BackState == BACK_SLOW) { BackSlowCount(1); return; }	//倒库减速等第二条边线
		if(!G1_Ready() || SideState != SIDE_IDLE || BackState != BACK_IDLE) return;
	}
	else
	{
		if(ParkMode == MODE_LINE) return;
		if(ParkMode == MODE_SIDE && SideState != SIDE_IDLE) return;
		if(ParkMode == MODE_BACK && BackState != BACK_IDLE && BackState != BACK_SLOW) return;
	}

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
	else if(ParkMode == MODE_G1)	//发挥一:按卡片识别的停车方式启动对应流程
	{
		if(GarageAction == 2)		//PAR:侧方
		{
			SideLost = 0;
			Tick = 0;
			SideState = SIDE_SLOW;
		}
		else						//REV:倒库,先减速等第二条边线
		{
			BackCount = 1;			//触发这条算第1条
			BackSawIdle = 0;
			BackTick = 0;
			BackState = BACK_SLOW;
		}
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

	if(ParkMode == MODE_G1) G1_Tick();	//发挥一:数直角弯,到目标边后置就绪

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
	if(SideState != SIDE_IDLE && (ParkMode == MODE_SIDE || ParkMode == MODE_AUTO || ParkMode == MODE_G1))
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
	if(BackState != BACK_IDLE && (ParkMode == MODE_BACK || ParkMode == MODE_AUTO || ParkMode == MODE_G1))
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
	static const char *ModeName[MODE_COUNT] = {"Line    ", "SidePark", "BackPark", "Auto    ", "Task3   ", "Adv1    "};
	OLED_ShowString(1,1,(char *)ModeName[ParkMode]);
	OLED_ShowString(3,1,"L");
	OLED_ShowNum(3,2,(uint32_t)Photo3_GetLeft(),1);
	OLED_ShowString(3,4,"R");
	OLED_ShowNum(3,5,(uint32_t)Photo3_GetRight(),1);
	OLED_ShowString(3,7,"B");
	OLED_ShowNum(3,8,(uint32_t)Photo3_GetBack(),1);
	OLED_ShowChar(3,10,AutoArmed ? 'F' : 'f');
}
