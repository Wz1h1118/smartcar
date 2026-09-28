#include "stm32f10x.h"                  // Device header
#include "Parking.h"
#include "Key.h"
#include "Photo3.h"
#include "OLED.h"
#include "SmartCar.h"

extern volatile uint8_t Track_Count;	//LineFollow.c定义:前方8路当前压线的传感器个数

/*========== 停车模式(PC13按键切换) ==========*/
#define PARK_SIDE  0	//侧方停车(开机默认)
#define PARK_BACK  1	//倒车入库
static volatile uint8_t ParkMode = PARK_SIDE;

/*========== 侧方停车可调参数(1拍=20ms) ==========*/
#define TRIG_LEVEL          1		//光电电平约定:白色地面=0,黑线=1,电平为1即触发
#define SIDE_DIR_SWAP       1		//转向方向取反开关:0=触发侧即转向侧,1=取反(实测进库方向反了时切换)
#define SIDE_SLOW_SPEED     30		//触发后降速直行的PWM
#define SIDE_SLOW_TICKS     5		//降速直行拍数(让车头越过触发点,留出转向空间)
#define SIDE_TURN_SPEED     150		//入库原地转向PWM
#define SIDE_TURN1_TICKS    12		//第一次转向拍数:拐向车位
#define SIDE_STRAIGHT_SPEED 200		//转向后直行的PWM
#define SIDE_STRAIGHT_TIMEOUT 75	//直行等压线超时拍数(1.5s),超时停车防止冲出车位
#define SIDE_TURN2_TICKS    15		//反向转弯拍数:把车身顺直

/*========== 倒车入库可调参数(1拍=20ms) ==========*/
#define BACK_SLOW_SPEED     200		//第一次检测后减速直行的PWM
#define BACK_BACKUP_SPEED   150		//原地转弯前倒退PWM
#define BACK_BACKUP_TICKS   20		//倒退拍数(让车尾先靠近车库)
#define BACK_TURN_SPEED     100		//倒库原地转向PWM
#define BACK_TURN_TICKS     50		//反方向原地转90°的拍数(实测调)
#define BACK_REV_SPEED      100		//倒车PWM
#define BACK_REV_TIMEOUT    400		//倒车找底线超时拍数(2s),超时停车防止失控
#define BACK_FWD_SPEED      150		//到底线后前挪PWM
#define BACK_FWD_TICKS      20		//前挪拍数

/*侧方停车阶段*/
typedef enum
{
	SIDE_IDLE = 0,		//等待触发(此时正常巡线)
	SIDE_SLOW,			//降速直行
	SIDE_TURN1,			//向车位一侧原地转
	SIDE_STRAIGHT,		//直行,等前方8路压到车位边线
	SIDE_TURN2,			//反向转弯顺直
	SIDE_DONE			//停车完成
} SideState_t;

/*倒车入库阶段*/
typedef enum
{
	BACK_IDLE = 0,		//等待第一次检测(此时正常巡线)
	BACK_SLOW,			//第一次检测后减速直行,等第二次检测
	BACK_BACKUP,		//原地转弯前先倒退一点
	BACK_TURN,			//往车库反方向原地转90°
	BACK_REV,			//倒车,等后方光电检测到底线
	BACK_FWD,			//到底线后稍微往前挪
	BACK_DONE			//停车完成
} BackState_t;

static volatile SideState_t SideState = SIDE_IDLE;
static volatile BackState_t BackState = BACK_IDLE;
static volatile uint8_t SideDir = 0;		//0=左侧 1=右侧(两种停车共用,倒库转向取反)
static volatile uint8_t SideLost = 0;		//SIDE_STRAIGHT阶段:是否已确认车头离开线
static volatile uint8_t SideArmed = 0;		//触发武装:两侧光电都空闲时才置1,防止开机/常触发误触发
static volatile uint16_t Tick = 0;			//侧方阶段拍数
static volatile uint16_t BackTick = 0;		//倒库阶段拍数
static volatile uint8_t BackCount = 0;		//倒库:侧面检测到边线的次数
static volatile uint8_t EdgeLatched = 0;	//倒库:本次压线是否已计过数
static uint8_t TrigCand = 0;				//候选触发方向:1=左 2=右
static uint8_t TrigCnt = 0;					//候选电平连续采样数(1ms采样,连续2次才确认)

void Parking_Init(void)
{
	Key_Init();
	Photo3_Init();
}

uint8_t Parking_Running(void)
{
	if(ParkMode == PARK_SIDE) return (SideState != SIDE_IDLE);
	else                      return (BackState != BACK_IDLE);
}

/*每1ms调用:按键切换+高速采样两侧光电*/
void Parking_Scan(void)
{
	/*PC13按键:侧方/倒库切换(任何时候按下都生效,切换时停车并复位)*/
	Key_Tick();
	if(Key_GetNum() == 1)
	{
		ParkMode ^= 1;
		SideState = SIDE_IDLE;
		BackState = BACK_IDLE;
		SideLost = 0;
		Tick = 0;
		BackTick = 0;
		BackCount = 0;
		EdgeLatched = 0;
		Move_Stop();
	}

	if(ParkMode == PARK_SIDE && SideState != SIDE_IDLE) return;	//停车流程进行中,不再触发
	if(ParkMode == PARK_BACK && BackState != BACK_IDLE && BackState != BACK_SLOW) return;	//减速等第二次检测期间继续采样

	uint8_t left  = Photo3_GetLeft();
	uint8_t right = Photo3_GetRight();

	if(left != TRIG_LEVEL && right != TRIG_LEVEL)	/*两侧空闲(白色地面):武装并清候选/锁存*/
	{
		TrigCand = 0;
		TrigCnt = 0;
		SideArmed = 1;
		EdgeLatched = 0;			//离开线:解除锁存,下一次压线可再计数
		return;
	}

	/*有电平变化:只一侧变化才确认方向,连续2次采样滤掉毛刺*/
	uint8_t dir = 0;
	if(left == TRIG_LEVEL && right != TRIG_LEVEL)      dir = 1;
	else if(right == TRIG_LEVEL && left != TRIG_LEVEL) dir = 2;
	else { TrigCand = 0; TrigCnt = 0; return; }		/*两侧同时变:不确认方向,忽略*/

	if(TrigCand == dir) { if(TrigCnt < 2) TrigCnt++; }
	else { TrigCand = dir; TrigCnt = 1; }

	if(TrigCnt < 2 || !SideArmed || Track_Count == 0) return;	/*去抖+武装+前方在线*/

	SideDir = (dir - 1) ^ SIDE_DIR_SWAP;	//左变化=0,右变化=1,SIDE_DIR_SWAP=1时取反
	SideArmed = 0;

	if(ParkMode == PARK_SIDE)				/*侧方:第一次检测即触发*/
	{
		SideLost = 0;
		Tick = 0;
		SideState = SIDE_SLOW;
	}
	else if(!EdgeLatched)					/*倒库:第一次检测减速,第二次检测启动入库*/
	{
		EdgeLatched = 1;
		BackCount++;
		if(BackCount >= 2)
		{
			BackTick = 0;
			BackState = BACK_BACKUP;
		}
		else								/*第一次检测:减速直行,等第二次检测*/
		{
			BackTick = 0;
			BackState = BACK_SLOW;
		}
	}
}

void Parking_Tick(void)
{
	if(ParkMode == PARK_SIDE)
	{
		if(SideState == SIDE_IDLE) return;	//等待由Parking_Scan触发

		switch(SideState)
		{
			case SIDE_SLOW:						/*先降速直行,把车头送过触发点*/
				Move_SetSpeed(SIDE_SLOW_SPEED, SIDE_SLOW_SPEED);
				if(++Tick >= SIDE_SLOW_TICKS) { Tick = 0; SideState = SIDE_TURN1; }
				break;

			case SIDE_TURN1:					/*向车位一侧原地转*/
				if(SideDir == 0) Move_SetSpeed(-SIDE_TURN_SPEED, SIDE_TURN_SPEED);		/*左转*/
				else             Move_SetSpeed(SIDE_TURN_SPEED, -SIDE_TURN_SPEED);		/*右转*/
				if(++Tick >= SIDE_TURN1_TICKS) { Tick = 0; SideState = SIDE_STRAIGHT; }
				break;

			case SIDE_STRAIGHT:					/*直行,等前方8路压到车位边线(丢线后的上升沿)*/
				Move_SetSpeed(SIDE_STRAIGHT_SPEED, SIDE_STRAIGHT_SPEED);
				if(Track_Count == 0) SideLost = 1;				/*先确认车头已离开线*/
				if(Track_Count > 0 && SideLost)					/*随后再压线:一次完整边沿*/
				{
					Tick = 0;
					SideState = SIDE_TURN2;
				}
				else if(++Tick > SIDE_STRAIGHT_TIMEOUT)			/*超时保护:直接停车*/
				{
					SideState = SIDE_DONE;
				}
				break;

			case SIDE_TURN2:					/*反向原地转,把车身顺直*/
				if(SideDir == 0) Move_SetSpeed(SIDE_TURN_SPEED, -SIDE_TURN_SPEED);		/*右转*/
				else             Move_SetSpeed(-SIDE_TURN_SPEED, SIDE_TURN_SPEED);		/*左转*/
				if(++Tick >= SIDE_TURN2_TICKS) { Tick = 0; SideState = SIDE_DONE; }
				break;

			case SIDE_DONE:						/*入库完成,停车;按PC13切模式或复位可重新开始*/
				Move_Stop();
				break;

			default:
				SideState = SIDE_IDLE;
				break;
		}
	}
	else	/*ParkMode == PARK_BACK:倒车入库*/
	{
		if(BackState == BACK_IDLE) return;	//等待由Parking_Scan触发

		switch(BackState)
		{
			case BACK_SLOW:						/*第一次检测后减速直行,一直等第二次检测(由Parking_Scan升级状态)*/
				Move_SetSpeed(BACK_SLOW_SPEED, BACK_SLOW_SPEED);
				break;

			case BACK_BACKUP:					/*原地转弯前先倒退一点*/
				Move_SetSpeed(-BACK_BACKUP_SPEED, -BACK_BACKUP_SPEED);
				if(++BackTick >= BACK_BACKUP_TICKS) { BackTick = 0; BackState = BACK_TURN; }
				break;

			case BACK_TURN:						/*往车库反方向原地转90°*/
				if(SideDir == 0) Move_SetSpeed(BACK_TURN_SPEED, -BACK_TURN_SPEED);	/*车库在左→右转*/
				else             Move_SetSpeed(-BACK_TURN_SPEED, BACK_TURN_SPEED);	/*车库在右→左转*/
				if(++BackTick >= BACK_TURN_TICKS) { BackTick = 0; BackState = BACK_REV; }
				break;

			case BACK_REV:						/*倒车,等后方光电检测到底线*/
				Move_SetSpeed(-BACK_REV_SPEED, -BACK_REV_SPEED);
				if(Photo3_GetBack() == TRIG_LEVEL)	/*后光电压到黑线:到底线*/
				{
					BackTick = 0;
					BackState = BACK_FWD;
				}
				else if(++BackTick > BACK_REV_TIMEOUT)	/*超时保护:直接停车*/
				{
					BackState = BACK_DONE;
				}
				break;

			case BACK_FWD:						/*到底线后稍微往前挪,离开底线*/
				Move_SetSpeed(BACK_FWD_SPEED, BACK_FWD_SPEED);
				if(++BackTick >= BACK_FWD_TICKS) { BackTick = 0; BackState = BACK_DONE; }
				break;

			case BACK_DONE:						/*入库完成,停车;按PC13切模式或复位可重新开始*/
				Move_Stop();
				break;

			default:
				BackState = BACK_IDLE;
				break;
		}
	}
}

/*OLED:第1行模式名,第2行当前阶段,第3行光电电平L R B+武装标志A*/
void Parking_Show(void)
{
	OLED_ShowString(1,1,ParkMode == PARK_SIDE ? "SidePark" : "BackPark");
	OLED_ShowNum(2,1,(uint32_t)(ParkMode == PARK_SIDE ? (uint8_t)SideState : (uint8_t)BackState),2);
	OLED_ShowNum(3,7,(uint32_t)Photo3_GetLeft(),1);
	OLED_ShowNum(3,9,(uint32_t)Photo3_GetRight(),1);
	OLED_ShowNum(3,11,(uint32_t)Photo3_GetBack(),1);
	OLED_ShowChar(3,15,SideArmed ? 'A' : 'a');
}
