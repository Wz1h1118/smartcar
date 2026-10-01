#include "stm32f10x.h"                  // Device header
#include "Parking.h"
#include "Key.h"
#include "Photo3.h"
#include "OLED.h"
#include "SmartCar.h"
#include "Ultrasonic.h"

extern volatile uint8_t Track_Count;	//LineFollow.c定义:前方8路当前压线的传感器个数

/*========== 运行模式(PC13按键循环切换:直行→侧方→倒库→自动) ==========*/
#define MODE_LINE  0	//直行巡线(不触发停车,开机默认)
#define MODE_SIDE  1	//侧方停车
#define MODE_BACK  2	//倒车入库
#define MODE_AUTO  3	//自动:右光电+超声波距离决定执行侧方还是倒库
#define MODE_COUNT  4
static volatile uint8_t ParkMode = MODE_LINE;
static volatile uint8_t Start = 0;		//发车标志:0=待发车(停车等待),1=已发车(PC15)
static volatile uint8_t AutoArmed = 0;	//自动模式标志:右光电压线后置1,超声波距离进入窗口时启动停车

/*自动模式超声波距离窗口(cm):范围内才执行对应停车*/
#define AUTO_SIDE_D_MIN  18
#define AUTO_SIDE_D_MAX  24
#define AUTO_BACK_D_MIN  28
#define AUTO_BACK_D_MAX  34

/*========== 侧方停车可调参数(1拍=20ms) ==========*/
#define TRIG_LEVEL          1		//光电电平约定:白色地面=0,黑线=1,电平为1即触发
#define SIDE_DIR_SWAP       0		//方向又反了再改1
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
static uint8_t AutoCnt = 0;					//自动模式:右光电连续读到黑线的采样数(标志位去抖用)
static uint8_t BackEdgeCnt = 0;				//自动倒库:减速期间第二条边线的去抖计数
static uint8_t BackSawIdle = 0;				//自动倒库:减速期间右光电已离开线(可数下一条边线)

void Parking_Init(void)
{
	Key_Init();
	Photo3_Init();
}

uint8_t Parking_Running(void)
{
	if(ParkMode == MODE_LINE) return 0;	//直行模式不接管电机
	return (SideState != SIDE_IDLE) || (BackState != BACK_IDLE);	//自动模式两种流程都算接管
}

uint8_t Parking_Go(void)
{
	return Start;
}

/*每1ms调用:按键切换+高速采样两侧光电*/
void Parking_Scan(void)
{
	/*PC13:循环切换模式(任何时候按下都生效,切换时停车并复位,等待重新发车)
	  PC15:发车(复位停车状态机,车开始运行)*/
	Key_Tick();
	uint8_t key = Key_GetNum();
	if(key == 1)
	{
		ParkMode = (ParkMode + 1) % MODE_COUNT;
		Start = 0;
		SideState = SIDE_IDLE;
		BackState = BACK_IDLE;
		SideLost = 0;
		Tick = 0;
		BackTick = 0;
		BackCount = 0;
		EdgeLatched = 0;
		Move_Stop();	//AutoArmed不清零:标志位永久锁存,只有重新上电才复位
	}
	else if(key == 2)
	{
		Start = 1;
		SideState = SIDE_IDLE;
		BackState = BACK_IDLE;
		SideLost = 0;
		Tick = 0;
		BackTick = 0;
		BackCount = 0;
		EdgeLatched = 0;
	}

	if(!Start) return;	//未发车:不做触发检测,等待PC15发车

	/*自动模式单独处理:右光电压线(1ms采样连续2次确认)即置标志位
	  不要求前方在线/武装/左光电空闲,避免其他条件挡掉锁存*/
	if(ParkMode == MODE_AUTO)
	{
		if(BackState == BACK_SLOW)			/*倒库流程中:减速直行,数第二条边线(专用计数器)*/
		{
			if(Photo3_GetRight() != TRIG_LEVEL) { BackEdgeCnt = 0; BackSawIdle = 1; }
			else if(BackSawIdle && ++BackEdgeCnt >= 2)
			{
				BackSawIdle = 0;			/*本条线已计数,离开线后重新武装*/
				BackCount++;
				if(BackCount >= 2)
				{
					BackTick = 0;
					BackState = BACK_BACKUP;
				}
			}
			return;
		}

		if(SideState != SIDE_IDLE || BackState != BACK_IDLE) return;	//其他流程进行中,不再检测

		if(Photo3_GetRight() == TRIG_LEVEL)
		{
			if(++AutoCnt >= 2)
			{
				AutoArmed = 1;					//标志位永久锁存
				SideDir = 1 ^ SIDE_DIR_SWAP;	//右方触发,方向约定与手动模式一致
			}
		}
		else AutoCnt = 0;
		return;
	}

	if(ParkMode == MODE_LINE) return;	//直行模式:不做触发检测
	if(ParkMode == MODE_SIDE && SideState != SIDE_IDLE) return;	//停车流程进行中,不再触发
	if(ParkMode == MODE_BACK && BackState != BACK_IDLE && BackState != BACK_SLOW) return;	//减速等第二次检测期间继续采样

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

	if(ParkMode == MODE_SIDE)				/*侧方:第一次检测即触发*/
	{
		SideLost = 0;
		Tick = 0;
		SideState = SIDE_SLOW;
	}
	else if(!EdgeLatched)					/*MODE_BACK:第一次检测减速,第二次检测启动入库*/
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
	if(!Start || ParkMode == MODE_LINE) return;	//未发车或直行:交给巡线模块(未发车时巡线也会被Go拦住)

	/*自动模式第二阶段:标志位已置1,且超声波距离进入窗口→启动对应停车(两个流程都空闲时)*/
	if(ParkMode == MODE_AUTO && AutoArmed && SideState == SIDE_IDLE && BackState == BACK_IDLE)
	{
		uint16_t dist = Ultrasonic_GetDistance();
		if(dist >= AUTO_SIDE_D_MIN && dist <= AUTO_SIDE_D_MAX)		/*18~24cm:执行侧方*/
		{
			SideLost = 0;
			Tick = 0;
			SideState = SIDE_SLOW;
		}
		else if(dist >= AUTO_BACK_D_MIN && dist <= AUTO_BACK_D_MAX)	/*28~34cm:倒库,先减速等第二条边线*/
		{
			BackCount = 1;					/*锁标志的边线算第1条*/
			BackSawIdle = 0;				/*先等右光电离开线*/
			BackTick = 0;
			BackState = BACK_SLOW;			/*减速直行,第二条边线由Scan数到后升级*/
		}
		/*标志位不清零:停车启动后仍保持1,距离不在窗口内继续等待*/
	}

	/*侧方状态机:侧方模式或自动模式触发侧方流程后执行*/
	if(SideState != SIDE_IDLE && (ParkMode == MODE_SIDE || ParkMode == MODE_AUTO))
	{
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
	/*倒库状态机:倒库模式或自动模式触发倒库流程后执行*/
	if(BackState != BACK_IDLE && (ParkMode == MODE_BACK || ParkMode == MODE_AUTO))
	{
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

/*OLED:第1行模式名,第2行阶段+发车状态,第3行光电电平+自动标志*/
void Parking_Show(void)
{
	static const char *ModeName[MODE_COUNT] = {"Line    ", "SidePark", "BackPark", "Auto    "};
	OLED_ShowString(1,1,(char *)ModeName[ParkMode]);
	OLED_ShowString(3,1,"L");
	OLED_ShowNum(3,2,(uint32_t)Photo3_GetLeft(),1);
	OLED_ShowString(3,4,"R");
	OLED_ShowNum(3,5,(uint32_t)Photo3_GetRight(),1);
	OLED_ShowString(3,7,"B");
	OLED_ShowNum(3,8,(uint32_t)Photo3_GetBack(),1);
	OLED_ShowChar(3,10,AutoArmed ? 'F' : 'f');	/*自动标志:F=右光电压过线,f=未置1*/
}
