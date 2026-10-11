#include "stm32f10x.h"                  // Device header
#include "G1.h"
#include "LineFollow.h"
#include "Serial.h"

extern volatile uint8_t Track_Count;	//前方8路压线传感器个数

#define G1_DIR_SWAP  0	//方向反了改1(车从A朝D出发时改成1)

static volatile uint8_t G1On = 0;		//1=发挥一进行中
static volatile uint8_t G1Corners = 0;	//已过弯数(独立计数)
static volatile uint8_t G1Target = 0;	//目标弯数:0=还没收到卡片
static volatile uint8_t G1ReadyFlag = 0;//1=已到目标边
static volatile uint8_t G1SawLine = 0;	//发车后是否已见过线(防发车瞬间误计弯)

void G1_Reset(void)
{
	G1On = 0;
	G1Corners = 0;
	G1Target = 0;
	G1ReadyFlag = 0;
	G1SawLine = 0;
}

/*PC15发车时调用:直接起步,不用等卡片*/
void G1_Launch(void)
{
	G1Corners = 0;
	G1Target = 0;
	G1ReadyFlag = 0;
	G1SawLine = 0;
	G1On = 1;
}

void G1_Tick(void)
{
	if(!G1On) return;

	if(Track_Count > 0) G1SawLine = 1;	//先确认压到线才开始数弯

	/*边走边识别:收到卡片后算出还要过几个弯(从当前位置往后数)*/
	if(G1Target == 0 && G1ReadyFlag == 0 && GarageLoc != 0)
	{
		uint8_t t = GarageLoc;					//BC=1 CD=2 AD=3
		if(G1_DIR_SWAP) t = 4 - t;
		if(G1Corners >= t) G1ReadyFlag = 1;		//已经在目标边上
		else G1Target = G1Corners + t;			//还要过t个弯
	}

	if(G1SawLine && CornerEvent)
	{
		CornerEvent = 0;
		G1Corners++;
		if(G1Target != 0 && G1Corners >= G1Target) G1ReadyFlag = 1;	//到目标边
	}
}

uint8_t G1_Ready(void)
{
	return G1ReadyFlag;
}
