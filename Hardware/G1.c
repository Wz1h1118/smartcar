#include "stm32f10x.h"                  // Device header
#include "G1.h"
#include "LineFollow.h"
#include "Serial.h"

extern volatile uint8_t Track_Count;	//前方8路当前压线的传感器个数

#define G1_DIR_SWAP  0	//方向反了改1(车从A朝D出发时改成1)

static volatile uint8_t G1On = 0;		//1=发挥一进行中
static volatile uint8_t G1Corners = 0;	//已过弯数(独立计数)
static volatile uint8_t G1Target = 0;	//目标弯数:过这么多个弯后就到了目标边
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

/*PC15发车时调用:按车库位置算目标弯数
  从A朝B出发:A→弯1(到B)→BC边, 弯2(到C)→CD边, 弯3(到D)→AD边*/
uint8_t G1_Launch(void)
{
	G1Corners = 0;
	G1ReadyFlag = 0;
	G1SawLine = 0;
	if(GarageLoc == 0) { G1On = 0; return 0; }	//没收到车库指令:不发车

	G1Target = GarageLoc;				//BC=1弯后 CD=2弯后 AD=3弯后
	if(G1_DIR_SWAP) G1Target = 4 - G1Target;	//反向出发时映射翻转
	G1On = 1;
	return 1;
}

void G1_Tick(void)
{
	if(!G1On) return;

	if(Track_Count > 0) G1SawLine = 1;	//先确认已经压到线才开始数弯

	if(G1SawLine && CornerEvent)		//消费一个直角弯事件
	{
		CornerEvent = 0;
		if(++G1Corners >= G1Target) G1ReadyFlag = 1;	//到目标边:启用库边检测
	}
}

uint8_t G1_Ready(void)
{
	return G1ReadyFlag;
}
