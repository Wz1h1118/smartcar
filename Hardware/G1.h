#ifndef __G1_H
#define __G1_H

/*发挥一:从A点出发,用独立计数器数直角弯确定到达哪条边,
  只有到达目标边后才启用库边检测(检测由Parking.c做)*/

uint8_t G1_Launch(void);	//发车时调用:按GarageLoc设置目标弯数,1=启动成功 0=没收到车库指令
void G1_Tick(void);			//每20ms:数弯道(独立计数,不与Stop_Count冲突)
uint8_t G1_Ready(void);		//1=已到目标边,可以检测库边
void G1_Reset(void);		//切模式时复位

#endif
