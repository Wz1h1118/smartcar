#ifndef __G1_H
#define __G1_H

/*发挥一:从A点出发边走边识别,收到卡片后算出目标边,
  只有到达目标边后才启用库边检测(检测由Parking.c做)*/

void G1_Launch(void);	//发车时调用:直接起步,卡片边走边收
void G1_Tick(void);		//每20ms:收卡片指令+数直角弯
uint8_t G1_Ready(void);	//1=已到目标边,可以检测库边
void G1_Reset(void);	//切模式时复位

#endif
