#ifndef __LINEFOLLOW_H
#define __LINEFOLLOW_H

/*直行巡线+直角弯模块:从main.c封装出来的核心巡线逻辑*/

void LineFollow_Tick(void);				//每20ms调用:读传感器+巡线/转弯控制
void LineFollow_Show(void);				//OLED+串口显示巡线调试信息,main循环里调用
void LineFollow_SetTargetTurns(uint8_t turns);	//设置停车弯数并复位计数
void LineFollow_DisableAutoStop(void);			//禁用按弯数自动停车(发挥一)

extern volatile uint8_t CornerEvent;	//确认一个直角弯时置1,任务模块消费后清零

#endif
