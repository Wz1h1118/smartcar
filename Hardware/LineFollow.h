#ifndef __LINEFOLLOW_H
#define __LINEFOLLOW_H

/*直行巡线+直角弯模块:从main.c封装出来的核心巡线逻辑*/

void LineFollow_Tick(void);				//每20ms调用:读传感器+巡线/转弯控制
void LineFollow_Show(void);				//OLED+串口显示巡线调试信息,main循环里调用

#endif
