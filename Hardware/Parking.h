#ifndef __PARKING_H
#define __PARKING_H

/*停车模块:PC13循环切换6种模式(直行/侧方/倒库/自动/任务三/发挥一),PC15发车*/

void Parking_Init(void);			//初始化按键+侧面光电,main里调用一次
void Parking_Scan(void);			//每1ms:按键处理+侧面光电触发检测
void Parking_Tick(void);			//每20ms:各模式状态机
uint8_t Parking_Running(void);		//1=停车流程接管电机,巡线让位
uint8_t Parking_Go(void);			//1=已发车,0=待发车(电机保持停止)
void Parking_Show(void);			//OLED第1行模式名,第3行光电电平

#endif
