#ifndef __PARKING_H
#define __PARKING_H

/*停车模块:PC13按键循环切换4种模式(开机默认直行)
  直行→侧方→倒库→自动(右光电触发+超声波距离18~24侧方/28~34倒库)*/

void Parking_Init(void);				//初始化按键+侧面光电,main里调用一次
void Parking_Scan(void);				//每1ms调用:PC13切模式/PC15发车+高速采样两侧光电判断触发
void Parking_Tick(void);				//每20ms调用:侧方/倒库状态机
uint8_t Parking_Running(void);			//1=停车动作执行中,主循环应跳过正常巡线
uint8_t Parking_Go(void);				//1=已发车(PC15按下过),0=待发车,电机应保持停止
void Parking_Show(void);				//OLED显示当前模式、阶段与光电原始电平

#endif
