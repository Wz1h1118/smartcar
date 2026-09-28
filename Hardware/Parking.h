#ifndef __PARKING_H
#define __PARKING_H

/*停车模块:侧方停车+倒车入库,PC13按键切换(开机默认侧方)
  巡线过程中侧面光电触发即进入对应停车流程*/

void Parking_Init(void);				//初始化按键+侧面光电,main里调用一次
void Parking_Scan(void);				//每1ms调用:按键切换+高速采样两侧光电判断触发
void Parking_Tick(void);				//每20ms调用:侧方/倒库状态机
uint8_t Parking_Running(void);			//1=停车动作执行中,主循环应跳过正常巡线
void Parking_Show(void);				//OLED显示当前模式、阶段与光电原始电平

#endif
