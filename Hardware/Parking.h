#ifndef __PARKING_H
#define __PARKING_H

/*停车模块:PC13循环切换6种模式,PC15发车*/
#define MODE_LINE  0	//直行巡线(默认)
#define MODE_SIDE  1	//侧方停车
#define MODE_BACK  2	//倒车入库
#define MODE_AUTO  3	//自动:右光电+超声波距离决定
#define MODE_TASK3 4	//任务三:罗马数字定点停车
#define MODE_G1    5	//发挥一:卡片识别动态车库
#define MODE_COUNT 6

void Parking_Init(void);			//初始化按键+侧面光电,main里调用一次
void Parking_Scan(void);			//每1ms:按键处理+侧面光电触发检测
void Parking_Tick(void);			//每20ms:各模式状态机
uint8_t Parking_Running(void);		//1=停车流程接管电机,巡线让位
uint8_t Parking_Go(void);			//1=已发车,0=待发车(电机保持停止)
uint8_t Parking_GetMode(void);		//当前模式(Serial.c按模式过滤视觉数据)
void Parking_Show(void);			//OLED第1行模式名,第3行光电电平

#endif
