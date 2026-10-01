#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

/*HC-SR04超声波测距:PB11=Trig,PA8=Echo(5V容忍,直接接;PB8/PB9已被OLED占用)
  测时用内核DWT周期计数器(72MHz),不占用任何定时器*/

void Ultrasonic_Init(void);				//初始化Trig输出+Echo外部中断+DWT,main里调用一次
void Ultrasonic_Trigger(void);			//发一次Trig脉冲,由20ms主循环每5拍(100ms)调用
uint16_t Ultrasonic_GetDistance(void);	//最近一次距离(cm),超时/无效返回0
void Ultrasonic_Show(void);				//OLED第4行显示距离

#endif
