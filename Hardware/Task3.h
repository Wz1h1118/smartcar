#ifndef __TASK3_H
#define __TASK3_H

/*任务三:罗马数字识别定点停车
  发车后直行出库,前方8路压线即转弯(A/B左转 C/D右转),转完交给巡线;
  停车弯数:III→B / V→C = 第1个弯停,I→A / VII→D = 第2个弯停*/

void Task3_Launch(void);		//发车时调用:设置停车弯数+出库转弯方向
void Task3_Tick(void);			//每20ms:出库直行/压线转弯阶段
void Task3_Reset(void);			//切模式时复位
uint8_t Task3_Running(void);	//1=任务接管电机(未识别保持停/出库/转弯中)

#endif
