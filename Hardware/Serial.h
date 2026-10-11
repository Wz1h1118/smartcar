#ifndef __SERIAL_H
#define __SERIAL_H

/*视觉模块串口:USART2,PA2=TX PA3=RX,9600*/

void Serial_Init(void);
void Serial_Printf(char *format, ...);
void Serial_Show(void);
void Serial_ResetVision(uint8_t mode);	//重置识别结果并给该模式默认值(切模式时调用)

extern uint8_t VisionResult;	//基础任务:1=I 3=III 5=V 7=VII
extern uint8_t GarageLoc;		//发挥一:1=BC 2=CD 3=AD
extern uint8_t GarageAction;	//发挥一:1=REV倒库 2=PAR侧停

#endif
