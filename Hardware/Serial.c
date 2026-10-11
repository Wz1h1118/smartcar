#include "stm32f10x.h"                  // Device header
#include "stdio.h"
#include "stdarg.h"
#include "OLED.h"
#include "Parking.h"	//Parking_GetMode/MODE_xxx

uint8_t RxFlag;			//收满一行(以\r结尾)标志
char RxData;			//最近收到的字节
char Str[100];			//接收缓冲

uint8_t VisionResult = 0;	//基础任务:1=I 3=III 5=V 7=VII
uint8_t GarageLoc = 0;		//发挥一车库位置:1=BC 2=CD 3=AD
uint8_t GarageAction = 0;	//发挥一停车方式:1=REV倒库 2=PAR侧停

/*切模式时重置识别结果:任务三给默认值(摄像头没数据时也能直接测),
  发挥一不给默认,必须等摄像头识别出卡片*/
void Serial_ResetVision(uint8_t mode)
{
	VisionResult = (mode == MODE_TASK3) ? 1 : 0;	//任务三默认 I→A点
	GarageLoc = 0;
	GarageAction = 0;
}

void Serial_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;		//PA2=USART2_TX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;		//PA3=USART2_RX
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2,&USART_InitStructure);

	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART2,ENABLE);
}

void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART2,Byte);
	while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
}

void Serial_SendString(char *str)
{
	while(*str) Serial_SendByte(*str++);
}

void Serial_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	Serial_SendString(String);
}

/*按\r分帧:每收满一行置RxFlag,由Serial_Show解析*/
void USART2_IRQHandler(void)
{
	static uint8_t pRxData;
	if(USART_GetITStatus(USART2,USART_IT_RXNE) == SET)
	{
		RxData = USART_ReceiveData(USART2);
		if(RxData != '\r')
		{
			if(pRxData < 99) Str[pRxData++] = RxData;
		}
		else
		{
			Str[pRxData] = '\0';
			pRxData = 0;
			RxFlag = 1;
		}
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
	}
}

/*按当前模式解析并显示:
  任务三=只认罗马数字结果(PA~PD或1/3/5/7), 发挥一=只认车库指令(Gxx,yyy),
  其他模式不接收视觉数据*/
void Serial_Show(void)
{
	uint8_t mode = Parking_GetMode();

	/*在整条缓冲里搜索合法格式,容忍前面有杂散字符(半截帧残留/多余的\n等),
	  否则一次丢字节会连累下一帧也被判废*/
	if(RxFlag)
	{
		RxFlag = 0;

		if(mode == MODE_TASK3)						//任务三:只认罗马数字
		{
			static const uint8_t Num[4] = {1,3,5,7};
			for(uint8_t i = 0; Str[i] != '\0'; i++)
			{
				if(Str[i] == 'P' && Str[i+1] >= 'A' && Str[i+1] <= 'D')
				{
					VisionResult = Num[Str[i+1] - 'A'];
					break;
				}
				else if(Str[i] >= '1' && Str[i] <= '7' && ((Str[i] - '0') & 1))
				{
					VisionResult = Str[i] - '0';	//1/3/5/7都是奇数
					break;
				}
			}
		}
		else if(mode == MODE_G1)					//发挥一:只认车库指令
		{
			for(uint8_t i = 0; Str[i] != '\0'; i++)
			{
				if(Str[i] != 'G') continue;
				uint8_t loc = 0;
				if(Str[i+1]=='B' && Str[i+2]=='C')      loc = 1;
				else if(Str[i+1]=='C' && Str[i+2]=='D') loc = 2;
				else if(Str[i+1]=='A' && Str[i+2]=='D') loc = 3;
				if(loc == 0 || Str[i+3] != ',') continue;

				GarageLoc = loc;
				if(Str[i+4]=='R')      GarageAction = 1;	//REV=倒车入库
				else if(Str[i+4]=='P') GarageAction = 2;	//PAR=侧方停车
				break;
			}
		}
	}

	/*第2行只在对应模式下显示:没有有效结果一律显示?*/
	if(mode == MODE_G1)
	{
		static const char *Loc[4] = {"??","BC","CD","AD"};
		static const char *Act[3] = {"???","REV","PAR"};
		OLED_ShowString(2,1,"G:");
		OLED_ShowString(2,3,(char *)Loc[GarageLoc]);
		OLED_ShowString(2,6,(char *)Act[GarageAction]);
	}
	else if(mode == MODE_TASK3)
	{
		char pt;
		if(VisionResult == 1) pt = 'A';
		else if(VisionResult == 3) pt = 'B';
		else if(VisionResult == 5) pt = 'C';
		else if(VisionResult == 7) pt = 'D';
		else pt = '?';
		OLED_ShowString(2,1,"V:");
		OLED_ShowNum(2,3,(uint32_t)VisionResult,1);
		OLED_ShowString(2,5,"->");
		OLED_ShowString(2,6,"  ");
		OLED_ShowChar(2,8,pt);
	}
	else
	{
		OLED_ShowString(2,1,"                ");	//其他模式不显示视觉数据
	}
}
