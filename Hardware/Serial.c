#include "stm32f10x.h"                  // Device header
#include "stdio.h"
#include "stdarg.h"
#include "OLED.h"

uint8_t RxFlag;
char RxData;
char Str[100];

uint8_t VisionResult = 0;	//基础任务:1=I 3=III 5=V 7=VII,供任务模块读取

/*发挥一:颜色+形状决定车库位置和停车方式,如 "GBC,REV"*/
uint8_t GarageLoc = 0;		//车库位置:0=无 1=BC边中点 2=CD边中点 3=AD边中点
uint8_t GarageAction = 0;	//停车方式:0=无 1=REV倒车入库 2=PAR侧方停车

void Serial_Init(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	//PA2=USART2_TX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	//PA3=USART2_RX
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate =9600 ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None ;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2,&USART_InitStructure);

	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd =ENABLE ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART2,ENABLE);

}

void Serial_SendByte(uint8_t Byte){
	USART_SendData(USART2,Byte);
	while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
}

void Serial_SendString(char *str){

	for(int i=0;str[i]!='\0';i++){
		Serial_SendByte(str[i]);
	}
}

int fputc(int ch,FILE * f){
	Serial_SendByte(ch);
	return ch;
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

void USART2_IRQHandler(void)
{
	static uint8_t pRxData;
	if(USART_GetITStatus(USART2,USART_IT_RXNE) == SET)
	{
		RxData = USART_ReceiveData(USART2);
		if(RxData != '\r')
		{
			if(pRxData < 99)
			{
				Str[pRxData] = RxData;
				pRxData++;
			}
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

/*OLED第2行显示串口收到的识别结果: V:数字->停车点,收到无法识别的字符显示?*/
void Serial_Show(void)
{
	static uint8_t Bad = 0;			//收到过无法识别的字符

	if(RxFlag)						//收到一行解析
	{
		RxFlag = 0;
		if(Str[0] == 'G')			//发挥一: "GBC,REV" / "GCD,PAR" / "GAD,REV"
		{
			if(Str[1]=='B' && Str[2]=='C')      GarageLoc = 1;
			else if(Str[1]=='C' && Str[2]=='D') GarageLoc = 2;
			else if(Str[1]=='A' && Str[2]=='D') GarageLoc = 3;
			if(Str[3] == ',')
			{
				if(Str[4]=='R')      GarageAction = 1;	//REV=倒车入库
				else if(Str[4]=='P') GarageAction = 2;	//PAR=侧方停车
			}
			Bad = 0;
		}
		else if(Str[0] == 'P' && Str[1] >= 'A' && Str[1] <= 'D')	//基础任务:PA~PD
		{
			static const uint8_t Num[4] = {1, 3, 5, 7};
			VisionResult = Num[Str[1] - 'A'];	//A→1 B→3 C→5 D→7
			Bad = 0;
		}
		else
		{
			uint8_t ch = (uint8_t)Str[0];
			if(ch >= '1' && ch <= '7') ch -= '0';
			if(ch == 1 || ch == 3 || ch == 5 || ch == 7) { VisionResult = ch; Bad = 0; }
			else Bad = 1;	//无法识别:显示问号
		}
	}

	/*第2行显示:收到车库指令优先显示,否则显示基础任务识别结果*/
	if(GarageLoc)
	{
		static const char *Loc[4] = {"--", "BC", "CD", "AD"};
		static const char *Act[3] = {"---", "REV", "PAR"};
		OLED_ShowString(2,1,"G:");
		OLED_ShowString(2,3,(char *)Loc[GarageLoc]);
		OLED_ShowString(2,6,(char *)Act[GarageAction]);
		return;
	}

	char pt;
	if(VisionResult == 1) pt = 'A';
	else if(VisionResult == 3) pt = 'B';
	else if(VisionResult == 5) pt = 'C';
	else if(VisionResult == 7) pt = 'D';
	else pt = Bad ? '?' : '-';
	OLED_ShowString(2,1,"V:");
	OLED_ShowNum(2,3,(uint32_t)VisionResult,1);
	OLED_ShowString(2,5,"->");
	OLED_ShowString(2,6,"  ");
	OLED_ShowChar(2,8,pt);
}
