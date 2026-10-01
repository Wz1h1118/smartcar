#include "stm32f10x.h"                  // Device header
#include "Ultrasonic.h"
#include "Delay.h"

#define TRIG_GPIO    GPIOB
#define TRIG_PIN     GPIO_Pin_11		//PB11 Trig,推挽输出(PB8/PB9已被OLED的I2C占用)
#define ECHO_GPIO    GPIOA
#define ECHO_PIN     GPIO_Pin_8			//PA8 Echo,5V容忍引脚,浮空输入

#define MAX_WIDTH_US 24000				//有效回波上限(约4m量程),超过判无效

static volatile uint32_t RiseTime = 0;	//Echo上升沿时的DWT计数值
static volatile uint16_t Distance = 0;	//最近一次距离(cm),0=无效/未测量
static volatile uint8_t  EchoOn = 0;	//已收到上升沿,等待下降沿

/*项目core_cm3.h没有定义DWT,这里补最小定义*/
typedef struct
{
	__IO uint32_t CTRL;		/*0x00 控制寄存器*/
	__IO uint32_t CYCCNT;	/*0x04 周期计数器*/
	__IO uint32_t CPICNT;	/*0x08*/
	__IO uint32_t EXCCNT;	/*0x0C*/
	__IO uint32_t SLEEPCNT;	/*0x10*/
	__IO uint32_t LSUCNT;	/*0x14*/
	__IO uint32_t FOLDCNT;	/*0x18*/
	__IO uint32_t PCSR;		/*0x1C*/
} DWT_Type;

#define DWT_BASE                (0xE0001000UL)
#define DWT                     ((DWT_Type *)DWT_BASE)
#define DWT_CTRL_CYCCNTENA_Msk  (1UL << 0)	/*CTRL bit0:使能周期计数器*/

/*使能内核DWT周期计数器:72MHz主频下每72个数=1us*/
static void DWT_Init(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void Ultrasonic_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	/*PB11 Trig:推挽输出,初始低电平*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = TRIG_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(TRIG_GPIO, &GPIO_InitStructure);
	GPIO_ResetBits(TRIG_GPIO, TRIG_PIN);

	/*PA8 Echo:浮空输入+外部中断,上升沿和下降沿都触发*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = ECHO_PIN;
	GPIO_Init(ECHO_GPIO, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource8);
	EXTI_InitStructure.EXTI_Line = EXTI_Line8;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//低于TIM1的0优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DWT_Init();
}

/*发一次Trig:10us高脉冲,HC-SR04收到后发射8个40kHz超声脉冲*/
void Ultrasonic_Trigger(void)
{
	if(EchoOn)						/*上次回波没回来(100ms内):判无效并复位*/
	{
		EchoOn = 0;
		Distance = 0;
	}
	GPIO_SetBits(TRIG_GPIO, TRIG_PIN);
	Delay_us(10);
	GPIO_ResetBits(TRIG_GPIO, TRIG_PIN);
}

uint16_t Ultrasonic_GetDistance(void)
{
	return Distance;
}

/*PB8外部中断:上升沿记起点,下降沿算距离*/
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line8) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line8);
		if(GPIO_ReadInputDataBit(ECHO_GPIO, ECHO_PIN))			/*上升沿*/
		{
			RiseTime = DWT->CYCCNT;
			EchoOn = 1;
		}
		else if(EchoOn)											/*下降沿:计算距离*/
		{
			uint32_t width_us = (DWT->CYCCNT - RiseTime) / 72;	/*72MHz:72个数=1us*/
			Distance = (width_us <= MAX_WIDTH_US) ? (uint16_t)(width_us / 58) : 0;	/*58us=1cm*/
			EchoOn = 0;
		}
	}
}
