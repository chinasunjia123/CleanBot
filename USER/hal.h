#ifndef HAL_H
#define HAL_H

#include "STM32Lib\\stm32f10x.h"
#include "STM32Lib\\stm32f10x_bkp.h"
#include "STM32Lib\\stm32f10x_flash.h"
#include "STM32Lib\\stm32f10x_rtc.h"

//#include "GSM.h"

//#define USART_FLAG_TXE                       ((uint16_t)0x0080)
//u8 buf[512];
extern unsigned char  RxIn;
extern unsigned char  Rx;
extern unsigned int	Distance;
extern unsigned char  SystemBuf[];  //Save export receive data
extern GPIO_TypeDef* PORT[];
extern const uint16_t PIN[];
//extern struct Date_s s_DateStructVar;
extern GPIO_TypeDef* Port[];
extern const uint16_t Pin[];

//Hardware initialization
extern void  ChipHalInit(void);
extern void  ChipOutHalInit(void);
extern void Mot1_Fwd(void);
extern void Mot2_Fwd(void);
extern void Mot1_Rev(void);
extern void Mot2_Rev(void);
extern void Mot1_Stop(void);
extern void Mot2_Stop(void);
extern unsigned char movFwd(void);
extern unsigned char movRev(void);
extern unsigned char movLeft(void);
extern unsigned char movRgt(void);
extern unsigned char stopMot(void);
extern void servo(unsigned char F);
extern void ChkDistance(void);
extern void DelaymS(u32 nTime);
extern unsigned char BCD_CONV(unsigned long int CONV_DATA,unsigned char CONV_CNT);
extern void ShowDistance(void);
extern void ServoDrive(unsigned char Dly);
//System clock delay
extern volatile u16 Timer1;
extern void SysTickDelay(u16);

extern void io_Init(void);
extern void Pin_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,GPIOMode_TypeDef Mode,GPIOSpeed_TypeDef Speed);
extern void lcd_initialize(void);
extern void send_command(unsigned char );
extern void send_data(unsigned char );
extern void lcd_gotoxy(unsigned char,unsigned char);

extern void delay(unsigned int);
extern void lcd_write_string(unsigned  char*);
extern void lcd_write_string_year(unsigned char*);
extern void send_four_bits(unsigned char);
extern void lcd_clr(void);

extern void USART1_SendByte(unsigned char );
extern void USART1_Putc(unsigned char date[],unsigned char);

//extern void USART3_SendByte(unsigned char );
//extern void USART3_Putc(unsigned char date[],unsigned char);

#define	PWM1_port					GPIOA
#define	PWM1_pin					GPIO_Pin_8
#define PWM2_port					GPIOA
#define PWM2_pin					GPIO_Pin_11
#define Led2_port					GPIOA
#define Led2_pin					GPIO_Pin_7
#define	beep_port					GPIOA
#define	beep_pin					GPIO_Pin_12
#define enab1_port					GPIOB
#define enab1_pin					GPIO_Pin_0
#define enab2_port					GPIOB
#define enab2_pin					GPIO_Pin_1
#define enab3_port					GPIOB
#define enab3_pin					GPIO_Pin_14
#define enab4_port					GPIOB
#define enab4_pin					GPIO_Pin_15
#define servo_port					GPIOB
#define servo_pin					GPIO_Pin_6
#define eko_port					GPIOB
#define eko_pin						GPIO_Pin_3
#define	trig_port					GPIOB
#define	trig_pin					GPIO_Pin_4
#define stair_port					GPIOB
#define stair_pin					GPIO_Pin_12
#define IR_port						GPIOB
#define IR_pin						GPIO_Pin_5
#define LBS_port					GPIOB
#define LBS_pin						GPIO_Pin_7
#define CBS_port					GPIOB
#define CBS_pin						GPIO_Pin_8
#define RBS_port					GPIOB
#define RBS_pin						GPIO_Pin_13
#define Led1_port					GPIOC
#define Led1_pin					GPIO_Pin_14
#define clean_port					GPIOC
#define clean_pin					GPIO_Pin_13









#endif
