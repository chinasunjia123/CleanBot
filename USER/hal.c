/***************************************************
**HAL.c
**Internal peripherals and external peripheral initialization, two major INIT function is mainly used for chip hardware
**Called in MAIN MAIN function to make has nothing to do with the hardware library
***************************************************/
#include "hal.h"
#include "STM32Lib\\stm32f10x.h"
#include "STM32Lib\\stm32f10x_gpio.h"
#include "STM32Lib\\stm32f10x_exti.h"
#include "STM32Lib\\stm32f10x_bkp.h"
#include "STM32Lib\\stm32f10x_pwr.h"
#include "STM32Lib\\stm32f10x_rtc.h"

//#define leds 									8
#define	lcd_lines							4
//#define lcd_on_off_port				GPIOA
#define lcd_line_1_port				GPIOA
#define lcd_line_2_port				GPIOA
#define lcd_line_3_port				GPIOA
#define lcd_line_4_port				GPIOA
//#define lcd_on_off_pin				GPIO_Pin_1  
#define lcd_line_1_pin				GPIO_Pin_3	//GPIO_Pin_0  
#define lcd_line_2_pin				GPIO_Pin_2	//GPIO_Pin_1
#define lcd_line_3_pin        		GPIO_Pin_1	//GPIO_Pin_2
#define lcd_line_4_pin				GPIO_Pin_0	//GPIO_Pin_3
#define en_pin						GPIO_Pin_4
#define rw_pin						GPIO_Pin_5
#define rs_pin						GPIO_Pin_6

#define en_port						GPIOA
#define rw_port						GPIOA
#define rs_port						GPIOA

//**************************************************
 
GPIO_TypeDef* PORT[lcd_lines] = {	lcd_line_1_port, lcd_line_2_port, 
									lcd_line_3_port, lcd_line_4_port, 
								};
const uint16_t PIN[lcd_lines] = {	lcd_line_1_pin, lcd_line_2_pin, 
									lcd_line_3_pin, lcd_line_4_pin,
								};
unsigned char  RxIn=120;	//Define the length of the array received 90
unsigned char Rx=0;
unsigned char  SystemBuf[120];  //Save export receive data
unsigned int	Distance;
/*GPIO_TypeDef* Port[leds] = {   GPIOB, GPIOB, 
                           GPIOB, GPIOB,
                           GPIOB, GPIOB, 
                           GPIOB, GPIOC
                        };
  

const uint16_t Pin[leds] = {   GPIO_Pin_2, GPIO_Pin_10, 
                           GPIO_Pin_11, GPIO_Pin_12, 
                           GPIO_Pin_13, GPIO_Pin_14, 
                           GPIO_Pin_15, GPIO_Pin_6
													}; 
 __IO uint32_t TimeDisplay = 0;
 uint8_t ClockSource;
 uint16_t SummerTimeCorrect; */
//Various internal hardware module configuration function
extern void GPIO_Configuration(void);			//GPIO
extern void RCC_Configuration(void);			//RCC
//extern void ADC_Configuration(void);			//ADC
extern void USART_Configuration(void);			//USART
extern void USART3_Configuration(void);
extern void NVIC_Configuration(void);			//NVIC
extern void TIM_Configuration(void);			//NVIC
void Pin_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,GPIOMode_TypeDef Mode,GPIOSpeed_TypeDef Speed);
//void SysTickDelay(u16);
extern void beep_buzzer (void);
void Mot1_Fwd(void);
void Mot2_Fwd(void);
void Mot1_Rev(void);
void Mot2_Rev(void);
void Mot1_Stop(void);
void Mot2_Stop(void);
unsigned char movFwd(void);
unsigned char movRev(void);
unsigned char movLeft(void);
unsigned char movRgt(void);
unsigned char stopMot(void);
void servo(unsigned char F);
void ChkDistance(void);
void DelaymS(u32 nTime);
unsigned char BCD_CONV(unsigned long int CONV_DATA,unsigned char CONV_CNT);
void ShowDistance(void);
void ServoDrive(unsigned char Dly);

/*******************************
**Function name::ChipHalInit()
**Function: on-chip hardware initialization
*******************************/
void  ChipHalInit(void)
{
	 //Initialization clock source
	RCC_Configuration();
	 	
	//Initialize GPIO
	GPIO_Configuration();

    //ADC initialization	
    //ADC_Configuration();
	 
	//Initialize USART	 	
	USART_Configuration();
	//USART3_Configuration();

	//Initialization NVIC
	NVIC_Configuration();

	//Initialization TIM
	//TIM_Configuration();
	
}

/*********************************
**Function name:ChipOutHalInit()
**Function: chip hardware initialization
*********************************/
void  ChipOutHalInit(void)
{
	
}
//************************************************************
void delay(unsigned int i )
{
	while(i--);
}

void send_four_bits(unsigned char d)
{
	char	a;
	for(a=0;a<4;a++)
	{	//send 4 bits
		if(d & (0x80 >> a))
			GPIO_SetBits(PORT[a],PIN[a]);

		else
			GPIO_ResetBits(PORT[a],PIN[a]);
	}
}

void send_command(unsigned char temp)
{
	unsigned char data1;
	//wait_while_busy();
	data1=temp;
	data1=data1 & 0xf0;
	send_four_bits(data1);
	delay(100);
	GPIO_ResetBits(rs_port,rs_pin);		//   rs=0;
	GPIO_SetBits(en_port,en_pin);		//   en=1;
	delay(100);
	GPIO_ResetBits(en_port,en_pin);		//   en=0;
	delay(100);

	data1=temp<<4;
	data1=data1 & 0xf0;
	send_four_bits(data1);
	delay(100);
	GPIO_ResetBits(rs_port,rs_pin);		//   rs=0;
	GPIO_SetBits(en_port,en_pin);		//   en=1;
	delay(100);
	GPIO_ResetBits(en_port,en_pin);		//   en=0;
	delay(100);

}
//*********************************************
void send_data(unsigned char temp)
{
   unsigned char data1;
	//wait_while_busy();
	data1=temp;
	data1=data1 & 0xf0;
	send_four_bits(data1);
	delay(100);
	GPIO_SetBits(rs_port,rs_pin);		//   rs=1;
	GPIO_SetBits(en_port,en_pin);		//   en=1;
	delay(100);
	GPIO_ResetBits(en_port,en_pin);		//   en=0;
	delay(100);

	data1=temp<<4;
	data1=data1 & 0xf0;
	send_four_bits(data1);
	delay(100);
	GPIO_SetBits(rs_port,rs_pin);		//   rs=1;
	GPIO_SetBits(en_port,en_pin);		//   en=1;
	delay(100);
	GPIO_ResetBits(en_port,en_pin);		//   en=0;
	delay(100);
}
void lcd_clr(void)
{
	delay(7200);
	send_command(0x01);
	delay(7200);
	delay(7200);
}

//**********************************************************************
void lcd_gotoxy(unsigned char row,unsigned char col)
{
	unsigned char j;

	if(row==1)
	j=128;
	if(row==2)
	j=192;

	j=j+col-1;

	send_command(j);
	delay(1000);delay(500);
}

//***********************************************************

void lcd_write_string(unsigned char  *ptr )
{
	while(*ptr!=0)
	{
		send_data(*ptr);
		ptr++;
		delay(1500);delay(1000);
	}
}   
  

void Pin_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,GPIOMode_TypeDef Mode,GPIOSpeed_TypeDef Speed)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  /*Enable Port RCC_APB2 configurations*/
  if(GPIOx == GPIOA)
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  else if(GPIOx == GPIOB)
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  else if(GPIOx == GPIOC)
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  else if(GPIOx == GPIOD)
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
  else if(GPIOx == GPIOE)
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
  else if(GPIOx == GPIOF)
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
  

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin;
  
  GPIO_InitStructure.GPIO_Mode = Mode;
  GPIO_InitStructure.GPIO_Speed = Speed;
  GPIO_Init(GPIOx, &GPIO_InitStructure);
}

void io_Init(void)
{
	//GPIO_InitTypeDef GPIO_InitStructure;
	/* Configure ref. pin as input floating */
  	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  	//GPIO_Init(GPIOB, &GPIO_InitStructure);
	Pin_Init(eko_port,eko_pin,GPIO_Mode_IPD,GPIO_Speed_50MHz);
	Pin_Init(stair_port,stair_pin,GPIO_Mode_IPD,GPIO_Speed_50MHz);
	Pin_Init(IR_port,IR_pin,GPIO_Mode_IPD,GPIO_Speed_50MHz);
	Pin_Init(LBS_port,LBS_pin,GPIO_Mode_IPD,GPIO_Speed_50MHz);
	Pin_Init(CBS_port,CBS_pin,GPIO_Mode_IPD,GPIO_Speed_50MHz);
	Pin_Init(RBS_port,RBS_pin,GPIO_Mode_IPD,GPIO_Speed_50MHz);
	Pin_Init(PWM1_port,PWM1_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz); 
  Pin_Init(PWM2_port,PWM2_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
	Pin_Init(enab1_port,enab1_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz); 
  Pin_Init(enab2_port,enab2_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
	Pin_Init(enab3_port,enab3_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz); 
  Pin_Init(enab4_port,enab4_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
	Pin_Init(servo_port,servo_pin, GPIO_Mode_Out_PP, GPIO_Speed_50MHz); 
  Pin_Init(clean_port,clean_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
	Pin_Init(Led1_port,Led1_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
	Pin_Init(Led2_port,Led2_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);		// GPIOB,GPIO_Pin_4
	Pin_Init(trig_port,trig_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
	Pin_Init(beep_port,beep_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
	
	GPIO_SetBits(PWM1_port,PWM1_pin);
	GPIO_SetBits(PWM2_port,PWM2_pin);
	GPIO_SetBits(enab1_port,enab1_pin);
	GPIO_SetBits(enab2_port,enab2_pin);
	GPIO_SetBits(enab3_port,enab3_pin);
	GPIO_SetBits(enab4_port,enab4_pin);
	GPIO_SetBits(servo_port,servo_pin);
	GPIO_SetBits(trig_port,trig_pin);
	GPIO_ResetBits(Led1_port,Led1_pin);
	//GPIO_ResetBits(Led2_port,Led2_pin);
	GPIO_SetBits(Led2_port,Led2_pin);
	GPIO_SetBits(clean_port,clean_pin);
}

//*******************************************************************
void lcd_initialize()
{
	delay(7200);

  Pin_Init(lcd_line_1_port,lcd_line_1_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
  Pin_Init(lcd_line_2_port,lcd_line_2_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
  Pin_Init(lcd_line_3_port,lcd_line_3_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
  Pin_Init(lcd_line_4_port,lcd_line_4_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
//	Pin_Init(lcd_on_off_port,lcd_on_off_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
  Pin_Init(en_port,en_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
  Pin_Init(rw_port,rw_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
  Pin_Init(rs_port,rs_pin,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
//	GPIO_WriteBit(lcd_on_off_port,lcd_on_off_pin, Bit_SET);
	GPIO_ResetBits(rw_port,rw_pin);
//	GPIO_WriteBit(lcd_on_off_port,lcd_on_off_pin, Bit_RESET);
			
		delay(7200);
	send_command(0x03);
	delay(7200);
	send_command(0x03);
	delay(7200);
	send_command(0x03);
	delay(7200);

	send_command(0x02);
	delay(7200);
	send_command(0x02);
	delay(7200);
	send_command(0x28);
	delay(7200);
	send_command(0x01);
	delay(7200);

	send_command(0x06);
	delay(7200);
	send_command(0x0C);
	delay(7200);

	//send_command(0x10,selected_lcd);
	send_command(0x80);
	delay(7200);

	//send_command(0x06,selected_lcd);
	//send_command(0x0f,selected_lcd);
	send_command(0x0c);
	delay(7200);
	delay(7200);
}

unsigned char movFwd(void)
{
	 GPIO_SetBits(enab2_port,enab2_pin);
	 GPIO_SetBits(enab4_port,enab4_pin);
	 delay(500);
	 GPIO_ResetBits(enab1_port,enab1_pin);
	 GPIO_ResetBits(enab3_port,enab3_pin);
	 delay(500);
	 GPIO_ResetBits(PWM1_port,PWM1_pin);
	 GPIO_ResetBits(PWM2_port,PWM2_pin);
	 return 0x01;
}	 

unsigned char movRev(void)
{
	 GPIO_SetBits(enab1_port,enab1_pin);
	 GPIO_SetBits(enab3_port,enab3_pin);
	 delay(500);
	 GPIO_ResetBits(enab2_port,enab2_pin);
	 GPIO_ResetBits(enab4_port,enab4_pin);
	 delay(500);
	 GPIO_ResetBits(PWM1_port,PWM1_pin);
	 GPIO_ResetBits(PWM2_port,PWM2_pin);
	 return 0x02;
}

unsigned char movLeft(void)
{
	 GPIO_SetBits(enab2_port,enab2_pin);
	 GPIO_SetBits(enab3_port,enab3_pin);
	 //GPIO_SetBits(enab4_port,enab4_pin);		// changed on 12.04.16
	 //GPIO_SetBits(PWM2_port,PWM2_pin);			// changed on 12.04.16
	 delay(500);
	 GPIO_ResetBits(enab1_port,enab1_pin);
	 GPIO_ResetBits(enab4_port,enab4_pin);		// changed on 12.04.16
	 delay(500);
	 GPIO_ResetBits(PWM1_port,PWM1_pin);
	 GPIO_ResetBits(PWM2_port,PWM2_pin);				// changed on 12.04.16
	 return 0x03;
}

unsigned char movRgt(void)
{
	GPIO_SetBits(enab1_port,enab1_pin);
	//GPIO_SetBits(enab2_port,enab2_pin);   // changed on 12.04.16
	GPIO_SetBits(enab4_port,enab4_pin);
	//GPIO_SetBits(PWM1_port,PWM1_pin);				// changed on 12.04.16
	delay(500);
	GPIO_ResetBits(enab3_port,enab3_pin);
	GPIO_ResetBits(enab2_port,enab2_pin);   // changed on 12.04.16
	delay(500);
	GPIO_ResetBits(PWM2_port,PWM2_pin);
	GPIO_ResetBits(PWM1_port,PWM1_pin);								// changed on 12.04.16
	//delay(500);
	return 0x04;
}

unsigned char stopMot(void)
{
	GPIO_SetBits(enab1_port,enab1_pin);
	GPIO_SetBits(enab2_port,enab2_pin);
	GPIO_SetBits(enab3_port,enab3_pin);
	GPIO_SetBits(enab4_port,enab4_pin);
	GPIO_SetBits(PWM1_port,PWM1_pin);
	GPIO_SetBits(PWM2_port,PWM2_pin);
	delay(500);
	return 0x00;
}

void servo(unsigned char F)
{
	switch(F)
	{
		case 0: 	GPIO_ResetBits(servo_port,servo_pin);
					delay(15);
					GPIO_SetBits(servo_port,servo_pin);
					delay(180);
					break;
		case 90: 	GPIO_ResetBits(servo_port,servo_pin);
					delay(15);
					GPIO_SetBits(servo_port,servo_pin);
					delay(180);
					break;
		case 180: 	GPIO_ResetBits(servo_port,servo_pin);
					delay(15);
					GPIO_SetBits(servo_port,servo_pin);
					delay(180); 
					break;
	}
}

void ChkDistance(void)
{
			Distance = 0;
			GPIO_ResetBits(trig_port,trig_pin);
			delay(0x7FFF);
			//delay(10000);//DelaymS(30);						//Ensure more than 10uS
			GPIO_SetBits(trig_port,trig_pin);
			while(GPIO_ReadInputDataBit(eko_port,eko_pin)==0){}
			//delay(10);
			while(GPIO_ReadInputDataBit(eko_port,eko_pin)==1){
				Distance++;
				//delay(15);
			}
			
			Distance = Distance * 0.17;
			return;
}

void DelaymS(u32 nTime)
{ 
  u32 TimingDelay = nTime*100;

  while(TimingDelay != 0);
}

unsigned char BCD_CONV(unsigned long int CONV_DATA,unsigned char CONV_CNT)
{
	unsigned long int TEMP_VALUE;
	unsigned char TEMP_CNT;

	for(TEMP_CNT=0;TEMP_CNT<CONV_CNT;TEMP_CNT++)
	{
		CONV_DATA					=	(((CONV_DATA/0x0A)*0x10)+(CONV_DATA%0x0A));
		TEMP_VALUE					=	CONV_DATA%0x10;
		CONV_DATA					=	CONV_DATA/0x10;
	}

	return(TEMP_VALUE+0x30);
}

void ShowDistance(void)
{
	unsigned char dd;
		dd = BCD_CONV(Distance,5);
		send_data(dd);
		dd = BCD_CONV(Distance,4);
		send_data(dd);
		dd = BCD_CONV(Distance,3);
		send_data(dd);
		dd = BCD_CONV(Distance,2);
		send_data(dd);
		dd = BCD_CONV(Distance,1);
		send_data(dd);
}

void ServoDrive(unsigned char Dly)
{
	unsigned int a,n;
		switch(Dly)
		{
			case 0:   a = 100; break;
			case 90:  a = 0x37ff; break;
			case 180: a = 0x5dff; break;
			default: return;
		}
		for(n = 0; n < 0xff; n++)
		{
			GPIO_ResetBits(servo_port,servo_pin);
			delay(a);
			GPIO_SetBits(servo_port,servo_pin);
			delay((0xffff - a));
		}
}
//*************************************************************




