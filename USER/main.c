//**************LCD*************************************************************************
//	PC0 - DB4				PC10	-	EN
//	PC1	-	DB5				PC11	- RW
//	PC2	-	DB6				PC12	-	RS
//	PC3 - DB7
#include "STM32Lib\\stm32f10x.h"
#include "STM32Lib\\stm32f10x_flash.h"
#include "STM32Lib\\stm32f10x_rtc.h"
#include "STM32Lib\\stm32f10x_bkp.h"
#include "STM32Lib\\stm32f10x_pwr.h"
#include "hal.h"			
#include "USART.h"
//#include "GSM.h"

unsigned char CMD, Enbl, Blink, Countr, StaT;
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

u8 dire = 0; 
//**************************************************


//**************************************************
//void send_one_byte(unsigned char );
//**************************************************
void display_logo(void);
void beep_buzzer (void);
unsigned char CheckLeft(void);
unsigned char CheckRight(void);
void beep_buzzer(void);


//extern 	RxIn;
/******************************************************************************************************************/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////Function void main (); functions: the main function,////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/******************************************************************************************************************/  
int main()
{
		ChipHalInit();			//Chip hardware initialization
		lcd_initialize();		//initialise the lcd
		io_Init();
		
		display_logo();			//display the logo
	  GPIO_SetBits(clean_port,clean_pin);
		delay(0xFFFFFF);
		delay(0xFFFFFF);
   	//Pin_Init( reset_port,	reset_pin,	GPIO_Mode_IPD,GPIO_Speed_50MHz);
	//	SysTickDelay(100);	//Delay of about 10 seconds, wait for the modules to networking  
		beep_buzzer();
		ServoDrive(90);
	  CMD = 0;
		Enbl = 0; 
		Blink = 0;
	  Countr = 0;
		StaT = 0;
		delay(0xFFFFFF);
	delay(0xFFF);
	lcd_gotoxy(1,1);	delay(0xFFF);
	lcd_write_string("SYSTEM READY    ");
	beep_buzzer();
	delay(0xFFF);
	while(1)
		{			
			Z:
			if(Enbl == 0x01)
				GPIO_SetBits(clean_port,clean_pin);
			else
				GPIO_ResetBits(clean_port,clean_pin);
			
			if(Blink == 0)
			{
				GPIO_SetBits(Led1_port,Led1_pin);
				if(Countr < 0xff)
				{
					Countr++;
				}
				else
				{
					Countr = 0;
					Blink = 1;
				}
			}
			if(Blink == 1)
			{
				GPIO_ResetBits(Led1_port,Led1_pin);
				if(Countr < 0xff)
				{
					Countr++;
				}
				else
				{
					Countr = 0;
					Blink = 0;
				}
			}
			//delay(0xFFFFFF);//
			delay(0xFFFFF);
			if(GPIO_ReadInputDataBit(LBS_port,LBS_pin) == 1)
			{
				CMD = movRev();
				delay(0xFFFF);
				CMD = movLeft();
				delay(0xFFFFFF);
				delay(0xFFFFF);
				//delay(0xFFFFFF);
			}
			if(GPIO_ReadInputDataBit(RBS_port,RBS_pin) == 1)
			{
				CMD = movRev();
				delay(0xFFFF);
				CMD = movRgt();
				delay(0xFFFFFF);
				delay(0xFFFFF);
				//delay(0xFFFFFF);
			}
			if(GPIO_ReadInputDataBit(CBS_port,CBS_pin) == 1)
			{
				if(GPIO_ReadInputDataBit(IR_port,IR_pin) == 0)
				{
					lcd_gotoxy(1,1);	delay(0xFFF);
					lcd_write_string("MOVE REVERSE    ");
					CMD = movRev();
					delay(0xFFFFFF);
					goto Z;
				}
				else
				{
					beep_buzzer();
					delay(0xFFF);
					goto Z;
				}
			}
		  ChkDistance();
			lcd_gotoxy(2,1);delay(0xFFF);
			ShowDistance();
			if((Distance > 0xFA) && (GPIO_ReadInputDataBit(stair_port,stair_pin) == 1) && (GPIO_ReadInputDataBit(CBS_port,CBS_pin) == 0))
			{
				lcd_gotoxy(1,1);	delay(0xFFF);
				lcd_write_string("MOVE FORWARD    ");
				CMD = movFwd();
				Enbl = 0x01;
			}
			else
			{
				lcd_gotoxy(1,1);	delay(0xFFF);
				lcd_write_string("STOP DIST. LIMIT");
				Enbl = 0x01;
				CMD = stopMot();
			  if((GPIO_ReadInputDataBit(stair_port,stair_pin) == 1)) //&& (GPIO_ReadInputDataBit(CBS_port,CBS_pin) == 0))
				{	
				  if(dire == 0)
				  {
					  if(CheckLeft()==1)
					  {
						  if(CheckRight()==1)
						  {
							  if(GPIO_ReadInputDataBit(IR_port,IR_pin) == 0)
							  {
								  lcd_gotoxy(1,1);	delay(0xFFF);
								  lcd_write_string("MOVE REVERSE    ");
								  CMD = movRev();
								  delay(0xFFFFFF); 
								  goto Z;
							  }
							  else
							  {
								  beep_buzzer();
								  delay(0xFFF);
									goto Z;
							  }
						  }
					  }
					  goto Z;
				  }
				  else
				  {
					  if(CheckRight()==1)
					  {
						  if(CheckLeft()==1)
						  {
							  if(GPIO_ReadInputDataBit(IR_port,IR_pin) == 0)
							  {
								  lcd_gotoxy(1,1);	delay(0xFFF);
								  lcd_write_string("MOVE REVERSE    ");
								  CMD = movRev();
								  delay(0xFFFFFF);
								  goto Z;
							  }
							  else
							  {
								  beep_buzzer();
								  delay(0xFFF);
									goto Z;
							  }
						  }
					  }
					  goto Z;
				  }
			 }
  		else
			{
				lcd_gotoxy(1,1);	delay(0xFFF);
				lcd_write_string("STOP STAIRCASE  ");
				CMD = stopMot();
				Enbl = 0x00;
				if(GPIO_ReadInputDataBit(IR_port,IR_pin) == 0)
				{
				  GPIO_SetBits(Led2_port,Led2_pin);	
				  lcd_gotoxy(1,1);	delay(0xFFF);
				  lcd_write_string("MOVE REVERSE    ");
				  CMD = movRev();
				  delay(0xFFFFFF);
				}
				else
				{
					beep_buzzer();
					delay(0xFFF);
					goto Z;
			  }	
			}
	 }
 }
}


void delay_key(unsigned int i )
{
	while(i--);
}



/**
  * @brief  Configures GPIO.
  * @param  GPIOx		: Specifies The Output port to be configured. 
  *			GPIO_Pin	: Specifies The	Output Pin To be Configured.
  * 		Mode		: Specifies The Mode Of Operation Of Output Pin.
  * 		Speed		: Specifies The Speed Of Operation Of Output Pin.
  * @retval None
  */

/*void send_one_byte(unsigned char d)
{
   char   a;

   for(a=0;a<8;a++)
   {   //send 8 bits 
      if(d & (0x80 >> a)) 
      {
         GPIO_ResetBits(Port[a],Pin[a]);
      }
			else
				  GPIO_SetBits(Port[a],Pin[a]);
   }
}		 */

void display_logo(void)
{
		lcd_clr();
		lcd_gotoxy(1,1);delay(0xFFF);
		lcd_write_string("   CLEANBOT     ");
		lcd_gotoxy(2,1);delay(0xFFF);
		//send_command(0x0c);delay(0xFFF);
		lcd_write_string("                ");
		delay(0xFFFF);
		
}

void beep_buzzer(void)
{
	unsigned int i;
	for(i=0; i< 500; i++)
	{
		GPIO_SetBits(beep_port,beep_pin);
		delay(500);
		GPIO_ResetBits(beep_port,beep_pin);
	}
}  

unsigned char CheckLeft(void)
{
	dire = 0x01;
	ServoDrive(180);
	delay(0xFFFF);
	ChkDistance();
	lcd_gotoxy(2,1);delay(0xFFF);
	ShowDistance();
	ServoDrive(90);
	if(Distance > 0xFA)
	{
		lcd_gotoxy(1,1);	delay(0xFFF);
		lcd_write_string("MOVE LEFT       ");
		CMD = movRev();
		delay(0xFFFF);
		CMD = movRgt();
		delay(0xFFFFFF);
		delay(0xFFFFF);
		//delay(0xFFFFFF);
		lcd_gotoxy(1,1);	delay(0xFFF);
		lcd_write_string("MOVE FORWARD    ");
		CMD = movFwd();
		Enbl = 0x01;
		return 0x00;
	}
	else
		return 0x01;
}

unsigned char CheckRight(void)
{
	dire = 0;
	ServoDrive(0);
	delay(0xFFFF);
	ChkDistance();
	lcd_gotoxy(2,1);delay(0xFFF);
	ShowDistance();
	ServoDrive(90);
	if(Distance > 0xFA)
	{
		lcd_gotoxy(1,1);	delay(0xFFF);
		lcd_write_string("MOVE RIGHT      ");
		CMD = movRev();
		delay(0xFFFF);
		CMD = movLeft();
		delay(0xFFFFFF);
		delay(0xFFFFF);
		//delay(0xFFFFFF);
		lcd_gotoxy(1,1);	delay(0xFFF);
		lcd_write_string("MOVE FORWARD    ");
		CMD = movFwd();
		Enbl = 0x01;
		return 0x00;				  
	}
	else
		return 0x01;
}


