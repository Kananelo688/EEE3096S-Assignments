/*
 * lcd.c
 *
 *  Created on: 15 Jul 2023
 *      Author: malef
 */

#include "lcd.h"
#include "systick.h"
static void lcd_send_nibble(uint8_t value);
static void lcd_enable(void);

void lcd_return_home(void){
	lcd_send_cmd(LCD_CMD_DIS_RET_HOME);
	SysTick_Init(2, EXC_DISABLE, MILLI_SECOND);

}
 void lcd_clear(void){
	lcd_send_cmd(LCD_CMD_DIS_CLEAR);
	SysTick_Init(2, EXC_DISABLE, MILLI_SECOND);

}
void lcd_send_cmd(uint8_t cmd){
/*RS=0 for LCD CMD*/
	GPIO_WritePin(LCD_GPIO_PORT0, LCD_GPIO_RS, PIN_RESET);
	lcd_send_nibble(cmd>>4);    //Upper Nibble
	lcd_send_nibble(cmd&0x0F);  //Lower Nibble

}
void lcd_send_char(uint8_t data){
	//RS =1
	GPIO_WritePin(LCD_GPIO_PORT0, LCD_GPIO_RS, PIN_SET);
	lcd_send_nibble(data>>4);    //Upper Nibble
	lcd_send_nibble(data&0x0F);  //Lower Nibble

}
void lcd_send_str(char* message){
	do{
		lcd_send_char((uint8_t)*message++);

	}while (*message!='\0');
}

static void lcd_send_nibble(uint8_t value){
	/*D5 and D4 1*/
	    GPIO_WritePin(LCD_GPIO_PORT1, LCD_GPIO_D4, ((value>>0)&0x1));
		GPIO_WritePin(LCD_GPIO_PORT1, LCD_GPIO_D5, ((value>>1)&0x1));

	   GPIO_WritePin(LCD_GPIO_PORT2, LCD_GPIO_D6, ((value>>2)&0x1));
	   GPIO_WritePin(LCD_GPIO_PORT2, LCD_GPIO_D7, ((value>>3)&0x1));
	   lcd_enable();





}
void lcd_init(void){
	//1.Configure GPIO pins used for lcd connection

	GPIO_Handle_t lcd_signal;
	//Initialize E and RS
	GPIO_ClockConfig(ENABLE, LCD_GPIO_PORT0);
	lcd_signal.GPIOx = LCD_GPIO_PORT0;
	lcd_signal.GPIOx_Config.PinMode=OutputMode;
	lcd_signal.GPIOx_Config.PinNumber=LCD_GPIO_RS;
	lcd_signal.GPIOx_Config.OutPutType= PUSHPULL;
	lcd_signal.GPIOx_Config.PinSpeed = MEDIUMSPEED;
	lcd_signal.GPIOx_Config.PullUpPullDown = NoPullUpOrPullDown;
	lcd_signal.GPIOx_Config.AFSEL = AF0;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIOx_Config.PinNumber=LCD_GPIO_EN;
	GPIO_Init(&lcd_signal);
	//D4 and D5
	GPIO_ClockConfig(ENABLE, LCD_GPIO_PORT1);
	lcd_signal.GPIOx = LCD_GPIO_PORT1;
	lcd_signal.GPIOx_Config.PinNumber=LCD_GPIO_D4;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIOx_Config.PinNumber=LCD_GPIO_D5;
	GPIO_Init(&lcd_signal);

	//D6 and D7
	GPIO_ClockConfig(ENABLE, LCD_GPIO_PORT2);
	lcd_signal.GPIOx = LCD_GPIO_PORT2;
	lcd_signal.GPIOx_Config.PinNumber=LCD_GPIO_D6;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIOx_Config.PinNumber=LCD_GPIO_D7;
	GPIO_Init(&lcd_signal);






	//LCD initialization
	SysTick_Init(40, EXC_DISABLE,MILLI_SECOND);

	/*RS=0 for LCD CMD*/
	GPIO_WritePin(LCD_GPIO_PORT0, LCD_GPIO_RS, PIN_RESET);

	lcd_send_nibble(0x03);





	SysTick_Init(5, EXC_DISABLE,MILLI_SECOND);
	lcd_send_nibble(0x03);

	SysTick_Init(200, EXC_DISABLE,MICRO_SECOND);
	lcd_send_nibble(0x03);
	lcd_send_nibble(0x02);

	//Function sent cmd
	lcd_send_cmd(LCD_CMD_4DL_5X8F);

	//Display control
	lcd_send_cmd(LCD_CMD_DON_CURON);

	//Dispaly clear
	lcd_clear();

	//Entry Mode Set
	lcd_send_cmd(LCD_CMD_INCADD);


}

void lcd_set_cursor(uint8_t row,uint8_t column){
	column--;
	switch(row){
	case 1:
		//Set Cursot to 1st row amd add index
		lcd_send_cmd((column|=0x80));
		break;
	case 2:
		//set cursor to 2nd row address and add index
		lcd_send_cmd((column|=0xC0));
		break;
	default:
		break;
	}
}

static void lcd_enable(void){
	GPIO_WritePin(LCD_GPIO_PORT0, LCD_GPIO_EN, PIN_SET);
	SysTick_Init(15, EXC_DISABLE,MICRO_SECOND);
	GPIO_WritePin(LCD_GPIO_PORT0, LCD_GPIO_EN, PIN_RESET);
	SysTick_Init(100, EXC_DISABLE,MICRO_SECOND);


}
