/*
 * lcd.h
 *
 *  Created on: 15 Jul 2023
 *      Author: malef
 */

#ifndef LCD_H_
#define LCD_H_

#include "stm32f051xx.h"
/*bsp exposed apis*/
void lcd_init(void);
void lcd_send_cmd(uint8_t cmd);
void lcd_send_char(uint8_t data);
void lcd_send_str(char* message);
void lcd_clear(void);
void lcd_return_home(void);
void lcd_set_cursor(uint8_t row,uint8_t column);

/*
 * Application configurablr items
 */
/*RS and E*/
#define  LCD_GPIO_PORT0    GPIOC
#define  LCD_GPIO_RS       GPIO_PIN14
#define  LCD_GPIO_EN       GPIO_PIN15
/*D4 and D5*/
#define  LCD_GPIO_PORT1    GPIOB
#define  LCD_GPIO_D4       GPIO_PIN8
#define  LCD_GPIO_D5       GPIO_PIN9

/*D6 and D7*/
#define  LCD_GPIO_PORT2    GPIOA
#define  LCD_GPIO_D6       GPIO_PIN12
#define  LCD_GPIO_D7       GPIO_PIN15


//LCD Commands

#define LCD_CMD_4DL_5X8F      0x28
#define LCD_CMD_DON_CURON     0x0E
#define LCD_CMD_INCADD        0x06
#define LCD_CMD_DIS_CLEAR     0x01
#define LCD_CMD_DIS_RET_HOME  0x02


#endif /* LCD_H_ */
