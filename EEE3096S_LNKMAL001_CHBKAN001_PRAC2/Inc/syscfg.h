/*
 * syscfg.h
 *
 *  Created on: 14 Jul 2023
 *      Author: malef
 */

#ifndef SYSCFG_H_
#define SYSCFG_H_

#include "stm32f051xx.h"


typedef enum {
	GPIO_PORTA,
	GPIO_PORTB,
	GPIO_PORTC,
	GPIO_PORTD,
	GPIO_PORTE,
	GPIO_PORTF,

}EXTICR_GPIOx_t;

void EXTI_IT_Config(GPIO_RegDef_t *GPIOx,uint8_t PinNumber);

#endif /* SYSCFG_H_ */
