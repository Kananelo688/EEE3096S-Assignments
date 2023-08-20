/*
 * syscfg.c
 *
 *  Created on: 14 Jul 2023
 *      Author: Malefetsane Lenka
 */
#include "syscfg.h"

void EXTI_IT_Config(GPIO_RegDef_t* GPIOx,uint8_t PinNumber){
	SYSCFG_PCLKEN();
	uint8_t EXTICRx = PinNumber/4;
	uint8_t EXTIx_POS = PinNumber%4;

	if (GPIOx==GPIOA){
		SYSCFG->EXTICR[EXTICRx] |= (GPIO_PORTA<<4*EXTIx_POS);
	}
	else if (GPIOx==GPIOB){
		SYSCFG->EXTICR[EXTICRx] |= (GPIO_PORTB<<4*EXTIx_POS);
	}
	else if (GPIOx==GPIOC){
		SYSCFG->EXTICR[EXTICRx] |= (GPIO_PORTC<<4*EXTIx_POS);
	}
	else if (GPIOx==GPIOD){
		SYSCFG->EXTICR[EXTICRx] |= (GPIO_PORTD<<4*EXTIx_POS);
	}
	else if (GPIOx==GPIOE){
		SYSCFG->EXTICR[EXTICRx] |= (GPIO_PORTE<<4*EXTIx_POS);
	}
	else if (GPIOx==GPIOF){
		SYSCFG->EXTICR[EXTICRx] |= (GPIO_PORTF<<4*EXTIx_POS);
	}







}


