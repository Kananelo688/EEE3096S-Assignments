/*
 * nvic.c
 *
 *  Created on: Jul 12, 2023
 *      Author: malef
 */
#include "nvic.h"

/*
 * Enables Interrupt for Given IRQ number
 */
void NVIC_IRQEnable(IRQn_t IRQn){
	NVIC->ISER |= (0x01<<IRQn);  //Enable Interrupt

}


void NVIC_IRQDisable(IRQn_t IRQn){
	NVIC->ICER |= (0x01<<IRQn);  //Clears Interrupt
}

void NVIC_IRQSetPending(IRQn_t IRQn){
	NVIC->ISPR |= (0x01<<IRQn);  //Pends the Interrupt

}

void NVIC_IRQClearPending(IRQn_t IRQn){
	NVIC->ICPR |= (0x01<<IRQn);    //Clears the pending Interrupt
}

void NVIC_IRQSetPriority(IRQn_t IRQn,uint8_t PriorityNumber){
	uint8_t IPRn = (IRQn/4);  //Get IPRn Register
	uint8_t PRIn = (IRQn%4);  //Get Priority Number

	NVIC->IPR[IPRn] |= (PriorityNumber<<(PRIn*8));  //Set Priority Number
}
