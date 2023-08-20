/*
 * nvic.h
 *
 *  Created on: Jul 12, 2023
 *      Author: malef
 */

#ifndef NVIC_H_
#define NVIC_H_

#include "core.h"

void NVIC_IRQEnable(IRQn_t IRQn);
void NVIC_IRQDisable(IRQn_t IRQn);
void NVIC_IRQSetPending(IRQn_t IRQn);
void NVIC_IRQClearPending(uint8_t IRQn);

void NVIC_IRQSetPriority(IRQn_t IRQn,uint8_t PriorityNumber);


#endif /* NVIC_H_ */
