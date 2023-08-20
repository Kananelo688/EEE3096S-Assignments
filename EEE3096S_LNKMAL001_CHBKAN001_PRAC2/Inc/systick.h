/*
 * systick.h
 *
 *  Created on: 14 Jul 2023
 *      Author: Malefetsane
 */


#ifndef SYSTICK_H_
#define SYSTICK_H_
#include "core.h"
#include "stm32f051xx.h"

#define MS_DELAY          8000
#define US_DELAY          8
#define EXC_ENABLE        ENABLE
#define EXC_DISABLE       DISABLE

#define MICRO_SECOND     DISABLE
#define MILLI_SECOND     ENABLE

/*
 *SysTick Control and status register Macros
 */
#define SYSTICK_CSR_EN                      (0x01UL<<0)
#define SYSTICK_CSR_EXCEPTION_EN             (0x01UL<<1)
#define SYSTICK_PROCESSOR_CLK                (0x01UL<<2)
#define SYSTICK_CSR_COUNTFLAG                (0x01UL<<16)


void SysTick_Init(uint32_t delay,uint8_t ExEnOrDi,uint8_t msorus);



#endif /* SYSTICK_H_ */
