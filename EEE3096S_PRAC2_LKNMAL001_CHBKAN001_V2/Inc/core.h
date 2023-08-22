/*
 * core.h
 *
 *  Created on: Jul 12, 2023
 *      Author: Malefetsane Lenka
 */

#ifndef CORE_H_
#define CORE_H_
#include <stdio.h>
#include <stdint.h>
#include "stm32f051xx.h"


#define SCB_BASE                                  0xE000ED00UL
#define NVIC_BASE                                 0xE000E100UL
#define SysTIck_BASE                              0xE000E010UL




/*
 * System Control Block Registers Structure
 */

typedef struct{
	volatile uint32_t CPUID;                 /* CPUID base register (CPUID)*/
	volatile uint32_t  ICSR;                 /*Interrupt control and state register (ICSR)*/
	uint32_t RESERVED0;
	volatile uint32_t AIRCR;                 /* Application interrupt and reset control register (AIRCR)*/
	volatile uint32_t  SCR;                  /*System control register (SCR)*/
	volatile uint32_t  CCR;                  /*Configuration and control register (CCR)*/
	volatile uint32_t  SHPR2;                /* System handler priority registers (SHPR2)*/
	volatile uint32_t  SHPR3;                /* System handler priority registers (SHPRx)*/


}SCB_RegDef_t;


/*
 * NVIC Registers Definition Structure
 */
typedef struct{
	volatile uint32_t ISER;                    /*Interrupt set-enable register*/
	uint32_t RESERVED0[31];
	volatile uint32_t ICER;                     /*Interrupt clear-enable register*/
	uint32_t RESERVED1[31];
	volatile uint32_t ISPR;                     /*Interrupt Set-Pending Register*/
	uint32_t RESERVED2[31];
	volatile uint32_t ICPR;                     /*Interrupt clear-Pending Register*/
	uint32_t RESERVED3[95];
	volatile uint32_t IPR[8];                   /*Interrupt Priority Register,IPR0-IPR7*/



}NVIC_RegDef_t;

/*
 * SysTick Registers Definition Structure
 */
typedef struct {
	volatile uint32_t CSR;                  /*SysTick control and status register      Address Offset:0x00 */
	volatile uint32_t RVR;                  /*SysTick reload value register            Address Offset:0x04 */
	volatile uint32_t CVR;                  /* SysTick current value register          Address Offset:0x08 */
	volatile uint32_t CALIB;                /*SysTick calibration value register       Address Offset:0x0C */

}SysTick_RegDef_t;


//SysTick
#define SysTick                         ((SysTick_RegDef_t*)SysTIck_BASE)
//SCB
#define SCB                             ((SCB_RegDef_t*)SCB_BASE)
//NVIC
#define NVIC                            ((NVIC_RegDef_t*)NVIC_BASE)

__attribute__((weak)) void switch_sp_to_psp(void);
#endif /* CORE_H_ */
