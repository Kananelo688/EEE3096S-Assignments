/*
 * systick.c
 *
 *  Created on: Jul 15, 2023
 *      Author: Malefetsane
 */

#include "systick.h"


static void pollflag(void){
	 while (!(SysTick->CSR&SYSTICK_CSR_COUNTFLAG)); //Wait until flag is set
	 SysTick->CSR &= SYSTICK_CSR_EN; //Enable Counter
	 SysTick->CSR &=  SYSTICK_PROCESSOR_CLK;

}

/*
 * @Brief: Initializes Systic Timer for Delays
 * @
 */
void SysTick_Init(uint32_t delay,uint8_t ExEnOrDi,uint8_t msorus){
	uint32_t ReloadVal = 0;

	if (msorus){
		 ReloadVal = (delay*MS_DELAY - 1)&0x00FFFFFF;  //Reload value

	}
	else{
		 ReloadVal = (delay*US_DELAY - 1)&0x00FFFFFF;  //Reload value

	}

	SysTick->RVR = ReloadVal; //Load Reload VALUE
	SysTick->CVR &= ~(0x00FFFFFF);  //Clear Current value
	SysTick->CSR  |= SYSTICK_PROCESSOR_CLK; //Select Processor Clock
    SysTick->CSR &=~EXC_ENABLE; //Clear the Exception enable bit
    SysTick->CSR |= ExEnOrDi; //Exception Enable/Diasble
    SysTick->CSR |= SYSTICK_CSR_EN; //Enable Counter
   if (ExEnOrDi==EXC_DISABLE){
	   //Polling
	   //Delay
	   pollflag();
   }
   //Otherwise Implement Systick Hnadler


}


