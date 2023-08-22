/*
 * stm32_startup.c
 *
 *  Created on: 23 Jul 2023
 *      Author: Malefetsane Lenka
 */
#include <stdint.h>
#define SRAM_START            0x20000000U
#define SRAM_SIZE             (4*1024)   //4KB
#define SRAM_END              ((SRAM_START)+(SRAM_SIZE))

#define STACK_START           SRAM_END 

extern uint32_t _etext;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;
extern uint32_t _la_data;

int main(void);
void __libc_init_array();

void Reset_Handler(void);
void NMI_Handler(void) __attribute__((weak,alias ("Default_Handler")));
void HardFault_Handler(void) __attribute__((weak,alias ("Default_Handler")));
void SVC_Handler(void) __attribute__((weak,alias ("Default_Handler")));
void PendSV_Handler(void) __attribute__((weak,alias ("Default_Handler")));
void SysTick_Handler(void) __attribute__((weak,alias ("Default_Handler")));
void WWDG_IRQHandler(void) __attribute__((weak,alias ("Default_Handler")));
void PVD_VDDIO2_IRQHandler(void) __attribute__((weak,alias ("Default_Handler")));
void RTC_IRQHandler(void) __attribute__((weak,alias ("Default_Handler")));
void FLASH_IRQHandler(void) __attribute__((weak,alias ("Default_Handler")));
void RCC_CRS_IRQHandler(void) __attribute__((weak,alias ("Default_Handler")));
void EXTI0_1_IRQHandler(void) __attribute__((weak,alias ("Default_Handler")));
void EXTI2_3_IRQHandler(void) __attribute__((weak,alias ("Default_Handler")));
void EXTI4_15_IRQHandler(void) __attribute__((weak,alias ("Default_Handler")));
void TSC_IRQHandler(void) __attribute__((weak,alias ("Default_Handler")));
void DMA_CH1_IRQHandler(void) __attribute__((weak,alias ("Default_Handler")));
void DMA_CH2_3_DMA2_CH1_2_IRQHandler(void) __attribute__((weak,alias ("Default_Handler")));
void DMA_CH4_5_6_7_DMA2_CH3_4_5_IRQHandler(void) __attribute__((weak,alias ("Default_Handler")));
void ADC_COMP_IRQHandler(void) __attribute__((weak,alias ("Default_Handler")));
void TIM1_BRK_COMP_TRG_COM_IRQHandler(void) __attribute__((weak,alias ("Default_Handler")));
void TIM1_CC_IRQHandler(void) __attribute__((weak,alias ("Default_Handler")));
void TIM2_IRQHandler(void) __attribute__((weak,alias ("Default_Handler")));
void TIM3_IRQHandler(void) __attribute__((weak,alias ("Default_Handler")));
void TIM6_DAC_IRQHandler(void) __attribute__((weak,alias ("Default_Handler")));
void TIM7_IRQHandler(void) __attribute__((weak,alias ("Default_Handler")));
void TIM14_IRQHandler(void) __attribute__((weak,alias ("Default_Handler")));
void TIM15_IRQHandler(void) __attribute__((weak,alias ("Default_Handler")));
void TIM16_IRQHandler(void) __attribute__((weak,alias ("Default_Handler")));
void TIM17_IRQHandler(void) __attribute__((weak,alias ("Default_Handler")));
void I2C1_IRQHandler(void) __attribute__((weak,alias ("Default_Handler")));
void I2C2_IRQHandler(void) __attribute__((weak,alias ("Default_Handler")));
void SPI1_IRQHandler(void) __attribute__((weak,alias ("Default_Handler")));
void SPI2_IRQHandler(void) __attribute__((weak,alias ("Default_Handler")));
void USART1_IRQHandler(void) __attribute__((weak,alias ("Default_Handler")));
void USART2_IRQHandler(void) __attribute__((weak,alias ("Default_Handler")));
void USART3_4_5_6_7_8_IRQHandler(void) __attribute__((weak,alias ("Default_Handler")));
void CEC_CAN_IRQHandler(void) __attribute__((weak,alias ("Default_Handler")));
void USB_IRQHandler(void) __attribute__((weak,alias ("Default_Handler")));
uint32_t  vectors[]  __attribute__((section (".isr_vector")))= {
    (uint32_t) STACK_START,
    (uint32_t) Reset_Handler,
    (uint32_t) NMI_Handler,
    (uint32_t) HardFault_Handler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    (uint32_t) SVC_Handler,
    0,
    0,
    (uint32_t) PendSV_Handler,
    (uint32_t) SysTick_Handler,
    (uint32_t) WWDG_IRQHandler,
    (uint32_t) PVD_VDDIO2_IRQHandler,
    (uint32_t) RTC_IRQHandler,
    (uint32_t) FLASH_IRQHandler,
    (uint32_t) RCC_CRS_IRQHandler,
    (uint32_t) EXTI0_1_IRQHandler,
    (uint32_t) EXTI2_3_IRQHandler,
    (uint32_t) EXTI4_15_IRQHandler,
    (uint32_t) TSC_IRQHandler,
    (uint32_t) DMA_CH1_IRQHandler,
    (uint32_t) DMA_CH2_3_DMA2_CH1_2_IRQHandler,
    (uint32_t) DMA_CH4_5_6_7_DMA2_CH3_4_5_IRQHandler,
    (uint32_t) ADC_COMP_IRQHandler,
    (uint32_t) TIM1_BRK_COMP_TRG_COM_IRQHandler,
    (uint32_t) TIM1_CC_IRQHandler,
    (uint32_t) TIM2_IRQHandler, 
    (uint32_t) TIM3_IRQHandler, 
    (uint32_t) TIM6_DAC_IRQHandler, 
    (uint32_t) TIM7_IRQHandler, 
    (uint32_t) TIM14_IRQHandler, 
    (uint32_t) TIM15_IRQHandler, 
    (uint32_t) TIM16_IRQHandler, 
    (uint32_t) TIM17_IRQHandler, 
    (uint32_t) I2C1_IRQHandler,
    (uint32_t) I2C2_IRQHandler,
    (uint32_t) SPI1_IRQHandler,
    (uint32_t) SPI2_IRQHandler,
    (uint32_t) USART1_IRQHandler,
    (uint32_t) USART2_IRQHandler,
    (uint32_t) USART3_4_5_6_7_8_IRQHandler,
    (uint32_t) CEC_CAN_IRQHandler,
    (uint32_t) USB_IRQHandler,
 
};


void Reset_Handler(void){
    //Copy .data to SRAM
    //&_edata returns memory address; since _edata is variable
    uint32_t size = (uint32_t)&_edata - (uint32_t)&_sdata;
    uint8_t *pDst = (uint8_t*) &_sdata;  //Flash
    uint8_t *pSrc =(uint8_t*) &_la_data;

    for (uint32_t i=0; i<size; i++){
        *pDst++ = *pSrc++;

    }
    // Init .bss to zero in SRAM
    size = (uint32_t)&_ebss - (uint32_t)&_sbss;
    pDst = (uint8_t*) &_sbss;
        for (uint32_t i=0; i<size; i++){
        *pDst++ = 0;

    }
    //Call c standard lib
    __libc_init_array();

    //Call main
    main();
    
}

void Default_Handler(void){

}