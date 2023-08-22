/*
 * stm32f051xx.h
 *
 *  Created on: Jul 12, 2023
 *      Author: Malefetsane Lenka
 */

#ifndef STM32F051XX_H_
#define STM32F051XX_H_
#include <stdint.h>

/*
 * APB and AHB Base Addresses
 */
#define AHB1_BASE           0x40020000UL
#define AHB2_BASE           0x48000000UL
#define APB1_BASE           0x40000000UL
#define APB2_BASE           0x40010000UL

/*
 * Peripherals Hangin on AHB2 Bus
 *
 */

#define GPIOA_BASE         (AHB2_BASE)                    //GPIOA Base Address
#define GPIOB_BASE         ((AHB2_BASE)+(0x0400))         //GPIOB Base Address
#define GPIOC_BASE         ((AHB2_BASE)+(0x0800))         //GPIOC Base Address
#define GPIOD_BASE         ((AHB2_BASE)+(0x0C00))         //GPIOD Base Address
#define GPIOE_BASE         ((AHB2_BASE)+(0x1000))         //GPIOE Base Address
#define GPIOF_BASE         ((AHB2_BASE)+(0x1400))         //GPIOF Base Address

/*
 * Peripherals Hanging on AHB1 Bus
 *
 */

#define RCC_BASE              ((AHB1_BASE)+ (0x1000))     //RCC Base Address

/*
 * Peripherals Hanging on APB2 Bus
 */

#define SYSCFG_BASE          (APB2_BASE)                 //SYSCFG Base Address
#define EXTI_BASE           ((APB2_BASE)+(0x0400))      //EXTI Base Address
#define ADC_BASE            ((APB2_BASE)+(0x2400))      //ADC Base Address
#define TIM16_BASE          ((APB2_BASE)+(0x4400))
/*
 * Peripherals on APB1 bus
 */
#define SPI2_BASE           ((APB1_BASE)+0x3800)

/*
 * IRQ Numbers
 */

typedef enum{
	WWDG_IRQn,
	PVD_VDDIO2_IRQn,
	RTC_IRQn,
	FLASH_IRQn,
	RCC_CRS_IRQn,
	EXTI0_1_IRQn,
	EXTI2_3_IRQn,
	EXTI4_15_IRQn,
	TSC_IRQn,
	DMA_CH1_IRQn,
	DMA_CH2_3_DMA2_CH1_2_IRQn,
	DMA_CH4_5_6_7_DMA2_CH3_4_5_IRQn,
	ADC_COMP_IRQn,
	TIM1_BRK_UP_TRG_COM_IRQn,
	TIM1_CC_IRQn,
	TIM2_IRQn,
	TIM3_IRQn,
	TIM6_DAC_IRQn,
	TIM7_IRQn,
	TIM14_IRQn,
	TIM15_IRQn,
	TIM16_IRQn,
	TIM17_IRQn,
	I2C1_IRQn,
	I2C2_IRQn,
	SPI1_IRQn,
	SPI2_IRQn,
	USART1_IRQn,
	USART2_IRQn,
	USART3_4_5_6_7_8_IRQn,
	CEC_CAN_IRQn,
	USB_IRQn,
}IRQn_t;

/*****************************************************************
 * RCC Registers definition Structure
 *****************************************************************/

typedef struct {
	volatile uint32_t CR;                                /*<!Clock control register                             Address Offset: 0x00>*/
	volatile uint32_t CFGR;                              /*<!Clock configuration register                       Address Offset: 0x04>*/
	volatile uint32_t CIR;                               /*<!Clock interrupt register                           Address Offset: 0x08>*/
	volatile uint32_t APB2RSTR;                          /*<!APB peripheral reset register 2                    Address Offset: 0x0C>*/
	volatile uint32_t APB1RSTR;                          /*<!APB peripheral reset register 1                    Address Offset: 0x10>*/
	volatile uint32_t AHBENR;                            /*<!AHB peripheral clock enable register               Address Offset: 0x14>*/
	volatile uint32_t APB2ENR;                           /*<!APB peripheral clock enable register 2             Address Offset: 0x18>*/
	volatile uint32_t APB1ENR;                           /*<!APB peripheral clock enable register 1             Address Offset: 0x1C>*/
	volatile uint32_t BDCR;                              /*<!RTC domain control register                        Address Offset: 0x20>*/
	volatile uint32_t CSR;                               /*<!Control/status register                            Address Offset: 0x24>*/
	volatile uint32_t AHBRSTR;                           /*<!AHB peripheral reset register                      Address Offset: 0x28>*/
	volatile uint32_t CFGR2;                             /*<!Clock configuration register 2                     Address Offset: 0x2C>*/
	volatile uint32_t CFGR3;                             /*<!Clock configuration register 3                     Address Offset: 0x30>*/
	volatile uint32_t CR2;                               /*<!Clock control register 2                           Address Offset: 0x34>*/
}RCC_RegDef_t;



/************************************************************************
 * GPIO Registers definition structure
 ************************************************************************/

typedef struct{
	volatile uint32_t MODER;                             /*GPIO port mode register                                 Address Offset:0x00*/
	volatile uint32_t OTYPER;                            /*GPIO port output type register                          Address Offset:0x04*/
	volatile uint32_t OSPEEDR;                           /*GPIO port output speed register                         Address Offset:0x08*/
	volatile uint32_t PUPDR;                             /*GPIO port pull-up/pull-down register                    Address Offset:0x0C*/
	volatile uint32_t IDR;                               /*GPIO port input data register                           Address Offset:0x10*/
	volatile uint32_t ODR;                               /*GPIO port input output register                         Address Offset:0x14*/
	volatile uint32_t BSRR;                              /*GPIO port bit set/reset register                        Address Offset:0x18*/
	volatile uint32_t LCKR;                              /*GPIO port configuration lock register                   Address Offset:0x1C*/
	volatile uint32_t AFR[2];                            /*GPIO alternate function AFR[0]=AFRL,AFR[1]=AFRH         Address Offset:0x20,0x24*/
	volatile uint32_t BRR;                               /*GPIO port bit reset register                            Address Offset:0x28*/

}GPIO_RegDef_t;


/*
 * SYSCFG registers definition structure
 */

typedef struct{
	volatile uint32_t CFGR1;
	uint32_t RESERVED0;
	volatile uint32_t EXTICR[4];
	volatile uint32_t CFGR2;
	uint32_t RESERVED1[25];
	volatile uint32_t ITLINE[31];

}SYSCFG_RegDef_t;


/*
 * EXTI registers definition structure
 */
typedef struct {
	volatile uint32_t IMR;
	volatile uint32_t EMR;
    volatile uint32_t RTSR;
    volatile uint32_t FTSR;
    volatile uint32_t SWIER;
    volatile uint32_t PR;


}EXTI_RegDef_t;

/*
 * ADC Registers Definition Structure
 */

typedef struct {
	volatile uint32_t ISR;
	volatile uint32_t IER;
	volatile uint32_t CR;
	volatile uint32_t CFGR[2];
	volatile uint32_t SMPR;
	uint32_t RESERVED0[2];
	volatile uint32_t TR;
	uint32_t RESERVED1;
	volatile uint32_t CHSELR;
	uint32_t RESERVED2[5];
	volatile uint32_t DR;
	uint32_t RESERVED3[177];
	volatile uint32_t CCR;



}ADC_RegDef_t;
/*
 * SPI Structure
 */
typedef struct{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;




}SPI_RegDef_t;

/*
 * TIM16 and TIM17 Registers definition structure
 */
typedef struct{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	uint32_t RESERVED0;
	volatile uint32_t DIER;
	volatile uint32_t SR;
	volatile uint32_t EGR;
	volatile uint32_t CCMR1;
	uint32_t RESERVED1;
	volatile uint32_t CCER;
	volatile uint32_t CNT;
	volatile uint32_t PSC;
	volatile uint32_t ARR;
	volatile uint32_t RCR;
	volatile uint32_t CCR1;
	uint32_t RESERVED2[3];
	volatile uint32_t BDTR;
	volatile uint32_t DCR;
	volatile uint32_t DMAR;



}TIM16_17_RegDef_t;
/*
 * TIMx Macro
 */
#define TIM16                    ((TIM16_17_RegDef_t*)TIM16_BASE)
/*
 * SPIx Macro
 */
#define SPI2                     ((SPI_RegDef_t*)SPI2_BASE)
/*
 * GPIOx Macros
 */

#define GPIOA                     ((GPIO_RegDef_t*)GPIOA_BASE)
#define GPIOB                     ((GPIO_RegDef_t*)GPIOB_BASE)
#define GPIOC                     ((GPIO_RegDef_t*)GPIOC_BASE)
#define GPIOD                     ((GPIO_RegDef_t*)GPIOD_BASE)
#define GPIOE                     ((GPIO_RegDef_t*)GPIOE_BASE)
#define GPIOF                     ((GPIO_RegDef_t*)GPIOF_BASE)


/*
 * RCC Macro
 */

#define RCC                      ((RCC_RegDef_t*)RCC_BASE)

/*
 * EXTI Macro
 */
#define EXTI                     ((EXTI_RegDef_t*)EXTI_BASE)

/*
 * SYSCFG Macro
 */
#define SYSCFG                   ((SYSCFG_RegDef_t*)SYSCFG_BASE)

/*
 * ADC Macro
 */
#define ADC                      ((ADC_RegDef_t*)ADC_BASE)
/*
 * TIMx Clock Enable Macro
 */
#define RCC_APB2ENR_APB2TIM16EN        ((1U<<17))

/*
 * SPIxClock Enable Macro
 */

#define RCC_APB1ENR_APB1SPI2EN            ((1U<<14))


/*
 * GPIOx clock Enable Macros
 */
#define RCC_AHBENR_GPIOAEN             ((0x01<<17))
#define RCC_AHBENR_GPIOBEN             ((0x01<<18))
#define GPIOC_PCLKEN()             ((RCC->AHBENR|=(0x01<<19)))
#define GPIOD_PCLKEN()             ((RCC->AHBENR|=(0x01<<20)))
#define GPIOE_PCLKEN()             ((RCC->AHBENR|=(0x01<<21)))
#define GPIOF_PCLKEN()             ((RCC->AHBENR|=(0x01<<22)))

/*
 * GPIOx clock Disable Macros
 */
#define GPIOA_PCLKDIS()             ((RCC->AHBENR&=~(0x01<<17)))
#define GPIOB_PCLKDIS()             ((RCC->AHBENR&=~(0x01<<18)))
#define GPIOC_PCLKDIS()             ((RCC->AHBENR&=~(0x01<<19)))
#define GPIOD_PCLKDIS()             ((RCC->AHBENR&=~(0x01<<20)))
#define GPIOE_PCLKDIS()             ((RCC->AHBENR&=~(0x01<<21)))
#define GPIOF_PCLKDIS()             ((RCC->AHBENR&=~(0x01<<22)))
/*
 * ADC Peripheral clock enable Macro
 */
#define ADC_PCLKEN()                (RCC->APB2ENR|=(0x01<<9))

/*
 * SYSCFG Peripheral Clock Enable Macro
 */
#define SYSCFG_PCLKEN()             (RCC->APB2ENR|=(0x01<<0))

/*
 * GPIOx Register Reset Macros
 */
#define GPIOA_REG_RESET()  do{RCC->AHBRSTR|=(0x01<<17); RCC->AHBRSTR&=~(0x01<<17);} while (0)
#define GPIOB_REG_RESET()  do{RCC->AHBRSTR|=(0x01<<18); RCC->AHBRSTR&=~(0x01<<18);} while (0)
#define GPIOC_REG_RESET()  do{RCC->AHBRSTR|=(0x01<<19); RCC->AHBRSTR&=~(0x01<<19);} while (0)
#define GPIOD_REG_RESET()  do{RCC->AHBRSTR|=(0x01<<20); RCC->AHBRSTR&=~(0x01<<20);} while (0)
#define GPIOE_REG_RESET()  do{RCC->AHBRSTR|=(0x01<<21); RCC->AHBRSTR&=~(0x01<<21);} while (0)
#define GPIOF_REG_RESET()  do{RCC->AHBRSTR|=(0x01<<22); RCC->AHBRSTR&=~(0x01<<22);} while (0)

/*
 * GPIOx Pin Numbers
 */
#define GPIO_PIN0            0x00
#define GPIO_PIN1            0x01
#define GPIO_PIN2            0x02
#define GPIO_PIN3            0x03
#define GPIO_PIN4            0x04
#define GPIO_PIN5            0x05
#define GPIO_PIN6            0x06
#define GPIO_PIN7            0x07
#define GPIO_PIN8            0x08
#define GPIO_PIN9            0x09
#define GPIO_PIN10           0x0A
#define GPIO_PIN11           0x0B
#define GPIO_PIN12           0x0C
#define GPIO_PIN13           0x0D
#define GPIO_PIN14           0x0E
#define GPIO_PIN15           0x0F
/*
 * SPIx Registers Macros
 */
#define SPI_SR_RXNE         (1U)
#define SPI_CR1_SSM         (1U<<9)
#define SPI_CR1_SSI         (1U<<8)
#define SPI_CR1_MSTR        (1U<<2)
#define SPI_CR1_SPE         (1U<<6)
#define SPI_CR2_FRXTH       (1U<<12)

/*
 * TIMx Registers Macro
 */
#define TIM_CR1_CEN         1U    //Counter Enable
#define TIM_DIER_UIE        1U    //Update Interrupt Enable
#define TIM_SR_UIF          1U    //Update Inerrupt Flag

/*
 * Enable or Disable Macros
 */
#define ENABLE        0x01
#define DISABLE       0x00
#define PIN_SET       ENABLE
#define PIN_RESET     DISABLE
#endif /* STM32F051XX_H_ */
