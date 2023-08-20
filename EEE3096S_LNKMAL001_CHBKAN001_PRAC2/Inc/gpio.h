/*
 * gpio.h
 *
 *  Created on: 13 Jul 2023
 *      Author: Malefetsane Lenka
 */


#ifndef GPIO_H_
#define GPIO_H_

#include "stm32f051xx.h"

/*
 * GPIO Possible Modes,
 */
typedef enum{
	InputMode,
	OutputMode,
	AltFucntionMode,
	AnalogMode,

}GPIO_MODE_t;

/*
 * GPIOx Pull up pull down Enum
 *
 */

typedef enum{
	NoPullUpOrPullDown,
	PullUp,
	PullDOwn,


}GPIO_PUPD_t;

/*
 * Possible Alternate fucntion numbers
 */
typedef enum{
	AF0,
	AF1,
	AF2,
	AF3,
	AF4,
	AF5,
	AF6,
	AF7,

}GPIO_AltFn_t;

/*
 * GPIO Configuration Structure
 */
typedef struct{
	uint8_t PinNumber;              //see  @GPIOx Pin Numbers
    GPIO_MODE_t PinMode;            //see  @GPIO Possible Modes
	uint8_t OutPutType;
	uint8_t PinSpeed;
	GPIO_PUPD_t PullUpPullDown;     //see  @GPIOx Pull up pull down Enum
	GPIO_AltFn_t AFSEL;             //seee  @Possible Alternate fucntion numbers


}GPIO_Config_t;

/*
 * GPIO Handle structure
 *
 */
typedef struct{
	GPIO_RegDef_t *GPIOx;       //see @ GPIOx Macros
	GPIO_Config_t GPIOx_Config; //see @GPIO Configuration Structure


}GPIO_Handle_t;

/*
 * GPIO OutPut possible configurations
 */
#define PUSHPULL                0x00
#define OPENDRAIN               0x01

/*
 * GPIO OutPutSpeed possible values Macro
 */
#define LOWSPEED                        0x00
#define MEDIUMSPEED                     0x01
#define HIGHSPEED                       0x03

/*
 * Falling, Rising edge Trigger Macros
 */
#define 	FT_INTERRUPT                0x00
#define     RT_INTERRUPT                0x01
#define     FRT_INTERRUPT               0x02

/********************************************************************************
 *GPIO API, full definition of the APIs are found in gpio.c
 ********************************************************************************/

void GPIO_ClockConfig(uint8_t EnOrDi,GPIO_RegDef_t *GPIOx);
void GPIO_Init(GPIO_Handle_t* GPIOx_Handle);
void GPIO_DeInit(GPIO_RegDef_t *GPIOx);
void GPIO_WritePin(GPIO_RegDef_t *GPIOx,uint8_t PinNumber,uint8_t PinState);
void GPIO_WritePort(GPIO_RegDef_t *GPIOx);
void GPIO_TogglePin(GPIO_RegDef_t *GPIOx,uint8_t PinNumber);
uint8_t GPIO_ReadPin(GPIO_RegDef_t *GPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadPort(GPIO_RegDef_t *GPIOx);
void GPIO_IRQEnable(GPIO_Handle_t* GPIOx_Handle,IRQn_t IRQn,uint8_t edge_trigger);
void GPIO_ISRHandler(uint8_t PinNumber);




#endif
