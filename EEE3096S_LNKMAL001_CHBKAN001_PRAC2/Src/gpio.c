/*
 * gpio.c
 *
 *  Created on: 13 Jul 2023
 *      Author: Malefetsane Lenka
 */

#include "gpio.h"
#include "syscfg.h"
#include  "core.h"


/*
 *@Brief:  Enables or Disables clock for given GPIOx
 *@param:  ENoRDi, see @Enable or Disable Macros in mcu header file
 *@param:  GPIOx, pointer to GPIO registers definition structure
 *         see @GPIOx Macros in mcu header file
 *@retval: None
 *@Note: None
 */
void GPIO_ClockConfig(uint8_t EnOrDi,GPIO_RegDef_t *GPIOx){
	if (EnOrDi==ENABLE){
		/*
		 * See GPIOCK Clock Enable Macros @GPIOx clock Enable Macros
		 */
		if (GPIOx==GPIOA){
			GPIOA_PCLKEN();
		}
		else if (GPIOx==GPIOB){
			GPIOB_PCLKEN();
		}
		else if (GPIOx==GPIOC){
			GPIOC_PCLKEN();
		}
		else if (GPIOx==GPIOD){
			GPIOD_PCLKEN();
		}
		else if (GPIOx==GPIOE){
			GPIOE_PCLKEN();
		}
		else if (GPIOx==GPIOF){
			GPIOF_PCLKEN();
		}

	}
	else {
		/*
		 * See GPIO peripheral clock disable macros @GPIOx clock Disable Macros
		 */
		if (GPIOx==GPIOA){
			GPIOA_PCLKDIS();
		}
		else if (GPIOx==GPIOB){
			GPIOB_PCLKDIS();
		}
		else if (GPIOx==GPIOC){
			GPIOC_PCLKDIS();
		}
		else if (GPIOx==GPIOD){
			GPIOD_PCLKDIS();
		}
		else if (GPIOx==GPIOE){
			GPIOE_PCLKDIS();
		}
		else if (GPIOx==GPIOF){
			GPIOF_PCLKDIS();
		}
	}

}


/*
 * @Brief: Resets all Registers of the GPIO port
 * @param : GPIOx, pointer to GPIO registers definition structure
 *         see @GPIOx Macros in mcu header file
 * @retval: None
 * Note: See @GPIOx Register Reset Macros in mcu header file
 */
void GPIO_DeInit(GPIO_RegDef_t *GPIOx){
	if (GPIOx==GPIOA){
		GPIOA_REG_RESET();
	}
	else if (GPIOx==GPIOB){
		GPIOB_REG_RESET();
	}
	else if (GPIOx==GPIOC){
		GPIOC_REG_RESET();
	}
	else if (GPIOx==GPIOD){
		GPIOD_REG_RESET();
	}
	else if (GPIOx==GPIOE){
		GPIOE_REG_RESET();
	}
	else if (GPIOx==GPIOF){
		GPIOF_REG_RESET();
	}

}



/*
 * @Brief:  Initializes the GPIO port configuration settings
 * @param: GPIOx_Handle:  Pointer to GPIO_Handle_t structure
 * 		   see @GPIO Handle structure
 * @retval: None
 * @Note: User GPIOx_Handle structure must initialize the all
 *        configuration settings not used in use app to default
 *        See MCU GPIO Registers
 */
void GPIO_Init(GPIO_Handle_t* GPIOx_Handle){
	//Reset the GPIOx Registers first
	//GPIO_DeInit(GPIOx_Handle->GPIOx);
	//GetPinNumber
	uint8_t PinNumber = GPIOx_Handle->GPIOx_Config.PinNumber;

	/*
	 * Configure GPIOx Registers
	 */
	//1. GPIO Mode
	GPIOx_Handle->GPIOx->MODER |= (GPIOx_Handle->GPIOx_Config.PinMode<<2*PinNumber);


	if (GPIOx_Handle->GPIOx_Config.PinMode==AltFucntionMode){
		uint8_t AFRx= PinNumber/8;
		uint8_t AFSELy=PinNumber%8;
		GPIOx_Handle->GPIOx->AFR[AFRx] |= (GPIOx_Handle->GPIOx_Config.AFSEL<<4*AFSELy);

	}
	//3. PullUp and PullDown
	GPIOx_Handle->GPIOx->PUPDR |= (GPIOx_Handle->GPIOx_Config.PullUpPullDown<<2*PinNumber);
	//3.OutPutType
	GPIOx_Handle->GPIOx->OTYPER |= (GPIOx_Handle->GPIOx_Config.OutPutType<<PinNumber);

	//3. OutPut Speed
	GPIOx_Handle->GPIOx->OSPEEDR |= (GPIOx_Handle->GPIOx_Config.PinSpeed<<2*PinNumber);
}
/*
 * @Brief: Writes one to given GPIO port pin Number
 * @param: GPIOx, pointer to GPIO registers definition structure
 *         see @GPIOx Macros in mcu header file
 * @param: PinNumber: GPIOx pin Number see @GPIOx Pin Numbers
 */
void GPIO_WritePin(GPIO_RegDef_t *GPIOx,uint8_t PinNumber,uint8_t PinState){
	if (PinState){
		GPIOx->ODR |= (PinState<<PinNumber);

	}
	else{
		GPIOx->ODR &= ~(PIN_SET<<PinNumber);

	}

}


/*
 * @Brief:  ENables External Interrupt Trigger for GPIO
 * @param: GPIOx_Handle:  Pointer to GPIO_Handle_t structure
 * 		   see @GPIO Handle structure
 * @param: IRQn,IRQ_t enum type see @IRQ Numbers in mcu header file
 * @param: edge_trigger, Falling/Rising edge trigger selectiom
 * 		   or Both  see @Falling, Rising edge Trigger Macros
 * @retval: None
 * @Note: None
 */
void GPIO_IRQEnable(GPIO_Handle_t* GPIOx_Handle,IRQn_t IRQn,uint8_t edge_trigger){
	GPIO_Init(GPIOx_Handle);
	EXTI_IT_Config(GPIOx_Handle->GPIOx, GPIOx_Handle->GPIOx_Config.PinNumber);
	EXTI->IMR  |=  (0x1<<GPIOx_Handle->GPIOx_Config.PinNumber);
	if (edge_trigger==FT_INTERRUPT){

		EXTI->FTSR |= (0x1<<GPIOx_Handle->GPIOx_Config.PinNumber);
	}

	else if (edge_trigger==RT_INTERRUPT){
		EXTI->RTSR |= (0x1<<GPIOx_Handle->GPIOx_Config.PinNumber);

	}
	else {
		EXTI->FTSR |= (0x1<<GPIOx_Handle->GPIOx_Config.PinNumber);
		EXTI->RTSR |= (0x1<<GPIOx_Handle->GPIOx_Config.PinNumber);

	}
	NVIC_IRQEnable(IRQn);
}
/*
 * @Brief:  Called by EXTI handler fucntion to clear pending bit
 * @param:  PinNumber: pin Number which interrupt is fired throug see @GPIOx Pin Numbers
 * @retval: None
 * @Note:   None
 */
void GPIO_ISRHandler(uint8_t PinNumber){
	uint32_t mask = 0x01UL<<PinNumber;
	while (!(mask&EXTI->PR));
	EXTI->PR |= (0x1<<PinNumber);   //Clear Pending Bit

}
/*
 * @Brief: Toggles given GPIO port pin Number
 * @param: GPIOx, pointer to GPIO registers definition structure
 *         see @GPIOx Macros in mcu header file
 * @param: PinNumber: GPIOx pin Number see @GPIOx Pin Numbers
 * @retval: None
 * @Note:  None
 */
void GPIO_TogglePin(GPIO_RegDef_t *GPIOx,uint8_t PinNumber){
	GPIOx->ODR ^= (0x01<<PinNumber); //Toggles the Pin

}
