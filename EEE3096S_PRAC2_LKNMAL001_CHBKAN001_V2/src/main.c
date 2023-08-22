#include "stm32f051xx.h"
#include "core.h"

#define RCC_CR_HSERDY        (0x1U<<17)
#define RCC_CFGR_SWS_HSE     (0x01<<2)
#define MICRO_DELAY           8

// Definitions for SPI usage
#define MEM_SIZE 8192 // bytes
#define WREN 0b00000110 // enable writing
#define WRDI 0b00000100 // disable writing
#define RDSR 0b00000101 // read status register
#define WRSR 0b00000001 // write status register
#define READ 0b00000011
#define WRITE 0b00000010



void GPIO_Init(void);
void RCC_Config(void);
static void init_spi(void);
static void TIM16_Enable(void);
static void write_to_address(uint16_t address, uint8_t data);
static uint8_t read_from_address(uint16_t address);
void delay(uint32_t delay_value);

static uint8_t patterns[] = {0xAA,0x55,0xCC,0x33,0xF0,0x0F};
static uint16_t address = 0x0000;
/*@arr_value*/
uint16_t arr_value = 999;

int main(void){
    RCC_Config();
    GPIO_Init();
    init_spi();
    for (int i=0; i<6; ++i){
		write_to_address(address, patterns[i]);
		address++;

	}
	address =0;
	TIM16_Enable();

	while(1){
		__asm volatile("NOP");
    }

	

}

/*
 * @Brief: Enables selects External Crystal Oscillator as a system clock for stable Clock
 *         Initialises clock for GPIOB, GPIOA, SPI2 and TIM16
 * @Param: None
 * @Retval: None
 *
 */
void RCC_Config(void){
    RCC->CR |= (1U<<16);   //

	while(!(RCC->CR & RCC_CR_HSERDY));   //Wait until HSE is READY

	RCC->CFGR |= 0x01;      //Switch System Clock to HSE

	while(!(RCC->CFGR & RCC_CFGR_SWS_HSE));   //Wait

    RCC->AHBENR |= (RCC_AHBENR_GPIOAEN|RCC_AHBENR_GPIOBEN);  //Enable clock for GPIOA and GPIOB

    RCC->APB1ENR |= RCC_APB1ENR_APB1SPI2EN;   //Enable clock for SPI2
    RCC-> APB2ENR |= RCC_APB2ENR_APB2TIM16EN; //Enable clock for TIM16

    RCC->APB2ENR |= 0x01;  //Enable clock for SYSCFG

}
/*
 *@Brief: Initialises PB0-PB7, PB12 in output MOde, PB13-PB15 Alternate Function for SPI2
 *        PA0 in falling interrupt Trigger
 *@Param: None
 *@Retval: None
 */
void GPIO_Init(void){
    GPIOB->MODER |=0x5555;   //PB0-PB7 output (D0-D7) 
    GPIOB->MODER |=(1U<<24);  //PB12 output !SS pin

    GPIOB->MODER |=(0x2A<<26); //P13-PB15 AF Mode, SCK,MOSI,MISO

    GPIOB->ODR |= (1<<12);     //Set CS high

    GPIOA->MODER &= ~(0x11);        //PA0 output
    GPIOA->PUPDR |= 0x01;          //Enable Pullup for PA0
    

    SYSCFG->EXTICR[0] &= ~0xFFFF;   //EXTI0 for GPIOA
    EXTI->IMR  |= 0x01;             //Unmask Interrupt line0
    EXTI->RTSR &= ~0x01;            //Disable Rising Trigger Interrupt
    EXTI->FTSR |= 0x01;             //Enable Falling Trigger Interrupt for PA0

    NVIC->ISER |= (1U<<EXTI0_1_IRQn);  //Enable Interrupt for EXTI0_1

}

/*
 *@Brief: Generates delay in microseconds using SysTick timer
 *@Param: delay_value, number of microseconds for delay
 *@Retval: None
 *@Note:   This is valid for System clock running at 8MHz
 */
void delay(uint32_t delay_value){
    SysTick->RVR = (MICRO_DELAY*delay_value-1)&0xFFFFFF;  //set reload value
    SysTick->CVR  = 0 ; //Clear current value
    SysTick->CSR |= 1<<2;  //Select Processor clock for SysTick
    SysTick->CSR |= 0x1;   //Enable counter;

    while(!(SysTick->CSR &(1U<<16)));  //Wait until counter flag is set
    SysTick->CSR &= ~0x1;   //Disable counter;
    SysTick->CSR &= ~(1<<2);  //Select Processor clock for SysTick
    SysTick->RVR = 0;
    SysTick->CVR = 0;

}
/*
 *@Brief: Initialises TIM16 counter in interrupt mode for delay value provided by @arr_value;
 *@Param: None
 *@Retval: None
 */
static void TIM16_Enable(void){
        TIM16->PSC = 0x1F3F;   //Setting TIM16 freq to 1kHz
        TIM16->ARR = arr_value;    //1s delay
        TIM16->DIER|= TIM_DIER_UIE;  //Update Interrupt flag set
        NVIC->ISER |= (1U<<TIM16_IRQn);  //Enable Interrupt for TIM16
        TIM16->CR1 |= TIM_CR1_CEN;    //Enable Counter for TIM16
}

/*
 *@Brief: Initialiases SPI2 in Master Mode @500kHz, using software slave management, using SPI Mode 00
 *@Param:  None
 *@Retval: None
 *@Note:   SPI speed is 500kHz since APB clock is 8MHz, if APB value is altered, set bit[5:3] correctly in CR1
 */
static void init_spi(void)
{
    SPI2->CR1 |= (0x3U<<3); 					           //Set Baud to fpclk / 16
    SPI2->CR1 |= SPI_CR1_MSTR; 							// Set to master mode
    SPI2->CR1 |= SPI_CR1_SSM;                             // Software slave management
    SPI2->CR1 |= SPI_CR1_SSI;                             //Slave internal select enable
    SPI2->CR2 |= SPI_CR2_FRXTH; 	                        // RXNE event generated after 8 bits recieved
	SPI2->CR2 |= (7<<8);                                  //Data Width 8 bits
    SPI2->CR1 |= SPI_CR1_SPE; 							// Enable the SPI peripheral
}
/*
 *@Brief: Transmit (using blocking mode)data to specified address to EEPROM 25LC640A 
 *@Param: address, address of EEPROM to store at
 *@Param: data, 8bits data to store at specified address
 *@Retval: None
 */ 
static void write_to_address(uint16_t address, uint8_t data){
    uint8_t dummy; // Junk from the DR

	// Set the Write Enable latch
	GPIOB->ODR &= ~(1U<<12);//Pull CS Low
    delay(1);          
	*((uint8_t*)(&SPI2->DR)) = WREN;
	while ((SPI2->SR & SPI_SR_RXNE) == 0); // Hang while RX is empty
	dummy = SPI2->DR;
	GPIOB->ODR |= (1<<12);   //Pull CS high
    delay(5000);  //wait for 5ms


	// Send write instruction
	GPIOB->ODR &= ~(1U<<12);//Pull CS Low
    delay(1);
	*((uint8_t*)(&SPI2->DR)) = WRITE;
	while ((SPI2->SR & SPI_SR_RXNE) == 0); 		// Hang while RX is empty
	dummy = SPI2->DR;

	// Send 16-bit address
	*((uint8_t*)(&SPI2->DR)) = (address >> 8); 	// Address MSB
	while ((SPI2->SR & SPI_SR_RXNE) == 0); 		// Hang while RX is empty
	dummy = SPI2->DR;
	*((uint8_t*)(&SPI2->DR)) = (address); 		// Address LSB
	while ((SPI2->SR & SPI_SR_RXNE) == 0); 		// Hang while RX is empty
	dummy = SPI2->DR;

	// Send the data
	*((uint8_t*)(&SPI2->DR)) = data;
	while ((SPI2->SR & SPI_SR_RXNE) == 0); // Hang while RX is empty
	dummy = SPI2->DR;
	GPIOB->ODR |= (1<<12);   //Pull CS high
	 delay(50000);  //wait for 50ms
}
/*
 *@Brief: Read 8 bit data as EEEPROM Specified address
 *@Param: address, address of EEPROM to read from
 *@Retval: data at a specified address
 */
static uint8_t read_from_address(uint16_t address){
    uint8_t dummy; // Junk from the DR

	// Send the read instruction
	GPIOB->ODR &=~(0x1<<12);   //Pull  CS low
    delay(1);
	*((uint8_t*)(&SPI2->DR)) = READ;
	while ((SPI2->SR & SPI_SR_RXNE) == 0); 		// Hang while RX is empty
	dummy = SPI2->DR;

	// Send 16-bit address
	*((uint8_t*)(&SPI2->DR)) = (address >> 8); 	// Address MSB
	while ((SPI2->SR & SPI_SR_RXNE) == 0);		// Hang while RX is empty
	dummy = SPI2->DR;
	*((uint8_t*)(&SPI2->DR)) = (address); 		// Address LSB
	while ((SPI2->SR & SPI_SR_RXNE) == 0); 		// Hang while RX is empty
	dummy = SPI2->DR;

	// Clock in the data
	*((uint8_t*)(&SPI2->DR)) = 0x42; 			    // Clock out some junk data
	while ((SPI2->SR & SPI_SR_RXNE) == 0); 		// Hang while RX is empty
	dummy = SPI2->DR;
	GPIOB->ODR |= (1<<12);                       //Pull CS high
	delay(5000);  //5ms delay

	return dummy;								              // Return read data
}

/*
 *@Brief: Switch TIM16 delay for 1s->0.5s or vice versa
 *@Param: None
 *@Retval: None
 */
void EXTI0_1_IRQHandler(void){
	EXTI->PR |= 1;            //Clear the Pending bit as Interrupt is being serviced
	RCC->APB2RSTR |= 1<<17;   //Reset TIM16 Registers before modifying ARR
	RCC->APB2RSTR &= ~1<<17;
	/*Change ARR value*/
	if(arr_value == 999){
		arr_value= 499;
	}
	else{
		arr_value= 999;
	}
	TIM16_Enable();    //Re-enable TIM16


}
/*
 *@Brief: Displays read from EEPROM stored at address 0x0000 to 0x0005 on PB0-PB7
 *        If data is not correct: 0x01 set to PB0-PB7
 *@Param: None
 *@Retval: None
 *@Note: Global interrupts are disabled so that higher priority interrupts do not preempt Interrupt handle in this subroutine(Except for exceptions)
 */
void TIM16_IRQHandler(void){
	__asm volatile("CPSID I");  //disable all global interrupts
	if(TIM16->SR & TIM_SR_UIF){
		TIM16->SR &= 0xFFFE;  //Clear Update Interrupt Flag
		uint8_t data = read_from_address(address);
		if (data!=patterns[address]){
			GPIOB->ODR = 0x0001;  //Wrong pattern
		}

		else{

			GPIOB->ODR = data;  //Correct bit pattern
		}
	}
	address++;  //increment the address
	address%=6;  //Values are store at 0x0000 to 0x0005
	__asm volatile("CPSIE I");   //enable all global interrupts
}