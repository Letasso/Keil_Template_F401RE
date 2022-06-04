
#include <stdint.h>
#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

void InitGPIO(void);
void Init_Peripheral_Clocks (void);
void InitSysTick_ms(const uint8_t inTimer);
void rcc_HSI_config (void);
unsigned int iuReset_StatusFlag;
int main(void)
{
  
  Init_Peripheral_Clocks();
  InitGPIO();
  
  rcc_HSI_config();
 
  InitSysTick_ms(1);

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

  
  
  while(1)
  {   
  }
}


void InitGPIO(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;		//Variable GPIO für GPIO Struktur definieren
  
  GPIO_InitStructure.GPIO_Pin =	 GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  
  GPIO_InitStructure.GPIO_Pin =	 GPIO_Pin_13;

	GPIO_InitStructure.GPIO_Mode &= ~(0xFFFFFFFF);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  
  GPIO_InitStructure.GPIO_PuPd &= ~(0xFFFFFFFF); 
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
  
}

void Init_Peripheral_Clocks (void)
{
	// Advanced High-Performance Bus 1
  RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_GPIOAEN,ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_GPIOCEN,ENABLE);
  
  // Advanced Performance Bus 1
  RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM2EN,ENABLE);
}

void rcc_HSI_config (void)
{
  //
  // Gonfiguration parameters --> STM32F401RE Clock Tree
  // 
  //HSI = 16MHz
  //PLL_M = 16
  //PLL_N = 336
  //PLL_P = 4
  //PLL_Q = 7 !!!
  //AHB_Prescaler = 1
  //Cortex Prescaler = 1
  // --> 84MHz Systemclock
  //APB1 Prescaler = 2 --> 42, 84 MHz
  //APB2 Prescaler = 1 --> 84, 84 MHz
  
  //PLL Configuration
  //PLL_M = 16
  if(RCC->CR & RCC_CR_PLLON)
  {
    if (RCC->CFGR & (RCC_CFGR_SW))
    {
      RCC->CFGR &=~(RCC_CFGR_SW);      
    }
    RCC->CR &=~RCC_CR_PLLON;
    RCC->CR &=~RCC_CR_HSEON;
  }
  RCC->PLLCFGR &=~(0x3F);
  RCC->PLLCFGR |= RCC_PLLCFGR_PLLM_4;
  
  RCC->PLLCFGR &=~(RCC_PLLCFGR_PLLN);
  RCC->PLLCFGR |= RCC_PLLCFGR_PLLN_4|RCC_PLLCFGR_PLLN_6|RCC_PLLCFGR_PLLN_8;
  
  RCC->PLLCFGR &=~(RCC_PLLCFGR_PLLP);
  RCC->PLLCFGR |= RCC_PLLCFGR_PLLP_0;
  
  RCC->PLLCFGR &=~(RCC_PLLCFGR_PLLQ);
  RCC->PLLCFGR |= RCC_PLLCFGR_PLLQ_2|RCC_PLLCFGR_PLLQ_1|RCC_PLLCFGR_PLLQ_0;
  
  /*HSI oschillator*/
  //Enable HSI Oscillator
  //RCC_HSICmd(ENABLE);
//  RCC->CR |= (RCC_CR_HSION);
//  //Wait for it to stabilize
//  while((RCC->CR & RCC_CR_HSIRDY) == 0);

  RCC->PLLCFGR &=~ (1U<<22);

  //RCC_PLLConfig(RCC_PLLSource_HSI,0x10,0x150,0x4,0x7);
  //Enable PLL
  RCC->CR |= RCC_CR_PLLON;
  //Wait for PLL ready
  while((RCC->CR & RCC_CR_PLLRDY) == 0);
  
  //Flash prefetch and wait state
  //3WS(Wait State)
  FLASH->ACR &=~(FLASH_ACR_LATENCY);//Clear latency fields
  FLASH->ACR |= FLASH_ACR_LATENCY_3WS;
  //Enable Prefetch buffer
  FLASH->ACR |= FLASH_ACR_PRFTEN;
  
  //Select PLL and main system clock
  RCC->CFGR &=~(RCC_CFGR_SW);
  
  //Wait for PLL to be the activ clk source
//  while((RCC->CFGR & RCC_CFGR_SWS_1) == 0);
  
  //Peripherals clock Setup
  //AHB Prescaler
  RCC->CFGR &=~ RCC_CFGR_HPRE;//Clear all the bit field
  //APB1 prescaler
  RCC->CFGR &=~ RCC_CFGR_PPRE1;
  RCC->CFGR |= RCC_CFGR_PPRE1_2;
  //APB2 prescaler
  RCC->CFGR &=~ RCC_CFGR_PPRE2;
}

void InitSysTick_ms(const uint8_t inTimer)
{
  SysTick_Config(SystemCoreClock / 1000 * inTimer); //Configure Systick to generate inTimer ms
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);  //Systick clock = AHB Clock
}

void SysTick_Handler(void)
{
  static uint16_t sTicksISR5;						//5ms Timer
	static uint16_t sTicksISR10;						//10ms Timer    
	static uint16_t sTicksISR50;						//50ms Timer    
	static uint16_t sTicksISR100;						//100ms Timer    
	static uint16_t sTicksISR200;						//200ms Timer    
	static uint16_t sTicksISR500;						//200ms Timer 
  
   //================ 5ms Timer =====================================================================
	if (++sTicksISR5 >= 5)		//5 x ISR
	{                                              
		sTicksISR5 = 0;
//		cuTimeEvent_5ms = true;
	}
	

 //================ 10ms Timer ====================================================================
	if (++sTicksISR10 >= 10)		//10 x ISR
	{                                              
		sTicksISR10 = 0;
//		cuTimeEvent_10ms = true;
	}


 //================ 50ms Timer ====================================================================
	if (++sTicksISR50 >= 50)		//50 x ISR
	{                                              
		sTicksISR50 = 0;
//		cuTimeEvent_50ms = true;
	}


 //================ 100ms Timer ===================================================================
	if (++sTicksISR100 >= 100)	//100 x ISR
	{                                              
		sTicksISR100 = 0;
//		cuTimeEvent_100ms = true;
	}
	
	
 //================ 200ms Timer ===================================================================
	if (++sTicksISR200 >= 200)	//200 x ISR
	{                                              
		sTicksISR200 = 0;
//		cuTimeEvent_200ms = true;
	}

	
 //================ 500ms Timer ===================================================================
	if (++sTicksISR500 >= 500)	//500 x ISR
	{                                              
		sTicksISR500 = 0;
		//cuTimeEvent_500ms = true;
    GPIO_ToggleBits(GPIOA,GPIO_Pin_5);
	}
}


