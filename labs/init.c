#include "init.h"
#include "stm32f10x_conf.h"

void sys_init() {

   // Reset the RCC configuration
    RCC_DeInit();

    // Turn on the HSE and wait for it to start up successfully
    RCC_HSEConfig(RCC_HSE_ON);
    while (RCC_WaitForHSEStartUp() != SUCCESS) {}

    // Enable prefetch buffer 
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    // Two latency cycles for flash memory  
    FLASH_SetLatency(FLASH_Latency_2);

    // Get the general APB clock from SYSCLK divided by 1
    RCC_HCLKConfig(RCC_SYSCLK_Div1);

    // Get the APB1 clock from HCLK divided by 1
    RCC_PCLK1Config(RCC_HCLK_Div1);

    // Get the APB2 clock from HCLK divided by 1
    RCC_PCLK2Config(RCC_HCLK_Div1);

    /*
     * Configure the PLL
     * PLLCLK = (HSE / 2) * 9 = 72 MHz 
     */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div2, RCC_PLLMul_9);

    // Enable the PLL and wait for it to turn on
    RCC_PLLCmd(ENABLE);
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {}

    // Make PLLCLK the system clock and wait for that to take effect
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    while (RCC_GetSYSCLKSource() != PLLCLK){}
}


