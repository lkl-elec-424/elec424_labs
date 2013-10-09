#include "stm32f10x_conf.h"

// This is dependent upon processor clock frequency, which is currently 8 MHz
#define DELAY_CYCLES 400000
int main() {

// Counter variable
int i = 0;

// Configure the GPIO peripheral clock
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

// Initial configuration of PB5
GPIO_InitTypeDef GPIO_InitStruct;
GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;

// Set slowest possible speed because only want to blink LED at 1 Hz.
GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_Init(GPIOB, &GPIO_InitStruct);

// Main loop which runs forever
while(1) {

	// Turn off LED		
	GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_SET);
	for (i = 0; i < DELAY_CYCLES; i++) {}

	// Turn on LED
	GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_RESET);
	for (i = 0; i < DELAY_CYCLES; i++) {}
	
}

// Should not get here
return 0;
}
