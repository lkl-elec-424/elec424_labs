/* 
 *  lab02 - Manual scheduling
 *	ELEC 424
 *	Zihe Huang, Lindsay Langford, Minh Nguyen
 */

#include "init.h"
#include "lab2.h"
#include "stm32f10x_conf.h"

/* Global var declarations */
// Defined in lab2.h, used for getting PID status of all 4 motors
MotorSpeeds motor_speeds; 
TIM_TimeBaseInitStruct timebase_init_struct; // Initial struct for timer modules
TIM_OCInitTypeDef timer_oc_init_struct;      // Initial struct for output compare timers
bool pid_set_flag = false;					// Flag that is set to true when motor
											// PIDs need to be scaled and set, false otherwise

/* Function definitions */
void config_timer(void);
void config_gpio(void);
void config_nvic(void);

/* #defines for constants */
#define PRESCALE			 1800   // Divides the 72MHz SYSCLK so timers run at 40KHz
#define TIM_PERIOD			39999   // Timers count to 40000 - 1 and then reset
#define DELAY_10MS			  400   // Corresponds to 10 ms at frequency of 40KHz
#define DELAY_100MS			 4000   // Corresponds to 100 ms at frequency of 40KHz
#define RED_PERIOD			 9999   // Half of the red led's 0.25Hz cycle
#define GREEN_PERIOD		19999   // Half of the green led's 0.5Hz cycle

// Main function
int main() {

	// Initialize the uC from the outside helper file
	sys_init();

	// Local config functions
	config_gpio();
	config_nvic();
	config_timer();
	 
	return 0;
}

/*
 * config_timer - Configures all of the TIM modules which are used in this lab.
 * 
 * Inputs: None
 * 
 * Outputs: None
 */
void config_timer(void) {

	// Initial values for all the used channels in TIM2 and TIM3
	// They are all offset to avoid interrupting each other in the course of the program
	__IO uint16_t TIM2_CCR1_init = 100;  // Emergency
	__IO uint16_t TIM2_CCR2_init = 300;  // Data
	__IO uint16_t TIM2_CCR3_init = 400;  // Orientation
	__IO uint16_t TIM2_CCR4_init = 900;  // PID

	__IO uint16_t TIM5_CCR1_init = 1100; // Debug 
	__IO uint16_t TIM5_CCR2_init = RED_PERIOD;  // Red LED
	__IO uint16_t TIM5_CCR3_init = GREEN_PERIOD;  // Green LED

	// All of these configs are valid for TIM2 and TIM5
	timebase_init_struct.TIM_Period = TIM_PERIOD;
	timebase_init_struct.TIM_Prescaler = 0;
	timebase_init_struct.TIM_ClockDivision = 0;
	timebase_init_struct.TIM_CounterMode = TIM_CounterMode_Up;

	// Initialize the timebases for TIM2 and TIM5
	TIM_TimeBaseInit(TIM2, &timebase_init_struct);
	TIM_TimeBaseInit(TIM5, &timebase_init_struct);

	// Configure prescalers for TIM2 and TIM5
	TIM_PrescalerConfig(TIM2, PRESCALE, TIM_PSCReloadMode_Immediate);
	TIM_PrescalerConfig(TIM5, PRESCALE, TIM_PSCReloadMode_Immediate);

	// Configure output compare timing mode for channel 1 of TIM2 and TIM5
	timer_oc_init_struct.TIM_OCMode = TIM_OCMode_Timing;
	timer_oc_init_struct.TIM_OutputState = TIMOutputState_Enable;
	timer_oc_init_struct.TIM_Pulse = TIM2_CCR1_init;
	timer_oc_init_struct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM2, &timer_oc_init_struct);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);

	timer_oc_init_struct.TIM_Pulse = TIM5_CCR1_init;
	TIM_OC1Init(TIM5, &timer_oc_init_struct);
	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Disable);

	// Configure output compare timing mode for channel 2 of TIM2 and TIM5
	timer_oc_init_struct.TIM_OCMode = TIM_OCMode_Timing;
	timer_oc_init_struct.TIM_OutputState = TIMOutputState_Enable;
	timer_oc_init_struct.TIM_Pulse = TIM2_CCR2_init;
	timer_oc_init_struct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC2Init(TIM2, &timer_oc_init_struct);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);

	timer_oc_init_struct.TIM_Pulse = TIM5_CCR2_init;
	TIM_OC2Init(TIM5, &timer_oc_init_struct);
	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Disable);

	// Configure output compare timing mode for channel 3 of TIM2 and TIM5
	timer_oc_init_struct.TIM_OCMode = TIM_OCMode_Timing;
	timer_oc_init_struct.TIM_OutputState = TIMOutputState_Enable;
	timer_oc_init_struct.TIM_Pulse = TIM2_CCR3_init;
	timer_oc_init_struct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC3Init(TIM2, &timer_oc_init_struct);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Disable);

	timer_oc_init_struct.TIM_Pulse = TIM5_CCR3_init;
	TIM_OC3Init(TIM5, &timer_oc_init_struct);
	TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Disable);

	// Configure output compare timing mode for channel 4 of TIM2 and TIM5
	timer_oc_init_struct.TIM_OCMode = TIM_OCMode_Timing;
	timer_oc_init_struct.TIM_OutputState = TIMOutputState_Enable;
	timer_oc_init_struct.TIM_Pulse = TIM2_CCR4_init;
	timer_oc_init_struct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC4Init(TIM2, &timer_oc_init_struct);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Disable);

	// Only need 3 channels for timer 5
	timer_oc_init_struct.TIM_OutputState = TIMOutputState_Disable;
	timer_oc_init_struct.TIM_Pulse = TIM5_CCR3_init; // Unchanged because unused 
	TIM_OC4Init(TIM5, &timer_oc_init_struct);
	TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Disable);

	// Enable the timer interrupts 
	TIM_ITConfig(TIM2, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);
	TIM_ITConfig(TIM5, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3, ENABLE);

	// Enable the timer counters
	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM5, ENABLE);
}

/*
 * config_nvic - Configures the interrupt vector handler for TIM2 and TIM5
 *
 * Inputs: None
 * 
 * Outputs: None
 */
void config_nvic(void) {

NVIC_InitTypeDef nvic_init_struct;

// Want interrupts to trigger from TIM2 and TIM5, and TIM2 should be the higher
// priority because it controls timing for the important functions and TIM5 just
// controls the LEDS
// Setting up interrupts for timer 2
nvic_init_struct.NVIC_IRQChannel = TIM2_IRQn;
nvic_init_struct.NVIC_IRQChannelPreemptionPriority = 0;
nvic_init_struct.NVIC_IRQChannelSubPriority = 1;
nvic_init_struct.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&nvic_init_struct);

// Setting up timer 5 interrupts
nvic_init_struct.NVIC_IRQChannel = TIM5_IRQn;
nvic_init_struct.NVIC_IRQChannelPreemptionPriority = 1;
nvic_init_struct.NVIC_IRQChannelSubPriority = 2;
NVIC_Init(&nvic_init_struct);
}

/* 
 * config_gpio - Configures the GPIO pins for the red and green LEDs. They are both
 * 			   push-pull outputs with a max speed to 50MHz.
 * 
 * Inputs: None
 *
 * Outputs: None
 */
void config_gpio(void) {

GPIO_InitTypeDef gpio_init_struct;

// Pins PB4 and PB5 are attached to the LEDs
gpio_init_struct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
gpio_init_struct.GPIO_Mode = GPIO_Mode_Out_PP;
gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOB, &gpio_init_struct);

}
