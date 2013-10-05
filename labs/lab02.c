/* 
 *  lab02 - Manual scheduling
 *	ELEC 424
 *	Zihe Huang, Lindsay Langford, Minh Nguyen
 */

#include "init.h"
#include "stm32f10x.h"
#include "lab2.h"
#include "stm32f10x_conf.h"
uint16_t CCR1_Val = 60;                 // PWM pulse value
uint32_t SystemCoreClock = 72000000;    // SYSCLK frequency

uint16_t step = 0;
uint16_t motor1_speed;
uint16_t motor2_speed;
uint16_t motor3_speed;
uint16_t motor4_speed;

uint16_t emer_flag = 0;
uint16_t data_flag = 0;
uint16_t orien_flag = 0;
uint16_t pid_flag = 0;
uint16_t log_flag = 0;
/* Global var declarations */
// Defined in lab2.h, used for getting PID status of all 4 motors
MotorSpeeds motor_speeds; 
TIM_TimeBaseInitTypeDef timebase_init_struct; // Initial struct for timer modules
TIM_OCInitTypeDef timer_oc_init_struct;      // Initial struct for output compare timers
MotorSpeeds motor_speeds;
uint16_t pid_set_flag = 0;					// Flag that is set to true when motor
uint16_t capture = 0;       // When an interrupt was triggered
uint16_t green_cnt = 0;     // Counter for the green LED
											// PIDs need to be scaled and set, false otherwise

/* Function definitions */
void config_timer(void);
void config_gpio(void);
void config_nvic(void);
void TIM2_IRQHandler(void);
void TIM1_IRQHandler(void);
//void Delay(__IO uint32_t nTime);
void setMotor1(uint16_t step);
void setMotor2(uint16_t step);
void setMotor3(uint16_t step);
void setMotor4(uint16_t step);

/* #defines for constants */
#define PRESCALE			 1800   // Divides the 72MHz SYSCLK so timers run at 40KHz
#define DELAY_10MS                        400   // 10 ms at frequency of 40KHz
#define DELAY_100MS                      4000   // 100 ms at frequency of 40KHz
#define DELAY_500MS                     20000   // Half a second at 40KHz
#define TIM_PERIOD			39999   // Timers count to 40000 - 1 and then reset
#define RED_START			 9999   // The red LED's starting time
#define GREEN_START			19999   // The green LED's starting time

// Main function
int main() {

	// Initialize the uC from the outside helper file
	sys_init();

	// Local config functions
	config_gpio();
	config_nvic();
	config_timer();
	
	while (1) {

		if (emer_flag == 1) {
			detectEmergency();
			emer_flag = 0;
		} else if (data_flag == 1) {
			refreshSensorData();
			data_flag = 0;
		} else if (orien_flag == 1) {
			calculateOrientation();
			orien_flag = 0;
		} else if (pid_flag == 1) {
			updatePid(&motor_speeds);

			// Get motor speeds from the struct
			motor1_speed = (uint16_t)motor_speeds.m1;
			motor2_speed = (uint16_t)motor_speeds.m2;
			motor3_speed = (uint16_t)motor_speeds.m3;
			motor4_speed = (uint16_t)motor_speeds.m4;

			// Set motors
			setMotor1(CCR1_Val * motor1_speed);
			setMotor2(CCR1_Val * motor2_speed);
			setMotor3(CCR1_Val * motor3_speed);
			setMotor4(CCR1_Val * motor4_speed);
			pid_flag = 0;				
		} else if (log_flag == 1) {
			logDebugInfo();
			log_flag = 0;
		}
	}
 
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

	// Initial values for all the used channels in TIM2 and TIM1
	// They are all offset to avoid interrupting each other in the course of the program
	__IO uint16_t TIM2_CCR1_init = 100;  // Emergency
	__IO uint16_t TIM2_CCR2_init = 300;  // Data
	__IO uint16_t TIM2_CCR3_init = 400;  // Orientation
	__IO uint16_t TIM2_CCR4_init = 900;  // PID
	__IO uint16_t TIM1_CCR1_init = 1100; // Debug 
	__IO uint16_t TIM1_CCR2_init = RED_START;  // Red LED
	__IO uint16_t TIM1_CCR3_init = GREEN_START;  // Green LED

	// Set up peripheral clock for timers
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	// All of these configs are valid for TIM2 and TIM1
	timebase_init_struct.TIM_Period = TIM_PERIOD;
	timebase_init_struct.TIM_Prescaler = 0;
	timebase_init_struct.TIM_ClockDivision = 0;
	timebase_init_struct.TIM_CounterMode = TIM_CounterMode_Up;

	// Initialize the timebases for TIM2 and TIM1
	TIM_TimeBaseInit(TIM2, &timebase_init_struct);
	TIM_TimeBaseInit(TIM1, &timebase_init_struct);

	// Configure prescalers for TIM2 and TIM1
	TIM_PrescalerConfig(TIM2, PRESCALE, TIM_PSCReloadMode_Immediate);
	TIM_PrescalerConfig(TIM1, PRESCALE, TIM_PSCReloadMode_Immediate);

	// Configure output compare timing mode for channel 1 of TIM2 and TIM1
	timer_oc_init_struct.TIM_OCMode = TIM_OCMode_Timing;
	timer_oc_init_struct.TIM_OutputState = TIM_OutputState_Enable;
	timer_oc_init_struct.TIM_Pulse = TIM2_CCR1_init;
	timer_oc_init_struct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM2, &timer_oc_init_struct);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);

	timer_oc_init_struct.TIM_Pulse = TIM1_CCR1_init;
	TIM_OC1Init(TIM1, &timer_oc_init_struct);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);

	// Configure output compare timing mode for channel 2 of TIM2 and TIM1
	timer_oc_init_struct.TIM_OCMode = TIM_OCMode_Timing;
	timer_oc_init_struct.TIM_OutputState = TIM_OutputState_Enable;
	timer_oc_init_struct.TIM_Pulse = TIM2_CCR2_init;
	timer_oc_init_struct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC2Init(TIM2, &timer_oc_init_struct);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);

	timer_oc_init_struct.TIM_Pulse = TIM1_CCR2_init;
	TIM_OC2Init(TIM1, &timer_oc_init_struct);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Disable);

	// Configure output compare timing mode for channel 3 of TIM2 and TIM1
	timer_oc_init_struct.TIM_OCMode = TIM_OCMode_Timing;
	timer_oc_init_struct.TIM_OutputState = TIM_OutputState_Enable;
	timer_oc_init_struct.TIM_Pulse = TIM2_CCR3_init;
	timer_oc_init_struct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC3Init(TIM2, &timer_oc_init_struct);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Disable);

	timer_oc_init_struct.TIM_Pulse = TIM1_CCR3_init;
	TIM_OC3Init(TIM1, &timer_oc_init_struct);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Disable);

	// Configure output compare timing mode for channel 4 of TIM2 and TIM1
	timer_oc_init_struct.TIM_OCMode = TIM_OCMode_Timing;
	timer_oc_init_struct.TIM_OutputState = TIM_OutputState_Enable;
	timer_oc_init_struct.TIM_Pulse = TIM2_CCR4_init;
	timer_oc_init_struct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC4Init(TIM2, &timer_oc_init_struct);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Disable);

	// Only need 3 channels for timer 1
	timer_oc_init_struct.TIM_OutputState = TIM_OutputState_Disable;
	timer_oc_init_struct.TIM_Pulse = TIM1_CCR3_init; // Unchanged because unused 
	TIM_OC4Init(TIM1, &timer_oc_init_struct);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Disable);

	//TIM_ClearFlag(TIM1, TIM_FLAG_Update);
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);
	// Enable the timer interrupts 
	TIM_ITConfig(TIM2, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);
	TIM_ITConfig(TIM1, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3, ENABLE);

	// Enable the timer counters
	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM1, ENABLE); 

	/* TIM configuration for motors */
	timebase_init_struct.TIM_Period = 2400;
	timebase_init_struct.TIM_Prescaler = (uint16_t)(SystemCoreClock/CCR1_Val)-1;
	timebase_init_struct.TIM_ClockDivision = 0;
	timebase_init_struct.TIM_CounterMode = TIM_CounterMode_Up;

	timer_oc_init_struct.TIM_OCMode = TIM_OCMode_PWM1;
	timer_oc_init_struct.TIM_OutputState = TIM_OutputState_Enable;
	timer_oc_init_struct.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_TimeBaseInit(TIM3, &timebase_init_struct);
	timer_oc_init_struct.TIM_Pulse = 0;

	TIM_OC4Init(TIM3, &timer_oc_init_struct);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_OC3Init(TIM3, &timer_oc_init_struct);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_TimeBaseInit(TIM4, &timebase_init_struct);

	TIM_OC4Init(TIM4, &timer_oc_init_struct);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_OC3Init(TIM4, &timer_oc_init_struct);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

	// Enable timer update interrupts
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
}

/*
 * config_nvic - Configures the interrupt vector handler for TIM2 and TIM1
 *
 * Inputs: None
 * 
 * Outputs: None
 */
void config_nvic(void) {

	NVIC_InitTypeDef nvic_init_struct;

	// Want interrupts to trigger from TIM2 and TIM1, and TIM2 should be the higher
	// priority because it controls timing for the important functions and TIM1 just
	// controls the LEDS
	// Setting up interrupts for timer 2
	nvic_init_struct.NVIC_IRQChannel = TIM2_IRQn;
	nvic_init_struct.NVIC_IRQChannelPreemptionPriority = 0;
	nvic_init_struct.NVIC_IRQChannelSubPriority = 1;
	nvic_init_struct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init_struct);

	// Setting up timer 5 interrupts
	nvic_init_struct.NVIC_IRQChannel = TIM1_CC_IRQn ;
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

	// Configure the GPIO peripheral clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	// Disable JTAG so pin attached to red LED can be GPIO
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST, ENABLE);	

	// Pins PB4 and PB5 are attached to the LEDs
	gpio_init_struct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	gpio_init_struct.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpio_init_struct);

	// Configure motor pins for PWM output
	gpio_init_struct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 |GPIO_Pin_9;
	gpio_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &gpio_init_struct);

}

void setMotor1(uint16_t step){
  TIM_TimeBaseInit(TIM3, &timebase_init_struct);
  timer_oc_init_struct.TIM_Pulse = step;

  TIM_OC4Init(TIM3, &timer_oc_init_struct);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
  }

void setMotor2(uint16_t step){
  TIM_TimeBaseInit(TIM3, &timebase_init_struct);
  timer_oc_init_struct.TIM_Pulse = step;

  TIM_OC3Init(TIM3, &timer_oc_init_struct);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
}

void setMotor3(uint16_t step){
  TIM_TimeBaseInit(TIM4, &timebase_init_struct);
  timer_oc_init_struct.TIM_Pulse = CCR1_Val;

  TIM_OC4Init(TIM4, &timer_oc_init_struct);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
}

void setMotor4(uint16_t step){
  TIM_TimeBaseInit(TIM4, &timebase_init_struct);
  timer_oc_init_struct.TIM_Pulse = CCR1_Val;

  TIM_OC3Init(TIM4, &timer_oc_init_struct);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
}

/*
 * TIM2_IRQHandler - The interrupt handler for timer 2. There are 4 possible
 *                   interrupt channels. 
 *
 * Inputs: None
 * Outputs: None
 * Modifies: Possibly extern global var pid_set_flag
 */
void TIM2_IRQHandler(void) {

    // Channel 1 is the interrupt for the detectEmegency() function
    if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET) {

        // Clear the IRQ
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
		TIM_ClearFlag(TIM2, TIM_FLAG_Update);
		emer_flag = 1;
        // Interrupt needs to trigger again in 10ms
        capture  = TIM_GetCapture1(TIM2);
        TIM_SetCompare1(TIM2, capture + DELAY_10MS);

 // Channel 2 is the interrupt for refreshSensorData()
    } else if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET) {

        // Clear the IRQ
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
		TIM_ClearFlag(TIM2, TIM_FLAG_Update);
		data_flag = 1;
        // Interrupt needs to trigger again in 100ms
        capture  = TIM_GetCapture2(TIM2);
        TIM_SetCompare2(TIM2, capture + DELAY_100MS);


    // Channel 3 is the interupt for calculateOrientation()
    } else if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET) {

        // Clear the IRQ
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
		TIM_ClearFlag(TIM2, TIM_FLAG_Update);
		orien_flag = 1;
        // This function refreshes once a second, so no need to reset the
        // compare register
    // Channel 4 is the interrupt for updatePid()
    } else {

        // Clear the IRQ
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
		TIM_ClearFlag(TIM2, TIM_FLAG_Update);
		pid_flag = 1;
        // This function refreshes once a second, so no need to reset the
        // compare register
    }
}

/*
 * TIM1_IRQHandler - The interrupt handler for timer 5. There are 3 possible
 *                   interrupt channels. 
 *
 * Inputs: None
 * Outputs: None
 * Modifies: None
 */
void TIM1_IRQHandler(void) {

    // Channel 1 is the interrupt for the logDebugInfo() function
    if (TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET) {

        // Clear the IRQ
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
		TIM_ClearFlag(TIM1, TIM_FLAG_Update);
		log_flag = 1;
        /* 
         * This function is supposed to happen "whenever there's time," so
         * it can just repeat once a second.
         */

	// Channel 2 is the interrupt handler for the red LED
    } else if (TIM_GetITStatus(TIM1, TIM_IT_CC2) != RESET) {

        // Clear the IRQ
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);
		TIM_ClearFlag(TIM1, TIM_FLAG_Update);

        // Toggle the red LED
        GPIO_WriteBit(GPIOB, GPIO_Pin_4,
             (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_4)));
        // The red LED flashes at a frequency of 0.5 Hz, which means it needs
        // to toggle every second. Don't reset the compare register.

	// Channel 3 is the interrupt handler for the green LED
    } else {
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC3);
		TIM_ClearFlag(TIM1, TIM_FLAG_Update);

        // The green LED flashes at 0.25Hz, so it should only be toggled
        // every other cycle
        if (green_cnt > 0) {

            // Toggle the green LED
            GPIO_WriteBit(GPIOB, GPIO_Pin_5,
                 (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_5)));
            green_cnt = 0;
        } else {
            green_cnt++;
        }
    }
}

