#include "lab02.h"
#include "init.h"
#include "stm32f10x_it.h"
#include "lab2.h"
#include "stm32f10x_conf.h"

/* #Defines */
#define MOTOR_PULSE_WIDTH 		24 			// PWM duty cycle width
#define MOTOR_PERIOD			2400		// Total period for PWM
#define SYS_CORE_CLK			72000000	// Clock speed of 72MHz
#define INTERRUPT_PRIORITY_0	0x00		// Interrupt priority of 0
#define DIV_10MS				100			// Sets SysTick interrupt to
											// every 10 ms
#define TIMING_DELAY			1000		// Intial value for TimingDelay
#define DELAY_100MS				10			// Counts until 100ms have elapsed
#define DELAY_SEC				100			// Counts until 1 second 
#define DELAY_2SEC				200			// Counts until 2 seconds

// These offsets are used to schdule the different tasks in the SysTick
// IRQ handler.
#define LED_OFFSET				0			// Offset for LEDs in task chooser
#define DATA_OFFSET				4			// Offset for getting sensor data
#define CALC_OFFSET				9			// Offset for orientation calc
#define MOTOR_OFFSET			7			// Offset for getting and setting
											// motor speeds
#define DEBUG_OFFSET			1			// Offset for logging debug info


/* Global variables */
// Structures to initialize the motor timers
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;

// Pulse width for motors
uint16_t CCR1_Val = MOTOR_PULSE_WIDTH;

// Clock speed as set in sys_init()
uint32_t SystemCoreClock = SYS_CORE_CLK;

// Motor speed variables
uint16_t motor1_speed;
uint16_t motor2_speed;
uint16_t motor3_speed;
uint16_t motor4_speed;

// Motor speed struct
MotorSpeeds motor_speeds;

// Timing variable controlled by SysTick interrupts
static  __IO uint32_t TimingDelay;

/* Private function prototypes -----------------------------------------------*/
void GPIO_Configuration(void);
void RCC_Configuration(void);
void setMotor1(uint16_t step);
void setMotor2(uint16_t step);
void setMotor3(uint16_t step);
void setMotor4(uint16_t step);

int main(void) {

	// Initialize the system clock to 72 MHz
	sys_init();

	/* System Clocks Configuration */
	RCC_Configuration();

	/* GPIO Configuration */
	GPIO_Configuration();

	// Start SysTick and configure it to request an interrupt every 10 ms
	if (SysTick_Config(SystemCoreClock/DIV_10MS)) {
			while(1);
	}

	// Set the SysTick interrupt to the highest priority
	NVIC_SetPriority(SysTick_IRQn, INTERRUPT_PRIORITY_0);

	// Initialize the motor timers
	TIM_TimeBaseStructure.TIM_Period = MOTOR_PERIOD;
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)(SystemCoreClock/CCR1_Val)-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	// Set timers to PWM mode
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	// Set initial value of TimingDelay
	TimingDelay = TIMING_DELAY;

	// Stay here forever and wait for interrupts
	while(1) {
	}
}

/*
 * void RCC_Configuration - Initial setup of the RCC for the various 
 *						    timers and peripherals.
 *
 * Inputs: None
 * Outputs: None
 */
void RCC_Configuration(void)
{
   /* TIM3 and TIM4 clock enable */
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 |
                         RCC_APB1Periph_TIM4, ENABLE);
    /* GPIOB and AFIO clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
}
 
/* 
 * void GPIO_Configuration - Set up the functionality of the GPIO pins used.
 * 							 The motor pins and the red LED pin are all their
 *							 alternate functions, and the green LED pin is its
 *							 primary function. Also disables the JTAG
 *							 functionality of the red LED pin.
 *
 * Inputs - None
 * Outputs - None
 */  
void GPIO_Configuration(void)
{

  // Disable JTAG so pin attached to red LED can be GPIO
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST, ENABLE);

  // Configure pins for motors
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 |GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  //Configure pins for LEDs
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/* 
 * void setMotor1 - Sets the pulse width for motor 1, allowing it to be 
 *				 	turned off or turned on and set to a specified speed.
 *
 * Inputs - uint16_t step: The width of the pulse for the motor's PWM control.
 * Outputs - None
 */	
void setMotor1(uint16_t step){
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_OCInitStructure.TIM_Pulse = step;

  TIM_OC4Init(TIM3, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
}

/* 
 * void setMotor2 - Sets the pulse width for motor 2, allowing it to be 
 *				 	turned off or turned on and set to a specified speed.
 *
 * Inputs - uint16_t step: The width of the pulse for the motor's PWM control.
 * Outputs - None
 */	
void setMotor2(uint16_t step){
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_OCInitStructure.TIM_Pulse = step;

  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
}

/* 
 * void setMotor3 - Sets the pulse width for motor 3, allowing it to be 
 *				 	turned off or turned on and set to a specified speed.
 *
 * Inputs - uint16_t step: The width of the pulse for the motor's PWM control.
 * Outputs - None
 */	
void setMotor3(uint16_t step){
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  TIM_OCInitStructure.TIM_Pulse = step;

  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
}

/* 
 * void setMotor4 - Sets the pulse width for motor 4, allowing it to be 
 *				 	turned off or turned on and set to a specified speed.
 *
 * Inputs - uint16_t step: The width of the pulse for the motor's PWM control.
 * Outputs - None
 */	
void setMotor4(uint16_t step){
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  TIM_OCInitStructure.TIM_Pulse = step;

  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
}

/*
 * void TimingDelay_Decrement - Decrements the TimingDelay counter, or wraps
 *								it back to the highest counter value if
 *								TimingDelay has hit 0.
 * 
 * Inputs - None
 * Outputs - None
 */
void TimingDelay_Decrement(void)
{
  if(TimingDelay != 0) {
      TimingDelay--;
  } else {
		// The -1 is to avoid running the functions controlled by logic with
		// a 0 offset twice in a row.
      TimingDelay = TIMING_DELAY - 1;
  }
}

/*
 * void chooseTask - Chooses which task(s) to execute based on the current
 *					 value of TimingDelay. Called from the SysTick IRQ handler.
 *
 * Inputs - None
 * Outputs - None
 */
void chooseTask(void) {

	// Interrupt happens every 10 ms, so detectEmergency() must happen 
    // every time this function is called.
	detectEmergency();

	// The offsets are all meant to be orthogonal to each other so that none
	// of the tasks are delayed.
	if (TimingDelay % DELAY_100MS == DATA_OFFSET){
    	refreshSensorData();
	}
	else if (TimingDelay % DELAY_SEC == CALC_OFFSET){
		calculateOrientation();
	}
	else if (TimingDelay % DELAY_SEC == MOTOR_OFFSET){
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
	}
	else if (TimingDelay % DELAY_100MS == DEBUG_OFFSET){
		logDebugInfo();
	}

	// Toggle the green LED
	if(TimingDelay % DELAY_SEC == LED_OFFSET) {
		GPIO_WriteBit(GPIOB, GPIO_Pin_5,
			(BitAction)(1 - GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_5)));
	}

	// Toggle the red LED
	if (TimingDelay % DELAY_2SEC == LED_OFFSET){
			GPIO_WriteBit(GPIOB, GPIO_Pin_4,
				(BitAction)(1 - GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_4)));
	}
}

