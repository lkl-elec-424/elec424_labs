/*
 *	Lab03 - FreeRTOS
 *  ELEC 424
 *	Zihe Huang, Lindsay Langford, Minh Nguyen 
 *
 */ 

/* Local includes */
#include "init.h"
#include "lab3.h"

/* STM32 includes */
#include "stm32f10x_conf.h"

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "portmacro.h"

/* #Defines */
#define MOTOR_PULSE_WIDTH 		24 			// PWM duty cycle width
#define MOTOR_PERIOD			2400		// Total period for PWM
#define SYS_CORE_CLK			72000000	// Clock speed of 72MHz
#define INTERRUPT_PRIORITY_0	0x00		// Interrupt priority of 0

#define TICKS_10MS				10			// Clock ticks in 10ms
#define TICKS_100MS				100			// Clock ticks in 100ms
#define TICKS_1SEC				1000		// Clock ticks in 1 second
#define TICKS_2SEC				2000	    // Clock ticks in 2 seconds
#define STACK_SIZE 				100			// Stack size for each task

// Relative priorities of all the different tasks.
// tskIDLE_PRIORITY is the lowest priority.
#define EMERGENCY_TASK_PRIORITY 		(tskIDLE_PRIORITY + 5)
#define DATA_TASK_PRIORITY 				(tskIDLE_PRIORITY + 4)
#define CALC_TASK_PRIORITY 				(tskIDLE_PRIORITY + 3)
#define PID_TASK_PRIORITY 				(tskIDLE_PRIORITY + 2)
#define LOG_TASK_PRIORITY 				(tskIDLE_PRIORITY + 1)
#define LED_TASK_PRIORITY				tskIDLE_PRIORITY 

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

// Semaphore handle
xSemaphoreHandle dataSemaphore;

/* Private function prototypes -----------------------------------------------*/
void GPIO_Configuration(void);
void RCC_Configuration(void);
void setMotor1(uint16_t step);
void setMotor2(uint16_t step);
void setMotor3(uint16_t step);
void setMotor4(uint16_t step);

/* FreeRTOS Task Prototypes */
void FreeRTOS_Configuration(void);
void vTaskDetectEmergency(void *pvParameters); 
void vTaskRefreshData(void *pvParameters);
void vTaskCalcOrientation(void *pvParameters); 
void vTaskUpdatePid(void *pvParameters); 
void vTaskLogDebugInfo(void *pvParameters);
void vTaskRedLED(void *pvParameters); 
void vTaskGreenLED(void *pvParameters); 

int main(void) {

	// Initialize the system clock to 72 MHz
	sys_init();

	/* System Clocks Configuration */
	RCC_Configuration();

	/* GPIO Configuration */
	GPIO_Configuration();

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

	// Create the semaphore and tasks	
	FreeRTOS_Configuration();	
	
	// Start the FreeRTOS scheduler
	vTaskStartScheduler();

	// Should not ever get here!
	return 0;
}

/*
 * void FreeRTOSConfiguration - Creates all of the tasks and the binary
 *								semaphore used for scheduling.
 *
 * Inputs: None
 * Outputs: None
 */
void FreeRTOS_Configuration(void) {

	// Create the binary semaphore
	vSemaphoreCreateBinary(dataSemaphore);

	// If the semaphore doesn't get created, TRAP.
	if (dataSemaphore == NULL) {
		for (;;){}	
	}

	/*
	 * Take the semaphore so that:
	 * 1. The task for refreshSensorData can give it
	 * 2. The task for calculateOrientation MUST wait for refreshSensorData
	 *	  to execute to take it.
     */
	xSemaphoreTake(dataSemaphore, 0);

	/*
	 * Task creation - The name, paramters, and task handle are all not used
     * and so are left as NULL.
     */
	xTaskCreate(vTaskDetectEmergency, NULL, 100 , NULL,
	    EMERGENCY_TASK_PRIORITY, NULL);
	xTaskCreate(vTaskRefreshData, NULL, 100 , NULL,
	    DATA_TASK_PRIORITY, NULL);
	xTaskCreate(vTaskCalcOrientation, NULL, 100 , NULL,
	    CALC_TASK_PRIORITY, NULL);
	xTaskCreate(vTaskUpdatePid, NULL, 100 , NULL,
	    PID_TASK_PRIORITY, NULL);
	xTaskCreate(vTaskLogDebugInfo, NULL, 100 , NULL,
	    LOG_TASK_PRIORITY, NULL);
	xTaskCreate(vTaskGreenLED, NULL, 100 , NULL,
		 LED_TASK_PRIORITY, NULL);
	xTaskCreate(vTaskRedLED, NULL, 100 , NULL,
		 LED_TASK_PRIORITY, NULL);
}

/*
 * void RCC_Configuration - Initial setup of the RCC for the various 
 *						    timers and peripherals.
 *
 * Inputs: None
 * Outputs: None
 */
void RCC_Configuration(void) {

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
void GPIO_Configuration(void) {

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
void setMotor1(uint16_t step) {
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
void setMotor2(uint16_t step) {
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
void setMotor3(uint16_t step) {
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
void setMotor4(uint16_t step) {
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  TIM_OCInitStructure.TIM_Pulse = step;

  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
}

/*
 * void vTaskDetectEmergency - A task which calls detectEmergency() 
 * 							   every 10ms.
 * 
 * Inputs: None
 * Outputs: None
 */
void vTaskDetectEmergency(void *pvParameters) {

	// Initialize the wake time for the delays
	portTickType xLastWakeTime;
	const portTickType xFrequency = TICKS_10MS;
	xLastWakeTime = xTaskGetTickCount();

	// Execute task and then delay
	for (;;) {
		detectEmergency();
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
} 

/*
 * void vTaskRefreshData - A task which calls refreshSensorData() 
 * 						   every 100ms and gives the binary semaphore to
 *						   vTaskCalcOrientation.
 * 
 * Inputs: None
 * Outputs: None
 */
void vTaskRefreshData(void *pvParameters) {

	// Initialize the wake time for the delays
	portTickType xLastWakeTime;
	const portTickType xFrequency = TICKS_100MS;
	xLastWakeTime = xTaskGetTickCount();

	// Execute task and then delay
	for (;;) {

    	refreshSensorData();

		// Give the semaphore so that the quadcopter orientation
		// can be calculated
		xSemaphoreGive(dataSemaphore);
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

	}
} 

/*
 * void vTaskCalcOrientation - A task which calls calculateOrientation() 
 * 							   every 100ms after getting the semaphore from
 *							   vTaskRefreshData.
 * 
 * Inputs: None
 * Outputs: None
 */
void vTaskCalcOrientation(void *pvParameters) {

	// Initialize the wake time for the delays
	portTickType xLastWakeTime;
	const portTickType xFrequency = TICKS_100MS;
	xLastWakeTime = xTaskGetTickCount();

	// Execute task and then delay
	for (;;) {

		// Block until the semaphore becomes available
		xSemaphoreTake(dataSemaphore, portMAX_DELAY);
		calculateOrientation();
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
} 

/*
 * void vTaskUpdatePid - A task which calls updatePid() and sets the motors
 * 					     every second.
 * 
 * Inputs: None
 * Outputs: None
 */
void vTaskUpdatePid(void *pvParameters) {

	// Initialize the wake time for the delays
	portTickType xLastWakeTime;
	const portTickType xFrequency = TICKS_1SEC;
	xLastWakeTime = xTaskGetTickCount();

	// Execute task and then delay
	for (;;) {
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

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
} 

/*
 * void vTaskLogDebugInfo - A task which calls logDebugInfo() every second.
 * 
 * Inputs: None
 * Outputs: None
 */
void vTaskLogDebugInfo(void *pvParameters) {

	// Initialize the wake time for the delays
	portTickType xLastWakeTime;
	const portTickType xFrequency = TICKS_10MS;
	xLastWakeTime = xTaskGetTickCount();

	// Execute task and then delay
	for (;;) {
		logDebugInfo();
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
} 

/*
 * void vTaskRedLED - A task which calls blinks the red LED at 0.25Hz 
 * 
 * Inputs: None
 * Outputs: None
 */
void vTaskRedLED(void *pvParameters) {

	// Initialize the wake time for the delays
	portTickType xLastWakeTime;
	const portTickType xFrequency = TICKS_2SEC;
	xLastWakeTime = xTaskGetTickCount();

	// Execute task and then delay
	for (;;) {
		GPIO_WriteBit(GPIOB, GPIO_Pin_4,
			(BitAction)(1 - GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_4)));
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
} 

/*
 * void vTaskGreenLED - A task which calls blinks the green LED at 0.25Hz 
 * 
 * Inputs: None
 * Outputs: None
 */
void vTaskGreenLED(void *pvParameters) {

	// Initialize the wake time for the delays
	portTickType xLastWakeTime;
	const portTickType xFrequency = TICKS_1SEC;
	xLastWakeTime = xTaskGetTickCount();

	// Execute task and then delay
	for (;;) {
	   	GPIO_WriteBit(GPIOB, GPIO_Pin_5,
			(BitAction)(1 - GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_5)));
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
} 

