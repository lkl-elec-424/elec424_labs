#include "intr.h"
#include "lab2.h"

/*
 * intr.c - Contains the interrupt handler routines for lab02.
 */

// #defines
#define DELAY_10MS            400   // 10 ms at frequency of 40KHz
#define DELAY_100MS          4000   // 100 ms at frequency of 40KHz
#define DELAY_500MS			20000   // Half a second at 40KHz

// Global vars
uint16_t capture = 0; 		// When an interrupt was triggered
uint16_t green_cnt = 0; 	// Counter for the green LED

/*
 * TIM2_IRQHandler - The interrupt handler for timer 2. There are 4 possible
 *					 interrupt channels. 
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
		detectEmergency();

		// Interrupt needs to trigger again in 10ms
		capture  = TIM_GetCapture1(TIM2);
		TIM_SetCompare1(TIM2, capture + DELAY_10MS);

	// Channel 2 is the interrupt for refreshSensorData()
	} else if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET) {

		// Clear the IRQ
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
		refreshSensorData();

		// Interrupt needs to trigger again in 100ms
		capture  = TIM_GetCapture2(TIM2);
		TIM_SetCompare2(TIM2, capture + DELAY_100MS);


	// Channel 3 is the interupt for calculateOrientation()
	} else if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET) {

		// Clear the IRQ
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
		calculateOrientation();
		// This function refreshes once a second, so no need to reset the
		// compare register

	// Channel 4 is the interrupt for updatePid()
	} else {

		// Clear the IRQ
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);

		// This function returns new motor PIDs, which means the main
		// program needs to do additional processing after the interrupt
		updatePid(&motor_speeds);
		pid_set_flag = True;
		// This function refreshes once a second, so no need to reset the
		// compare register
	}

}

/*
 * TIM5_IRQHandler - The interrupt handler for timer 5. There are 3 possible
 *					 interrupt channels. 
 *
 * Inputs: None
 * Outputs: None
 * Modifies: None
 */
void TIM5_IRQHandler(void) {

	// Channel 1 is the interrupt for the logDebugInfo() function
	if (TIM_GetITStatus(TIM5, TIM_IT_CC1) != RESET) {

		// Clear the IRQ
		TIM_ClearITPendingBit(TIM5, TIM_IT_CC1);
		logDebugInfo();
		/* 
		 * This function is supposed to happen "whenever there's time," so
		 * it can just repeat once a second.
		 */

	// Channel 2 is the interrupt handler for the red LED
	} else if (TIM_GetITStatus(TIM5, TIM_IT_CC2) != RESET) {

		// Clear the IRQ
		TIM_ClearITPendingBit(TIM5, TIM_IT_CC2);

		// Toggle the red LED
		GPIO_WriteBit(GPIOB, GPIO_Pin_4,
			 (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_4));
		// The red LED flashes at a frequency of 0.5 Hz, which means it needs
		// to toggle every second. Don't reset the compare register.

	// Channel 3 is the interrupt handler for the green LED
	} else {
		TIM_ClearITPendingBit(TIM5, TM_IT_CC3);

		// The green LED flashes at 0.25Hz, so it should only be toggled
		// every other cycle
		if (green_cnt > 0) {

			// Toggle the green LED
			GPIO_WriteBit(GPIOB, GPIO_Pin_5,
				 (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_5));
			green_cnt = 0;
		} else {
			green_cnt++;
		}
	}

}
