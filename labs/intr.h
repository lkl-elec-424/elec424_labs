#ifndef __INTR_H
#define __INTR_H
/* 
 * intr.h - The header file for the interrupt handler functions used in lab02.
 */

// External variables from the main file
extern bool pid_set_flag;
extern MotorSpeeds motor_speeds;

// Function declarations
void TIM2_IRQHandler(void);
void TIM5_IRQHandler(void);

#endif
