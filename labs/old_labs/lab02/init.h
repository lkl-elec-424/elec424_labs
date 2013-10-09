#ifndef __INIT_H
#define __INIT_H

// Macro definitions

/* 
 * When calling RCC_GetSYSCLKSource, the return value 0x08 corresponds to 
 * the PLLCLK.    
 */ 
#define PLLCLK 0x08

// Function definitions
void sys_init(void);

#endif
