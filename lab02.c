#include "main.h"
#include "init.h"
#include "stm32f10x_it.h"
#include "lab2.h"
#include "stm32f10x_conf.h"
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;

uint16_t CCR1_Val = 240;
uint32_t SystemCoreClock = 72000000;
uint16_t motor1_speed;
uint16_t motor2_speed;
uint16_t motor3_speed;
uint16_t motor4_speed;
RCC_ClocksTypeDef RCC_ClockFreq;
static  __IO uint32_t TimingDelay;
MotorSpeeds motor_speeds;


/* Private function prototypes -----------------------------------------------*/
void Delay(__IO uint32_t nTime);

void RCC_Configuration(void);
void GPIO_Configuration(void);
void setMotor1(uint16_t step);
void setMotor2(uint16_t step);
void setMotor3(uint16_t step);
void setMotor4(uint16_t step);

int main(void)
{
  int pid_set_flag;
  //sys_init();
  /* System Clocks Configuration */
  RCC_Configuration();
  RCC_GetClocksFreq(&RCC_ClockFreq);
  /* GPIO Configuration */
  GPIO_Configuration();

  if (SysTick_Config(SystemCoreClock/1000))
    {
      while(1);
    }
  TIM_TimeBaseStructure.TIM_Period = 2400;
  TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)(SystemCoreClock/CCR1_Val)-1;;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;


  /* TIM configuration */
TimingDelay = 10000;
  while(1)
  {
    if(TimingDelay % 10 == 0)
      detectEmergency();
    else if (TimingDelay % 100 == 4)
      refreshSensorData();
    else if (TimingDelay % 1000 == 9)
      calculateOrientation();
    else if (TimingDelay % 1000 == 7){
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
    else if (TimingDelay % 100 == 1){
      logDebugInfo();
    }
	
  }
}
void RCC_Configuration(void)
{
   /* TIM3 and TIM4 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 |
                         RCC_APB1Periph_TIM4, ENABLE);
    /* GPIOB clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
}

void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 |GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void setMotor1(uint16_t step){
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_OCInitStructure.TIM_Pulse = step;

  TIM_OC4Init(TIM3, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /*TIM_OCInitStructure.TIM_Pulse = 0;

  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_TimeBaseInit(TIM4, &timebase_init_struct);

  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

  Delay(1000);*/
}

void setMotor2(uint16_t step){
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_OCInitStructure.TIM_Pulse = step;

  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
}

void setMotor3(uint16_t step){
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  TIM_OCInitStructure.TIM_Pulse = step;

  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
}

void setMotor4(uint16_t step){
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  TIM_OCInitStructure.TIM_Pulse = step;

  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
}

void Delay(__IO uint32_t nTime)
    {
        TimingDelay = nTime;

        while(TimingDelay != 0);
    }
void TimingDelay_Decrement(void)
{
  if(TimingDelay != 0x00)
    {
      TimingDelay--;
    }else if(TimingDelay == 0x00){
        TimingDelay = 10000;
    }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
