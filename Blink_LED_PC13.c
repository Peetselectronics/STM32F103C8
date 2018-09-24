//Blink by peet's electronics
//Put this code in main.c of new project in TrueStudio
#include <stddef.h>
#include "stm32f10x.h"

//Private variables
 GPIO_InitTypeDef  GPIO_InitStructure;

int main(void)
{
  int i = 0;
  long delayT = 0x500000;

  //Initialize Leds mounted on STM32F103C8 ebay board (the blue pill)
  GPIO_InitTypeDef  GPIO_InitStructure;
  //Initialize LED which connected to PC13, Enable the Clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  //Configure the GPIO_LED pin
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  //Infinite loop
  while (1)
  {
                //delay
	        for(i=0;i<delayT;i++);

	        GPIOC->ODR ^= GPIO_Pin_13;

	        //delay
	        for(i=0;i<delayT;i++);

  }
}
