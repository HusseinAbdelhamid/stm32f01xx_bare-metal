/*
 * main.c
 *
 *  Created on: Dec 16, 2023
 *      Author: hussein
 */


#include "stm32f401xx.h"
#include "stm32f401xx_gpio_driver.h"

void delay(void){
	for(int i=0 ;i<500000/2;i++){

	}
}
GPIO_Handle_t gpio_led;
GPIO_Handle_t gpio_button;

int main(){

	/*Init led pin*/
	gpio_led.pGPIOx = GPIOA;
	gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_8;
	gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
	gpio_led.GPIO_PinConfig.GPIO_PUPDControl = GPIO_NO_PUPD;
	GPIO_PClkControl(gpio_led.pGPIOx, ENABLE);
	GPIO_Init(&gpio_led);

	/*Init button pin*/
	gpio_button.pGPIOx = GPIOB;
	gpio_button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	gpio_button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
//	gpio_button.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
	gpio_button.GPIO_PinConfig.GPIO_PUPDControl = GPIO_PU;
	GPIO_Init(&gpio_button);

	GPIO_PClkControl(gpio_button.pGPIOx, ENABLE);
	GPIO_IRQConfig(IRQ_NO_EXTI15_10, ENABLE);
//	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, 1);
	while(1){
//		if(GPIO_ReadFromInpPin(gpio_button.pGPIOx, gpio_button.GPIO_PinConfig.GPIO_PinNumber) == GPIO_PIN_RESET){
//			delay(); /*for button debouncing  */
//			GPIO_TogglePin(gpio_led.pGPIOx, gpio_led.GPIO_PinConfig.GPIO_PinNumber);
//		}
	}

	return 0;

}

void EXTI15_10_IRQHandler(void){

	GPIO_IRQHandling(GPIO_PIN_12);
	GPIO_TogglePin(GPIOA, GPIO_PIN_8);

}

