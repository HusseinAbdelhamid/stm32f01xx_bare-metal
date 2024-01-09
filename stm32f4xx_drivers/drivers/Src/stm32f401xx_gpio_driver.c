/*
 * stm32f401xx_gpio_driver.c
 *
 *  Created on: Dec 16, 2023
 *      Author: hussein
 */

#include <stm32f401xx_gpio_driver.h>

/*Initialitazion / Deinitialization */
/**
 * @brief Initialize GPIO port
 *
 * @param pGPIOx base address of gpio peripheral
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {

	uint8_t pin_number = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	uint32_t temp=0;

	// enable peripheral clock
	GPIO_PClkControl(pGPIOHandle->pGPIOx, ENABLE);

	//1. configure mode of gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		// non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pin_number));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3<<pin_number);
		pGPIOHandle->pGPIOx->MODER |= temp;
	}else{
		// interrupt mode

		pGPIOHandle->pGPIOx->MODER &= ~(0x3<<pin_number); // clear to configure pin mode type as input

		//1. Configure rising/falling edge

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			EXTI->FTSR |= (1<<pin_number);
			EXTI->RTSR &= ~(1<<pin_number); //clear corresponding RTSR bit
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			EXTI->RTSR |= (1<<pin_number);
			EXTI->FTSR &= ~(1<<pin_number); //clear corresponding FTSR bit
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FRT){
			EXTI->RTSR |= (1<<pin_number);
			EXTI->FTSR |= (1<<pin_number);
		}

		// 2. Configure gpio port selection in SYSCFG_EXTICR
		uint8_t temp1 = pin_number/4;
		uint8_t temp2 = pin_number%4;

		uint16_t port_code = GPIO_ADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();

		//SYSCFG->EXTICR[temp1] &= ~(0xF << 4*temp2);
		SYSCFG->EXTICR[temp1] |= (port_code << 4*temp2);

		//3. Enable the exti innterupt delivery using IMR
		EXTI->IMR |= (1<<pin_number);

	}

	temp = 0;
	//2. configure speed
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pin_number);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3<<pin_number);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;
	//3. configure pupd settings
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PUPDControl << (2 * pin_number);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3<<pin_number);
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;
	//4. configure output type
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pin_number);
	pGPIOHandle->pGPIOx->OTYPER &= ~(1<<pin_number);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	//5. configure alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALT){

		if(pin_number < GPIO_PIN_8){
			temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*pin_number);
			pGPIOHandle->pGPIOx->AFRL &= ~(0xF<<pin_number);
			pGPIOHandle->pGPIOx->AFRL |= temp;
		}else{
			temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*(pin_number%8));
			pGPIOHandle->pGPIOx->AFRH &= ~(0xF<<pin_number%8);
			pGPIOHandle->pGPIOx->AFRH |= temp;
		}

	}

}

/**
 * @brief reset GPIOx peripheral registers
 *
 * @param pGPIOx
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){ /*use RCC peripheral reset register, depending on which bus the peripheral is hanging on */

	 if (pGPIOx == GPIOA) {
	      GPIOA_RESET();
	    } else if (pGPIOx == GPIOB) {
	      GPIOB_RESET();
	    } else if (pGPIOx == GPIOC) {
	      GPIOC_RESET();
	    } else if (pGPIOx == GPIOD) {
	      GPIOD_RESET();
	    } else if (pGPIOx == GPIOE) {
	      GPIOE_RESET();
	    } else if (pGPIOx == GPIOH) {
	      GPIOH_RESET();
	    }

}

/*Peripheral clock control*/
/**
 * @brief Enable/Disable peripheral clock
 *
 * @param pGPIOx    base address of gpio peripheral
 * @param enable    ENABLE/DISABLE macros
 */
void GPIO_PClkControl(GPIO_RegDef_t *pGPIOx, uint8_t enable) {

  if (enable == ENABLE) {

    if (pGPIOx == GPIOA) {
      GPIOA_PCLK_EN();
    } else if (pGPIOx == GPIOB) {
      GPIOB_PCLK_EN();
    } else if (pGPIOx == GPIOC) {
      GPIOC_PCLK_EN();
    } else if (pGPIOx == GPIOD) {
      GPIOD_PCLK_EN();
    } else if (pGPIOx == GPIOE) {
      GPIOE_PCLK_EN();
    } else if (pGPIOx == GPIOH) {
      GPIOH_PCLK_EN();
    }

  } else {
    if (pGPIOx == GPIOA) {
      GPIOA_PCLK_DIS();
    } else if (pGPIOx == GPIOB) {
      GPIOB_PCLK_DIS();
    } else if (pGPIOx == GPIOC) {
      GPIOC_PCLK_DIS();
    } else if (pGPIOx == GPIOD) {
      GPIOD_PCLK_DIS();
    } else if (pGPIOx == GPIOE) {
      GPIOE_PCLK_DIS();
    } else if (pGPIOx == GPIOH) {
      GPIOH_PCLK_DIS();
    }
  }
}

/*Data read and write*/
/**
 * @brief Read input of pin 'Pin' from GPIOx port
 *
 * @param pGPIOx base address of gpio peripheral
 * @param Pin    pin number
 * @return uint8_t input pin value
 */
uint8_t GPIO_ReadFromInpPin(GPIO_RegDef_t *pGPIOx, uint8_t Pin){

	uint8_t out = (uint8_t)(pGPIOx->IDR >> Pin) & (0x00000001);
	return out;

}

/**
 * @brief Read all pin values of GPIOx port
 *
 * @param pGPIOx base address of gpio peripheral
 * @return uint16_t all pin values of port GPIOx
 */
uint16_t GPIO_ReadFromInpPort(GPIO_RegDef_t *pGPIOx){

	uint16_t out = (uint16_t) pGPIOx->IDR;
	return out;

}

/**
 * @brief Set pin 'Pin' of GPIOx to 'val'
 *
 * @param pGPIOx base address of gpio peripheral
 * @param Pin    pin number
 * @param val    GPIO_PIN_SET/GPIO_PIN_RESET
 */
void GPIO_WriteToOutPin(GPIO_RegDef_t *pGPIOx, uint8_t Pin, uint8_t val){

	if(val == GPIO_PIN_SET){
		pGPIOx->ODR |= (1<<Pin);
	}else{
		pGPIOx->ODR &= ~(1<<Pin);
	}

}

/**
 * @brief Set pins of GPIOx port to 'val'
 *
 * @param pGPIOx base address of gpio peripheral
 * @param val    port value
 */
void GPIO_WriteToOutPort(GPIO_RegDef_t *pGPIOx, uint16_t val){
	pGPIOx->ODR = val;
}

/**
 * @brief Toggle 'Pin' of port GPIOx
 *
 * @param pGPIOx base address of gpio peripheral
 * @param Pin    pin number
 */
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t Pin){
	pGPIOx->ODR ^= (1 << Pin);
}

/*Interrupt config and handling*/
/**
 * @brief   Configure Interrupt
 *
 * @param IRQNumber
 * @param IRQPriority priority for interrupt
 * @param enable      ENABLE/DISABLE macros
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t enable){

	if(enable == ENABLE){

		if(IRQNumber <=31){

			*NVIC_ISER0 |= (1<<IRQNumber);

		}else if (IRQNumber > 31 && IRQNumber < 64){

			*NVIC_ISER1 |= (1<< (IRQNumber%32) );

		}else if (IRQNumber >=64 && IRQNumber < 96){

			*NVIC_ISER2 |= (1<< (IRQNumber%32) );
		}

	}else{
		if(IRQNumber <=31){

			*NVIC_ICER0 |= (1<<IRQNumber);

		}else if (IRQNumber > 31 && IRQNumber < 64){

			*NVIC_ICER1 |= (1<< (IRQNumber%32) );

		}else if (IRQNumber >=64 && IRQNumber < 96){

			*NVIC_ICER2 |= (1<< (IRQNumber%32) );
		}

	}

}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){

	*NVIC_IPRx(IRQNumber) |= ((IRQPriority << 8*(IRQNumber%4)) << (8-NO_PR_BITS_IMPLEMENTED));

}

void GPIO_IRQHandling(uint8_t Pin){
	// clear bit in EXTI pending register by writing 1
	if(EXTI->PR & (1 << Pin)){
		EXTI->PR |= (1 << Pin);
	}
}
