/*
 * stm32f401xx_gpio_driver.h
 *
 *  Created on: Dec 16, 2023
 *      Author: hussein
 */

#ifndef INC_STM32F401XX_GPIO_DRIVER_H_
#define INC_STM32F401XX_GPIO_DRIVER_H_

#include "stm32f401xx.h"


/*gpio pin numbers*/
#define GPIO_PIN_0 			0
#define GPIO_PIN_1 			1
#define GPIO_PIN_2 			2
#define GPIO_PIN_3 			3
#define GPIO_PIN_4 			4
#define GPIO_PIN_5 			5
#define GPIO_PIN_6 			6
#define GPIO_PIN_7 			7
#define GPIO_PIN_8 			8
#define GPIO_PIN_9 			9
#define GPIO_PIN_10 		10
#define GPIO_PIN_11 		11
#define GPIO_PIN_12 		12
#define GPIO_PIN_13 		13
#define GPIO_PIN_14 		14
#define GPIO_PIN_15 		15

/*@GPIO_PIN_MODES*/
/*gpio pin modes macros*/
#define GPIO_MODE_INPUT 	0
#define GPIO_MODE_OUTPUT 	1
#define GPIO_MODE_ALT 		2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT 	4  /*falling edge interrupt*/
#define GPIO_MODE_IT_RT 	5  /*rising edge interrupt*/
#define GPIO_MODE_IT_FRT 	6  /*falling and rising edge interrupt*/

/*gpio pin output type macros*/
#define GPIO_OPTYPE_PP 		0 /*output push pull*/
#define GPIO_OPTYPE_OD 		1 /*output open drain*/

/*gpio port output speeds macros*/
#define GPIO_SPEED_LOW 		0
#define GPIO_SPEED_MEDIUM 	1
#define GPIO_SPEED_HIGH 	2
#define GPIO_SPEED_VERYHIGH 3

/*gpio pull-up/pull-down*/
#define GPIO_NO_PUPD 		0
#define GPIO_PU 			1
#define GPIO_PD 			2

/*gpio alternate function macros*/
#define GPIO_AF0 			0
#define GPIO_AF1 			1
#define GPIO_AF2 			2
#define GPIO_AF3 			3
#define GPIO_AF4 			4
#define GPIO_AF5 			5
#define GPIO_AF6 			6
#define GPIO_AF7 			7
#define GPIO_AF8 			8
#define GPIO_AF9 			9
#define GPIO_AF10 			10
#define GPIO_AF11 			11
#define GPIO_AF12 			12
#define GPIO_AF13 			13
#define GPIO_AF14 			14
#define GPIO_AF15 			15

typedef struct {
  uint8_t GPIO_PinNumber;
  uint8_t GPIO_PinMode; /*!< possible values from @GPIO_PIN_MODES>*/
  uint8_t GPIO_PinSpeed;
  uint8_t GPIO_PUPDControl;
  uint8_t GPIO_PinOPType;
  uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

typedef struct {
  GPIO_RegDef_t *pGPIOx; /*base address of gpio port the bin belongs to*/
  GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;

/*API's supported by gpio driver*/

/*Initialitazion / Deinitialization */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(
    GPIO_RegDef_t *pGPIOx); /*use RCC peripheral reset register, depending on
                               which bus the peripheral is hanging on */

/*Peripheral clock control*/
void GPIO_PClkControl(GPIO_RegDef_t *pGPIOx, uint8_t enable);

/*Data read and write*/
uint8_t GPIO_ReadFromInpPin(GPIO_RegDef_t *pGPIOx, uint8_t Pin);
uint16_t GPIO_ReadFromInpPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutPin(GPIO_RegDef_t *pGPIOx, uint8_t Pin, uint8_t output);
void GPIO_WriteToOutPort(GPIO_RegDef_t *pGPIOx, uint16_t output);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t Pin);

/*Interrupt config and handling*/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t enable);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t Pin);

#endif /* INC_STM32F401XX_GPIO_DRIVER_H_ */
