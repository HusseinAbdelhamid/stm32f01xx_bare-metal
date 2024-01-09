/*
 * SPI_test.c
 *
 *  Created on: Jan 3, 2024
 *      Author: hussein
 */


#include "stm32f401xx.h"
#include "stm32f401xx_gpio_driver.h"
#include "stm32f01xx_spi_driver.h"
#include <string.h>

/*
 * AF 5 for SPI
 * SPI2_MOSI ---> PB15
 * SPI2_MISO ---> PB14
 * SPI2_SCKL ---> PB13
 * SPI2_NSS  ---> PB12
 */

void SPI2_GPIOInits(void){
	GPIO_Handle_t spi2_pins;
	spi2_pins.pGPIOx = GPIOB;
	spi2_pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT;
	spi2_pins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF5;
	spi2_pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP; //SPI doesn't need open drain
	spi2_pins.GPIO_PinConfig.GPIO_PUPDControl = GPIO_NO_PUPD;
	spi2_pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//SCLK
	spi2_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&spi2_pins);

	//MOSI
	spi2_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&spi2_pins);

	//MISO
	spi2_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&spi2_pins);

	//NSS
	spi2_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&spi2_pins);
}

void SPI2_Inits(void){
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusCongif = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEV_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BIT;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_IDLE_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_FIRST_EDGE;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_ENABLED;

	SPI_Init(&SPI2Handle);

}

int main(void){


	char userdata[] = "hello world";

	SPI2_GPIOInits();

	SPI2_Inits();

	SPI_SSIConfig(SPI2, ENABLE);

	SPI_Enable(SPI2);

	SPI_SendData(SPI2, (uint8_t*)userdata, strlen(userdata));

	SPI_Disable(SPI2);

	while(1);

	return 0;
}
