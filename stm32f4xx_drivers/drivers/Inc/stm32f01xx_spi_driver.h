/*
 * stm32f01xx_spi_driver.h
 *
 *  Created on: Dec 20, 2023
 *      Author: hussein
 */

#ifndef INC_STM32F01XX_SPI_DRIVER_H_
#define INC_STM32F01XX_SPI_DRIVER_H_

#include "stm32f401xx.h"

typedef struct{
	uint8_t SPI_DeviceMode;  /* @SPI_DeviceMode spi device modes Master/Slave */
	uint8_t SPI_BusCongif;	/* @SPI_BusConfig bus config */
	uint8_t SPI_SclkSpeed;	/*@SPI_SclkSpeed spi clock speed setup */
	uint8_t SPI_DFF;		/*@SPI_DFF data frame format */
	uint8_t SPI_CPOL;		/*@SPI_CPIL SCLK polarity*/
	uint8_t SPI_CPHA;		/*@SPI_CPHA SCLK phase*/
	uint8_t SPI_SSM;		/*@SPI_SSM software slave management*/
}SPI_Config_t;

typedef struct{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
}SPI_Handle_t;


/*
 * @SPI_DeviceMode
 */
#define SPI_DEV_MODE_MASTER					1
#define SPI_DEV_MODE_SALVE					0

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2
#define SPI_BUS_CONFIG_SIMPLEX_TX			3
#define SPI_BUS_CONFIG_SIMPLEX_RX			4

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2					0
#define SPI_SCLK_SPEED_DIV4					1
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BIT						0
#define SPI_DFF_16BIT						1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_IDLE_LOW					0
#define SPI_CPOL_IDLE_HIGH					1

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_FIRST_EDGE					0
#define SPI_CPHA_SECOND_EDGE				1

/*
 * @SPI_SSM
 */
#define SPI_SSM_DISABLED					0
#define SPI_SSM_ENABLED						1

/*
 ************************ SPI APIs ************************
 */


/*Initialitazion / Deinitialization */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx); /*use RCC peripheral reset register, depending on  which bus the peripheral is hanging on */

/*Peripheral clock control*/
void SPI_PClkControl(SPI_RegDef_t *pSPIx, uint8_t enable);

/*
 * Data send and receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuff, uint32_t TxLen);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuff, uint32_t TxLen);

/*
 * IRQ Config and ISR Handling
 */
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t enable);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);



#endif /* INC_STM32F01XX_SPI_DRIVER_H_ */
