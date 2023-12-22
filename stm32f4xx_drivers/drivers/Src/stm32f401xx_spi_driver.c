/*
 * stm32f401xx_spi_driver.c
 *
 *  Created on: Dec 20, 2023
 *      Author: hussein
 */

#include "stm32f01xx_spi_driver.h"

static void SPI_reset(SPI_RegDef_t *pSPIx){
	uint32_t reset = 0;
	pSPIx->CR1 = reset;
	pSPIx->CR2 = reset;
	pSPIx->SR = 0x0002;
	pSPIx->DR = reset;
	pSPIx->CRCPR = 0x0007;
	pSPIx->RXCRCR = reset;
	pSPIx->TXCRCR = reset;
	pSPIx->I2SCFGR = reset;
	pSPIx->I2SPR = 0x0002;
}

/*Initialitazion / Deinitialization */
void SPI_Init(SPI_Handle_t *pSPIHandle){
	uint32_t temp = 0;
	temp = pSPIHandle->SPIConfig.SPI_DeviceMode;
	temp |= pSPIHandle->SPIConfig.SPI_BusCongif;
	temp |= pSPIHandle->SPIConfig.SPI_SclkSpeed;
	temp |= pSPIHandle->SPIConfig.SPI_CPHA;
	temp |= pSPIHandle->SPIConfig.SPI_CPOL;
	temp |= pSPIHandle->SPIConfig.SPI_DFF;
	temp |= pSPIHandle->SPIConfig.SPI_SSM;
	pSPIHandle->pSPIx->CR1 = temp;
}

/*use RCC peripheral reset register, depending on  which bus the peripheral is hanging on */
void SPI_DeInit(SPI_RegDef_t *pSPIx){

	SPI_reset(pSPIx);
	if(pSPIx == SPI1){
		SPI1_PCLK_DIS();
	}else if(pSPIx == SPI2){
		SPI2_PCLK_DIS();
	}else if(pSPIx == SPI3){
		SPI3_PCLK_DIS();
	}else if(pSPIx == SPI4){
		SPI4_PCLK_DIS();
	}

}

/*Peripheral clock control*/
void SPI_PClkControl(SPI_RegDef_t *pSPIx, uint8_t enable){
	if(enable == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2){
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
		}else if(pSPIx == SPI4){
			SPI4_PCLK_EN();
		}

	}else{
		if(pSPIx == SPI1){
			SPI1_PCLK_DIS();
		}else if(pSPIx == SPI2){
			SPI2_PCLK_DIS();
		}else if(pSPIx == SPI3){
			SPI3_PCLK_DIS();
		}else if(pSPIx == SPI4){
			SPI4_PCLK_DIS();
		}
	}

}

/*
 * Data send and receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuff, uint32_t TxLen){

	for (int i=0;i<TxLen;i++){
		pSPIx->DR = pTxBuff[i];
		while(!(pSPIx->SR & (1 << 1)));
	}

}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuff, uint32_t TxLen){

}

/*
 * IRQ Config and ISR Handling
 */
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t enable){

}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){

}
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){

}
