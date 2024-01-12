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
	temp = pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	SPI_PClkControl(pSPIHandle->pSPIx, ENABLE);

	if(pSPIHandle->SPIConfig.SPI_BusCongif == SPI_BUS_CONFIG_FD){
		//BIDI mode should be cleared
		temp &= ~(1<<SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusCongif == SPI_BUS_CONFIG_HD){
		//BIDI mode should be set
		temp |= (1<<SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusCongif == SPI_BUS_CONFIG_SIMPLEX_RX){
		//BIDI mode should be cleared
		temp &= ~(1<<SPI_CR1_BIDIMODE);
		//RXONLY should be set
		temp |= (1<<SPI_CR1_RXONLY);
	}

	temp |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	temp |= (pSPIHandle->SPIConfig.SPI_CPHA <<SPI_CR1_CPHA);

	temp |= (pSPIHandle->SPIConfig.SPI_CPOL<<SPI_CR1_CPOL);

	temp |= (pSPIHandle->SPIConfig.SPI_DFF<<SPI_CR1_DFF);

	temp |= (pSPIHandle->SPIConfig.SPI_SSM<<SPI_CR1_SSM);

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

void SPI_Enable(SPI_RegDef_t *pSPIx){
	pSPIx->CR1 |= (1<<SPI_CR1_SPE);
}

void SPI_Disable(SPI_RegDef_t *pSPIx){
	pSPIx->CR1 &= ~(1<<SPI_CR1_SPE);
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t enable){
	if(enable){
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t enable){
	if(enable){
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t flag){
	{
	    if(pSPIx->SR & (1 << flag))
	    {
	        return FLAG_SET;
	    }
	    return FLAG_RESET;
	}
}

/*
 * Data send and receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuff, uint32_t TxLen){

	uint8_t dff = (pSPIx->CR1 & (1 << SPI_CR1_DFF)) ;

	while (TxLen>0){
		while(!(pSPIx->SR & (1 << 1))); // wait till TXE is set

		// check the DFF bit in SPI_CR1
		if(pSPIx->CR1 &(1<<SPI_CR1_DFF)){
			pSPIx->DR = *((uint16_t*)pTxBuff);
			TxLen--;
			TxLen--;
			(uint16_t*)pTxBuff++;
		}else{
			pSPIx->DR = *pTxBuff;
			TxLen--;
			pTxBuff++;
		}

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
