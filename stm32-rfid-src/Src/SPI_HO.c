/*
 * SPI_HO.c
 *
 *  Created on: Mar 21, 2024
 *      Author: natha
 */


#include "SPI_HO.h"
#include "stm32u5xx.h"

#include "stdint.h"
#include "stdio.h"

/**
 * @brief Initializes the SPI interface for communication with RC522
 * SPI has to be reinitialized as HAL is not giving the 'full' initialization
 */
void SPI_Init(void)
{
	#define AF5 0x05
	// Enable the clock for GPIOA and SPI1
	RCC->AHB1ENR|=RCC_AHB2ENR1_GPIOAEN;
	RCC->APB2ENR|=RCC_APB2ENR_SPI1EN;

	// Configure PA5, PA6, and PA7 as Alternate Function for SPI1
	GPIOA->MODER|=GPIO_MODER_MODE5_1|GPIO_MODER_MODE6_1|GPIO_MODER_MODE7_1;
	GPIOA->MODER&=~(GPIO_MODER_MODE5_0|GPIO_MODER_MODE6_0|GPIO_MODER_MODE7_0);

	// Set high speed for PA5, PA6, and PA7
	GPIOA->OSPEEDR|=GPIO_OSPEEDR_OSPEED5|GPIO_OSPEEDR_OSPEED6|GPIO_OSPEEDR_OSPEED7;

	// Set Alternate Function 5 (AF5) for PA5, PA6, and PA7
	GPIOA->AFR[0]|=(AF5<<20)|(AF5<<24)|(AF5<<28);

	// Reset and configure SPI1 control registers
	SPI1->CR2=0;
	SPI1->CR1=SPI_CR1_SSI|SPI_CR1_SPE;
	SPI1->CR1=SPI_CR1_SSI|SPI_CR1_SPE;

	// Set SPI1 clock prescaler to divide by 8
	SPI1->CFG1=SPI_CFG1_MBR_2;
	// Configure SPI1 as master and enable software slave management
	SPI1->CFG2=SPI_CFG2_SSM|SPI_CFG2_MASTER;

}


