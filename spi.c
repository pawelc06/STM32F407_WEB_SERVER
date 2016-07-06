/*
 * spi.c
 *
 *  Created on: 18 cze 2016
 *      Author: Pawe�
 */

/**
 * @file spiL1.cpp
 * @brief STM32L1 SPI class.
 * @date January 2015
 * @author André Heßling
 *
 * Class for communicating with other SPI devices using the STM32L1 controller.
 */


#include "stm32f4xx.h"
#include "spi.h"
/** @addtogroup SPI
 * @{
 */
//#include <spiL1.h>
  SPI_TypeDef* _spi = SPI2; ///< SPI interface
  GPIO_TypeDef* _csGPIO = GPIOC; ///< GPIO port of /CS pin
  volatile uint16_t _csPin=0; ///< GPIO pin of /CS
  volatile uint8_t isInit;
  volatile uint8_t _bits=8;






/**
 * Deinit the SPI peripheral.
 */
void spi_deinit()
{
  // fixme: check if transfer is pending
  SPI_I2S_DeInit(_spi);

  isInit = 0;
}

/**
 * SPI read/write.
 *
 * Depending on the number of bits per transfer unit,
 * 8 or 16 bits are read/written.
 *
 * @param data Data to be sent
 * @return Data received
 */
uint16_t spi_transfer(uint16_t data)
{

	if (_bits == 8)
	    data &= 0xff;

	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
	SPI_I2S_SendData(SPI2, data);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
	data = SPI_I2S_ReceiveData(SPI2);



	// truncate data if necessary
	  if (_bits == 8)
	    return data & 0xff;
	  else
	    return data;


}









/**
 * Select the device.
 *
 * Can be used if only one device shall be addressed.
 *
 * @note Can only be used if configureCS() has been called before.
 */
void spi_select()
{
  if (_csGPIO != 0)

    _csGPIO->BSRRH = _csPin;
	//  _csGPIO->BSRR |= _csPin;
}

/**
 * Release the device.
 *
 * Can be used if only one device shall be addressed.
 *
 * @note Can only be used if configureCS() has been called before.
 */
void spi_unselect()
{
  if (_csGPIO != 0)
    _csGPIO->BSRRL = _csPin;
	//  _csGPIO->BSRR &= ~_csPin;
}



void mySPI_Init(void){


RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

SPI_InitTypeDef SPI_InitTypeDefStruct;

/*
SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  	  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  	  SPI_InitStructure.SPI_DataSize = 8;
  	  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  	  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  	  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  	  SPI_InitStructure.SPI_BaudRatePrescaler = 0;
  	  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  	  SPI_InitStructure.SPI_CRCPolynomial = 7;

*/

SPI_InitTypeDefStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
SPI_InitTypeDefStruct.SPI_Mode = SPI_Mode_Master;
SPI_InitTypeDefStruct.SPI_DataSize = SPI_DataSize_8b;
SPI_InitTypeDefStruct.SPI_CPOL = SPI_CPOL_Low;
SPI_InitTypeDefStruct.SPI_CPHA = SPI_CPHA_1Edge;
SPI_InitTypeDefStruct.SPI_NSS = SPI_NSS_Soft;
//SPI_InitTypeDefStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; //SCLK=5,23 MHz
SPI_InitTypeDefStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; //SCLK=10,46 MHz
SPI_InitTypeDefStruct.SPI_FirstBit = SPI_FirstBit_MSB;

SPI_Init(SPI2, &SPI_InitTypeDefStruct);


RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC , ENABLE);

GPIO_InitTypeDef GPIO_InitTypeDefStruct;

GPIO_InitTypeDefStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
GPIO_InitTypeDefStruct.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitTypeDefStruct.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitTypeDefStruct.GPIO_OType = GPIO_OType_PP;
GPIO_InitTypeDefStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
GPIO_Init(GPIOC, &GPIO_InitTypeDefStruct);

GPIO_InitTypeDefStruct.GPIO_Pin = GPIO_Pin_10;
GPIO_InitTypeDefStruct.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitTypeDefStruct.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitTypeDefStruct.GPIO_OType = GPIO_OType_PP;
GPIO_InitTypeDefStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
GPIO_Init(GPIOB, &GPIO_InitTypeDefStruct);

/** NSS/CS SPI pin */
GPIO_InitTypeDefStruct.GPIO_Pin = GPIO_Pin_0;
GPIO_InitTypeDefStruct.GPIO_Mode = GPIO_Mode_OUT;
GPIO_InitTypeDefStruct.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitTypeDefStruct.GPIO_PuPd = GPIO_PuPd_UP;
GPIO_InitTypeDefStruct.GPIO_OType = GPIO_OType_PP;
GPIO_Init(GPIOC, &GPIO_InitTypeDefStruct);



GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_SPI2);
GPIO_PinAFConfig(GPIOC, GPIO_PinSource2, GPIO_AF_SPI2);
GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF_SPI2);

GPIO_SetBits(GPIOC, GPIO_Pin_0);


SPI_Cmd(SPI2, ENABLE);

}


