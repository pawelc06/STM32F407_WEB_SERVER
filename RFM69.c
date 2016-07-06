/*
 * RFM69.c
 *
 *  Created on: 10 cze 2016
 *      Author: Pawe³
 */
#include "rfm69registers.h"
#include <stdbool.h>
#include "rfm69.h"
#include "systimer.h"
#include <stdio.h>
#include <string.h>

//#define DEBUG 1


GPIO_TypeDef* _csGPIO;
  uint16_t _csPin;
  GPIO_TypeDef* _resetGPIO;
  uint16_t _resetPin;
  GPIO_TypeDef* _dataGPIO;
  uint16_t _dataPin;
  bool _init2;
  volatile RFM69Mode _mode;
  bool _highPowerDevice;
  uint8_t _powerLevel;
  int _rssi;
  bool _autoReadRSSI;
  bool _ookEnabled;
  RFM69DataMode _dataMode;
  bool _highPowerSettings;
  bool _csmaEnabled;
  volatile char _rxBuffer[RFM69_MAX_PAYLOAD];
  volatile unsigned int _rxBufferLength;


/**
 * Read a RFM69 register value.
 *
 * @param reg The register to be read
 * @return The value of the register
 */



uint8_t readRegister(uint8_t reg)
{
  // sanity check
  if (reg > 0x7f)
    return 0;

  // read value from register
  chipSelect();

  spi_transfer(reg);
  uint8_t value = spi_transfer(0x00);

  chipUnselect();

  return value;
}

/**
 * Write a RFM69 register value.
 *
 * @param reg The register to be written
 * @param value The value of the register to be set
 */
void writeRegister(uint8_t reg, uint8_t value)
{
  // sanity check
  if (reg > 0x7f)
    return;

  // transfer value to register and set the write flag
  chipSelect();

  spi_transfer(reg | 0x80);
  spi_transfer(value);

  chipUnselect();
}

/**
 * Acquire the chip.
 */
void RFM69_chipSelect()
{
  GPIO_ResetBits(_csGPIO, _csPin);
}


void setPASettings(uint8_t forcePA,bool highPowerDevice)
{
  // disable OCP for high power devices, enable otherwise
  writeRegister(0x13, 0x0A | (highPowerDevice ? 0x00 : 0x10));

  if (0 == forcePA)
  {
    if (true == highPowerDevice)
    {
      // enable PA1 only
      writeRegister(0x11, (readRegister(0x11) & 0x1F) | 0x40);
    }
    else
    {
      // enable PA0 only
      writeRegister(0x11, (readRegister(0x11) & 0x1F) | 0x80);
    }
  }
  else
  {
    // PA settings forced
    uint8_t pa = 0;

    if (forcePA & 0x01)
      pa |= 0x80;

    if (forcePA & 0x02)
      pa |= 0x40;

    if (forcePA & 0x04)
      pa |= 0x20;

    // check if high power settings are forced
    //_highPowerSettings = (forcePA & 0x08) ? true : false;
    //setHighPowerSettings(_highPowerSettings);

    writeRegister(0x11, (readRegister(0x11) & 0x1F) | pa);
  }
}

void clearFIFO()
{
  // clear flags and FIFO
  writeRegister(0x28, 0x10);
}

void RFM69_init(uint8_t freqBand,uint8_t networkID){

	 //_spi = SPI2;
	  _csGPIO = GPIOC;
	  _csPin = GPIO_Pin_0;
	  _resetGPIO = 0;
	  _resetPin = 0;
	  _init2 = 0;
	  _mode = RFM69_MODE_STANDBY;
	  _highPowerDevice = false;
	  _powerLevel = 0;
	  _rssi = -127;
	  _ookEnabled = 0;
	  _autoReadRSSI = 0;
	  _dataMode = RFM69_DATA_MODE_PACKET;
	  _dataGPIO = 0;
	  _dataPin = 0;
	  _highPowerSettings = 0;
	  _csmaEnabled = 0;
	  _rxBufferLength = 0;

	const uint8_t CONFIG[][2] =
	  {
	    /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
	    /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 }, // no shaping
	    /* 0x03 */ { REG_BITRATEMSB, RF_BITRATEMSB_55555}, // default: 4.8 KBPS
	    /* 0x04 */ { REG_BITRATELSB, RF_BITRATELSB_55555},
	    /* 0x05 */ { REG_FDEVMSB, RF_FDEVMSB_50000}, // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
	    /* 0x06 */ { REG_FDEVLSB, RF_FDEVLSB_50000},

	    /* 0x07 */ { REG_FRFMSB, (uint8_t) (freqBand==RF69_315MHZ ? RF_FRFMSB_315 : (freqBand==RF69_433MHZ ? RF_FRFMSB_433 : (freqBand==RF69_868MHZ ? RF_FRFMSB_868 : RF_FRFMSB_915))) },
	    /* 0x08 */ { REG_FRFMID, (uint8_t) (freqBand==RF69_315MHZ ? RF_FRFMID_315 : (freqBand==RF69_433MHZ ? RF_FRFMID_433 : (freqBand==RF69_868MHZ ? RF_FRFMID_868 : RF_FRFMID_915))) },
	    /* 0x09 */ { REG_FRFLSB, (uint8_t) (freqBand==RF69_315MHZ ? RF_FRFLSB_315 : (freqBand==RF69_433MHZ ? RF_FRFLSB_433 : (freqBand==RF69_868MHZ ? RF_FRFLSB_868 : RF_FRFLSB_915))) },

	    // looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
	    // +17dBm and +20dBm are possible on RFM69HW
	    // +13dBm formula: Pout = -18 + OutputPower (with PA0 or PA1**)
	    // +17dBm formula: Pout = -14 + OutputPower (with PA1 and PA2)**
	    // +20dBm formula: Pout = -11 + OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
	    ///* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
	    ///* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, // over current protection (default is 95mA)

	    // RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4KHz)
	    /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2 }, // (BitRate < 2 * RxBw)
	    //for BR-19200: /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_3 },
	    /* 0x25 */ { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, // DIO0 is the only IRQ we're using
	    /* 0x26 */ { REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF }, // DIO5 ClkOut disable for power saving
	    /* 0x28 */ { REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // writing to this bit ensures that the FIFO & status flags are reset
	    /* 0x29 */ { REG_RSSITHRESH, 220 }, // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
	    ///* 0x2D */ { REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE } // default 3 preamble bytes 0xAAAAAA
	    /* 0x2E */ { REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0 },
	    /* 0x2F */ { REG_SYNCVALUE1, 0x2D },      // attempt to make this compatible with sync1 byte of RFM12B lib
	    /* 0x30 */ { REG_SYNCVALUE2, networkID }, // NETWORK ID
	    /* 0x37 */ { REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF },
	    /* 0x38 */ { REG_PAYLOADLENGTH, 66 }, // in variable length mode: the max frame size, not used in TX
	    ///* 0x39 */ { REG_NODEADRS, nodeID }, // turned off because we're not using address filtering
	    /* 0x3C */ { REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE }, // TX on FIFO not empty
	    /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
	    //for BR-19200: /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
	    /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
	    {255, 0}
	  };

	  for (uint8_t i = 0; CONFIG[i][0] != 255; i++)
		  writeRegister(CONFIG[i][0], CONFIG[i][1]);

	  // set PA and OCP settings according to RF module (normal/high power)
	    setPASettings(0,false);

	    // clear FIFO and flags
	    clearFIFO();

#ifdef USE_IRQ
	    ExtInt_Config();
#endif
}

/**
   * Enable/disable the CSMA/CA (carrier sense) algorithm before sending a packet.
   *
   * @param enable true or false
   */
  void RFM69_setCSMA(bool enable)
  {
    _csmaEnabled = enable;
  }

void RFM69_reset()
{
  if (0 == _resetGPIO)
    return;

  _init2 = false;

  // generate reset impulse
  GPIO_SetBits(_resetGPIO, _resetPin);
  delay_ms(1);
  GPIO_ResetBits(_resetGPIO, _resetPin);

  // wait until module is ready
  delay_ms(10);

  _mode = RFM69_MODE_STANDBY;
}

bool RFM69_setAESEncryption(const void* aesKey, unsigned int keyLength)
{
  bool enable = false;

  // check if encryption shall be enabled or disabled
  if ((0 != aesKey) && (16 == keyLength))
    enable = true;

  // switch to standby
  setMode(RFM69_MODE_STANDBY);

  if (true == enable)
  {
    // transfer AES key to AES key register
    chipSelect();

    // address first AES MSB register
    spi_transfer(0x3E | 0x80);

    // transfer key (0x3E..0x4D)
    for (unsigned int i = 0; i < keyLength; i++)
      spi_transfer(((uint8_t*)aesKey)[i]);

    chipUnselect();
  }

  // set/reset AesOn Bit in packet config
  writeRegister(0x3D, (readRegister(0x3D) & 0xFE) | (enable ? 1 : 0));

  return enable;
}

RFM69Mode setMode(RFM69Mode mode)
{
  if ((mode == _mode) || (mode > RFM69_MODE_RX))
    return _mode;

  // set new mode
  writeRegister(0x01, mode << 2);

  // set special registers if this is a high power device (RFM69HW)
  if (true == _highPowerDevice)
  {
    switch (mode)
    {
    case RFM69_MODE_RX:
      // normal RX mode
      if (true == _highPowerSettings)
        setHighPowerSettings(false);
      break;

    case RFM69_MODE_TX:
      // +20dBm operation on PA_BOOST
      if (true == _highPowerSettings)
        setHighPowerSettings(true);
      break;

    default:
      break;
    }
  }

  _mode = mode;

  return _mode;
}

/**
 * Enable the +20 dBm high power settings of RFM69Hxx modules.
 *
 * @note Enabling only works with high power devices.
 *
 * @param enable true or false
 */
void setHighPowerSettings(bool enable)
{
  // enabling only works if this is a high power device
  if (true == enable && false == _highPowerDevice)
    enable = false;

  writeRegister(0x5A, enable ? 0x5D : 0x55);
  writeRegister(0x5C, enable ? 0x7C : 0x70);
}

void RFM69_sleep()
{
  setMode(RFM69_MODE_SLEEP);
}

/**
 * Set the output power level in dBm.
 *
 * This function takes care of the different PA settings of the modules.
 * Depending on the requested power output setting and the available module,
 * PA0, PA1 or PA1+PA2 is enabled.
 *
 * @param dBm Output power in dBm
 * @return 0 if dBm valid; else -1.
 */
int RFM69_setPowerDBm(int8_t dBm)
{
  /* Output power of module is from -18 dBm to +13 dBm
   * in "low" power devices, -2 dBm to +20 dBm in high power devices */
  if (dBm < -18 || dBm > 20)
    return -1;

  if (false == _highPowerDevice && dBm > 13)
    return -1;

  if (true == _highPowerDevice && dBm < -2)
    return -1;

  uint8_t powerLevel = 0;

  if (false == _highPowerDevice)
  {
    // only PA0 can be used
    powerLevel = dBm + 18;

    // enable PA0 only
    writeRegister(0x11, 0x80 | powerLevel);
  }
  else
  {
    if (dBm >= -2 && dBm <= 13)
    {
      // use PA1 on pin PA_BOOST
      powerLevel = dBm + 18;

      // enable PA1 only
      writeRegister(0x11, 0x40 | powerLevel);

      // disable high power settings
      _highPowerSettings = false;
      setHighPowerSettings(_highPowerSettings);
    }
    else if (dBm > 13 && dBm <= 17)
    {
      // use PA1 and PA2 combined on pin PA_BOOST
      powerLevel = dBm + 14;

      // enable PA1+PA2
      writeRegister(0x11, 0x60 | powerLevel);

      // disable high power settings
      _highPowerSettings = false;
      setHighPowerSettings(_highPowerSettings);
    }
    else
    {
      // output power from 18 dBm to 20 dBm, use PA1+PA2 with high power settings
      powerLevel = dBm + 11;

      // enable PA1+PA2
      writeRegister(0x11, 0x60 | powerLevel);

      // enable high power settings
      _highPowerSettings = true;
      setHighPowerSettings(_highPowerSettings);
    }
  }

  return 0;
}


/**
 * Put the RFM69 module in RX mode and try to receive a packet.
 *
 * @note The module resides in RX mode.
 *
 * @param data Pointer to a receiving buffer
 * @param dataLength Maximum size of buffer
 * @return Number of received bytes; 0 if no payload is available.
 */
int RFM69_receive(char* data, unsigned int dataLength)
{
  // check if there is a packet in the internal buffer and copy it
  if (_rxBufferLength > 0)
  {
    // copy only until dataLength, even if packet in local buffer is actually larger
    memcpy(data, _rxBuffer, dataLength);

    unsigned int bytesRead = _rxBufferLength;

    // empty local buffer
    _rxBufferLength = 0;

    return bytesRead;
  }
  else
  {
    // regular receive
    return _receive(data, dataLength);
  }
}

int RFM69_receive_non_block(char* data, uint8_t dataLength){


	unsigned int bytesRead = _rxBufferLength;

	// go to RX mode if not already in this mode
	  if (RFM69_MODE_RX != _mode)
	  {
	    setMode(RFM69_MODE_RX);
	    waitForModeReady();
	  }

	  // copy only until dataLength, even if packet in local buffer is actually larger
	if(bytesRead >= dataLength){
			memcpy(data, _rxBuffer, dataLength);
	}



	    // empty local buffer
	    _rxBufferLength = 0;

	    return bytesRead;
}

/**
 * Wait until the requested mode is available or timeout.
 */
void waitForModeReady()
{
  uint32_t timeEntry = mstimer_get();

  while (((readRegister(0x27) & 0x80) == 0) && ((mstimer_get() - timeEntry) < TIMEOUT_MODE_READY));
}

int readRSSI()
{
  _rssi = -readRegister(0x24) / 2;

  return _rssi;
}


/**
 * Put the RFM69 module in RX mode and try to receive a packet.
 *
 * @note This is an internal function.
 * @note The module resides in RX mode.
 *
 * @param data Pointer to a receiving buffer
 * @param dataLength Maximum size of buffer
 * @return Number of received bytes; 0 if no payload is available.
 */
int _receive(char* data, unsigned int dataLength)
{
  // go to RX mode if not already in this mode
  if (RFM69_MODE_RX != _mode)
  {
    setMode(RFM69_MODE_RX);
    waitForModeReady();
  }

  // check for flag PayloadReady
  if (readRegister(0x28) & 0x04)
  {
    // go to standby before reading data
    setMode(RFM69_MODE_STANDBY);

    // get FIFO content
    unsigned int bytesRead = 0;

    // read until FIFO is empty or buffer length exceeded
    while ((readRegister(0x28) & 0x40) && (bytesRead < dataLength))
    {
      // read next byte
      data[bytesRead] = readRegister(0x00);
      bytesRead++;
    }

    // automatically read RSSI if requested
    if (true == _autoReadRSSI)
    {
      readRSSI();
    }

    // go back to RX mode
    setMode(RFM69_MODE_RX);
    // todo: wait needed?
    //		waitForModeReady();

    return bytesRead;
  }
  else
    return 0;
}

/**
 * Debug function to dump all RFM69 registers.
 *
 * Symbol 'DEBUG' has to be defined.
 */
void RFM69_dumpRegisters(void)
{

  for (unsigned int i = 1; i <= 0x71; i++)
  {
    printf("[0x%X]: 0x%X\n", i, readRegister(i));
  }




}

void chipSelect()
{
  GPIO_ResetBits(_csGPIO, _csPin);
}

void chipUnselect()
{
  GPIO_SetBits(_csGPIO, _csPin);
}

void ExtInt_Config(){
	EXTI_InitTypeDef   EXTI_InitStructure;
	  GPIO_InitTypeDef   GPIO_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;

	  /* Enable GPIOA clock */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	  /* Enable SYSCFG clock */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	  /* Configure PA3 pin as input floating */
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  /* Connect EXTI Line0 to PA0 pin */
	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource2);

	  /* Configure EXTI Line0 */
	  EXTI_InitStructure.EXTI_Line = EXTI_Line3;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);

	  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
}

void EXTI3_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line3) != RESET)
  {
    /* Toggle LED1 */


	GPIO_SetBits(GPIOB,GPIO_Pin_5);


	  // go to standby before reading data
	      setMode(RFM69_MODE_STANDBY);

	      // get FIFO content
	      unsigned int bytesRead = 0;

	      // read until FIFO is empty or buffer length exceeded
	      while ((readRegister(0x28) & 0x40) && (bytesRead < RFM69_MAX_PAYLOAD))
	      {
	        // read next byte
	    	_rxBuffer[bytesRead] = readRegister(0x00);
	        bytesRead++;
	      }
	      _rxBuffer[bytesRead] = 0;

	      _rxBufferLength = bytesRead;

	      // go back to RX mode
	      setMode(RFM69_MODE_RX);

	  /*******************/


    /* Clear the  EXTI line 0 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line3);
    GPIO_ResetBits(GPIOB,GPIO_Pin_5);
  }
}

