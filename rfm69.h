/**
 * @file rfm69.hpp
 *
 * @brief RFM69 and RFM69HW library for sending and receiving packets in connection with a STM32 controller.
 * @date January, February 2015
 * @author André Heßling
 *
 * This is a protocol agnostic driver library for handling HopeRF's RFM69 433/868/915 MHz RF modules.
 * Support is also available for the +20 dBm high power modules called RFM69HW/RFM69HCW.
 *
 * A CSMA/CA (carrier sense multiple access) algorithm can be enabled to avoid collisions.
 * If you want to enable CSMA, you should initialize the random number generator before.
 *
 * This library is written for the STM32 family of controllers, but can easily be ported to other devices.
 *
 * You have to provide your own functions for delay_ms and mstimer_get.
 * Use the SysTick timer (for example) with a 1 ms resolution which is present on all ARM controllers.
 *
 * If you want to port this library to other devices, you have to provide an SPI instance
 * derived from the SPIBase class.
 */

#include <stdint.h>
#include "stm32f4xx_gpio.h"
#include "rfm69registers.h"
#include "spi.h"

#define RESET_PIN GPIO_Pin_5
#define RESET_PORT GPIOA


#define RF69_315MHZ            31 // non trivial values to avoid misconfiguration
#define RF69_433MHZ            43
#define RF69_868MHZ            86
#define RF69_915MHZ            91

#define TIMEOUT_MODE_READY    100 ///< Maximum amount of time until mode switch [ms]
#define TIMEOUT_PACKET_SENT   100 ///< Maximum amount of time until packet must be sent [ms]
#define TIMEOUT_CSMA_READY    500 ///< Maximum CSMA wait time for channel free detection [ms]
#define CSMA_RSSI_THRESHOLD   -85 ///< If RSSI value is smaller than this, consider channel as free [dBm]

#define USE_IRQ 1

/** @addtogroup RFM69
 * @{
 */
#define RFM69_MAX_PAYLOAD		64 ///< Maximum bytes payload

/**
 * Valid RFM69 operation modes.
 */
typedef enum
{
  RFM69_MODE_SLEEP = 0,//!< Sleep mode (lowest power consumption)
  RFM69_MODE_STANDBY,  //!< Standby mode
  RFM69_MODE_FS,       //!< Frequency synthesizer enabled
  RFM69_MODE_TX,       //!< TX mode (carrier active)
  RFM69_MODE_RX        //!< RX mode
} RFM69Mode;

/**
 * Valid RFM69 data modes.
 */
typedef enum
{
  RFM69_DATA_MODE_PACKET = 0,                 //!< Packet engine active
  RFM69_DATA_MODE_CONTINUOUS_WITH_SYNC = 2,   //!< Continuous mode with clock recovery
  RFM69_DATA_MODE_CONTINUOUS_WITHOUT_SYNC = 3,//!< Continuous mode without clock recovery
} RFM69DataMode;


  void RFM69_reset();

void RFM69_init(uint8_t freqBand,uint8_t networkID);
int RFM69_receive_non_block(char* data, uint8_t dataLength);


  void setFrequency(unsigned int frequency);

#include <stdbool.h>

  void setFrequencyDeviation(unsigned int frequency);

  void setBitrate(unsigned int bitrate);

  RFM69Mode setMode(RFM69Mode mode);

  void setPowerLevel(uint8_t power);

  int RFM69_setPowerDBm(int8_t dBm);

  void setHighPowerSettings(bool enable);

  void setCustomConfig(const uint8_t config[][2], unsigned int length);

  int send(const void* data, unsigned int dataLength);

  int RFM69_receive(char* data, unsigned int dataLength);

  void RFM69_sleep();

  /**
   * Gets the last "cached" RSSI reading.
   *
   * @note This only gets the latest reading that was requested by readRSSI().
   *
   * @return RSSI value in dBm.
   */
  int getRSSI();

  void setOOKMode(bool enable);

  void setDataMode(RFM69DataMode dataMode);

  /**
   * Enable/disable the automatic reading of the RSSI value during packet reception.
   *
   * Default is off (no reading).
   *
   * @param enable true or false
   */
  void setAutoReadRSSI(bool enable);


  /**
   * Enable/disable the CSMA/CA (carrier sense) algorithm before sending a packet.
   *
   * @param enable true or false
   */
  void RFM69_setCSMA(bool enable);


  void continuousBit(bool bit);

  void RFM69_dumpRegisters();

  void setPASettings(uint8_t forcePA,bool highPowerDevice);
  
  bool RFM69_setAESEncryption(const void* aesKey, unsigned int keyLength);


  uint8_t readRegister(uint8_t reg);

  void writeRegister(uint8_t reg, uint8_t value);

  void chipSelect();

  void chipUnselect();

  void RFM69_clearFIFO();

  void waitForModeReady();

  void waitForPacketSent();

  int readRSSI();

  bool channelFree();

  int _receive(char* data, unsigned int dataLength);

  bool initialize(uint8_t freqBand, uint8_t nodeID, uint8_t networkID);

  void ExtInt_Config();


  /** @}
   *
   */




/** @}
 *
 */
