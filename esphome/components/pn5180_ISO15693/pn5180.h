// based on https://github.com/tueddy/PN5180-Library

#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/nfc/nfc_tag.h"
#include "esphome/components/nfc/nfc.h"
#include "esphome/components/nfc/automation.h"
#include "esphome/components/spi/spi.h"

#include <cinttypes>
#include <vector>

namespace esphome {
namespace pn5180 {

// PN5180 Registers
static const uint8_t SYSTEM_CONFIG = 0x00;
static const uint8_t IRQ_ENABLE = 0x01;
static const uint8_t IRQ_STATUS = 0x02;
static const uint8_t IRQ_CLEAR = 0x03;
static const uint8_t TRANSCEIVE_CONTROL = 0x04;
static const uint8_t TIMER1_RELOAD = 0x0c;
static const uint8_t TIMER1_CONFIG = 0x0f;
static const uint8_t RX_WAIT_CONFIG = 0x11;
static const uint8_t CRC_RX_CONFIG = 0x12;
static const uint8_t RX_STATUS = 0x13;
static const uint8_t TX_WAIT_CONFIG = 0x17;
static const uint8_t TX_CONFIG = 0x18;
static const uint8_t CRC_TX_CONFIG = 0x19;
static const uint8_t RF_STATUS = 0x1d;
static const uint8_t SYSTEM_STATUS = 0x24;
static const uint8_t TEMP_CONTROL = 0x25;
static const uint8_t AGC_REF_CONFIG = 0x26;

// PN5180 EEPROM Addresses
static const uint8_t DIE_IDENTIFIER = 0x00;
static const uint8_t PRODUCT_VERSION = 0x10;
static const uint8_t FIRMWARE_VERSION = 0x12;
static const uint8_t EEPROM_VERSION = 0x14;
static const uint8_t IRQ_PIN_CONFIG = 0x1A;

enum PN5180TransceiveState {
  PN5180_TS_Idle = 0,
  PN5180_TS_WaitTransmit = 1,
  PN5180_TS_Transmitting = 2,
  PN5180_TS_WaitReceive = 3,
  PN5180_TS_WaitForData = 4,
  PN5180_TS_Receiving = 5,
  PN5180_TS_LoopBack = 6,
  PN5180_TS_RESERVED = 7
};

// PN5180 IRQ_STATUS
static const uint32_t RX_IRQ_STAT = 1U << 0;              // End of RF receiption IRQ
static const uint32_t TX_IRQ_STAT = 1U << 1;              // End of RF transmission IRQ
static const uint32_t IDLE_IRQ_STAT = 1U << 2;            // IDLE IRQ
static const uint32_t RFOFF_DET_IRQ_STAT = 1U << 6;       // RF Field OFF detection IRQ
static const uint32_t RFON_DET_IRQ_STAT = 1U << 7;        // RF Field ON detection IRQ
static const uint32_t TX_RFOFF_IRQ_STAT = 1U << 8;        // RF Field OFF in PCD IRQ
static const uint32_t TX_RFON_IRQ_STAT = 1U << 9;         // RF Field ON in PCD IRQ
static const uint32_t RX_SOF_DET_IRQ_STAT = 1U << 14;     // RF SOF Detection IRQ
static const uint32_t GENERAL_ERROR_IRQ_STAT = 1U << 17;  // General error IRQ
static const uint32_t LPCD_IRQ_STAT = 1U << 19;           // LPCD Detection IRQ

static const uint8_t MIFARE_CLASSIC_KEYA = 0x60;  // Mifare Classic key A
static const uint8_t MIFARE_CLASSIC_KEYB = 0x61;  // Mifare Classic key B

/*
 * 11.4.1 Physical Host Interface
 * The interface of the PN5180 to a host microcontroller is based on a SPI interface,
 * extended by signal line BUSY. The maximum SPI speed is 7 Mbps and fixed to CPOL
 * = 0 and CPHA = 0.
 */
// Settings for PN5180: 7Mbps, MSB first, SPI_MODE0 (CPOL=0, CPHA=0)
class PN5180 : public PollingComponent,
               public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST,
                                     spi::CLOCK_POLARITY_LOW,   // CPOL=0
                                     spi::CLOCK_PHASE_LEADING,  // CPHA=0
                                     spi::DATA_RATE_7MHZ> {
 private:
  // SPISettings SPI_SETTINGS;
  static uint8_t readBufferStatic16[16];
  uint8_t *readBufferDynamic508 = NULL;

 public:
  void begin();
  void end();

  /*
   * PN5180 direct commands with host interface
   */
 public:
  /* cmd 0x00 */
  bool writeRegister(uint8_t reg, uint32_t value);
  /* cmd 0x01 */
  bool writeRegisterWithOrMask(uint8_t addr, uint32_t mask);
  /* cmd 0x02 */
  bool writeRegisterWithAndMask(uint8_t addr, uint32_t mask);

  /* cmd 0x04 */
  bool readRegister(uint8_t reg, uint32_t *value);

  /* cmd 0x06 */
  bool writeEEprom(uint8_t addr, uint8_t *buffer, uint8_t len);
  /* cmd 0x07 */
  bool readEEprom(uint8_t addr, uint8_t *buffer, int len);

  /* cmd 0x09 */
  bool sendData(uint8_t *data, int len, uint8_t validBits = 0);
  /* cmd 0x0a */
  uint8_t *readData(int len);
  bool readData(int len, uint8_t *buffer);
  /* prepare LPCD registers */
  bool prepareLPCD();
  /* cmd 0x0B */
  bool switchToLPCD(uint16_t wakeupCounterInMs);
  /* cmd 0x0C */
  int16_t mifareAuthenticate(uint8_t blockno, uint8_t *key, uint8_t keyType, uint8_t *uid);
  /* cmd 0x11 */
  bool loadRFConfig(uint8_t txConf, uint8_t rxConf);

  /* cmd 0x16 */
  bool setRF_on();
  /* cmd 0x17 */
  bool setRF_off();

  bool sendCommand(uint8_t *sendBuffer, size_t sendBufferLen, uint8_t *recvBuffer, size_t recvBufferLen);
  void set_reset_pin(GPIOPin *pin) { this->rst_pin_ = pin; }
  void set_busy_pin(GPIOPin *pin) { this->bsy_pin_ = pin; }
  /*
   * Helper functions
   */
 public:
  void reset();

  uint16_t commandTimeout = 500;
  uint32_t getIRQStatus();
  bool clearIRQStatus(uint32_t irqMask);

  PN5180TransceiveState getTransceiveState();

 protected:
  // TODO: I don't think this is needed since they come from the underlying spi component
  //  GPIOPin *cs_{nullptr};  // same as NSS
  GPIOPin *rst_pin_{nullptr};
  // GPIOPin *mosi_{nullptr};
  // GPIOPin *miso_{nullptr};
  // GPIOPin *sck_{nullptr};
  GPIOPin *bsy_pin_{nullptr};

  /*
   * Private methods, called within an SPI transaction
   */
  // esphome doesn't recommend using private
  //  private:
  bool transceiveCommand(uint8_t *sendBuffer, size_t sendBufferLen, uint8_t *recvBuffer = 0, size_t recvBufferLen = 0);


};

}  // namespace pn5180
}  // namespace esphome