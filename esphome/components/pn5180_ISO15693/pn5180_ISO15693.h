// NAME: PN5180ISO15693.h
//
// DESC: ISO15693 protocol on NXP Semiconductors PN5180 module for Arduino.
//
// Copyright (c) 2018 by Andreas Trappmann. All rights reserved.
//
// This file is part of the PN5180 library for the Arduino environment.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//
#ifndef pn5180_ISO15693_H
#define pn5180_ISO15693_H

#include "esphome/components/pn5180_ISO15693/pn5180.h"

enum ISO15693ErrorCode {
  EC_NO_CARD = -1,
  ISO15693_EC_OK = 0,
  ISO15693_EC_NOT_SUPPORTED = 0x01,
  ISO15693_EC_NOT_RECOGNIZED = 0x02,
  ISO15693_EC_OPTION_NOT_SUPPORTED = 0x03,
  ISO15693_EC_UNKNOWN_ERROR = 0x0f,
  ISO15693_EC_BLOCK_NOT_AVAILABLE = 0x10,
  ISO15693_EC_BLOCK_ALREADY_LOCKED = 0x11,
  ISO15693_EC_BLOCK_IS_LOCKED = 0x12,
  ISO15693_EC_BLOCK_NOT_PROGRAMMED = 0x13,
  ISO15693_EC_BLOCK_NOT_LOCKED = 0x14,
  ISO15693_EC_CUSTOM_CMD_ERROR = 0xA0
};

namespace esphome {
namespace pn5180_ISO15693 {

class PN5180ISO15693 : public pn5180::PN5180 {
 public:
  // ctor shouldn't be needed
  // PN5180ISO15693(uint8_t SSpin, uint8_t BUSYpin, uint8_t RSTpin, SPIClass& spi=SPI);
  void setup() override;

  void dump_config() override;

  void update() override;
  float get_setup_priority() const override;

  void loop() override;
  // TODO: map to PN5180::powerdown()
  void on_shutdown() override {}

 protected:
  ISO15693ErrorCode issueISO15693Command(uint8_t *cmd, uint8_t cmdLen, uint8_t **resultPtr);
  ISO15693ErrorCode inventoryPoll(uint8_t *uid, uint8_t maxTags, uint8_t *numCard, uint8_t *numCol,
                                  uint16_t *collision);

 public:
  ISO15693ErrorCode getInventory(uint8_t *uid);
  ISO15693ErrorCode getInventoryMultiple(uint8_t *uid, uint8_t maxTags, uint8_t *numCard);

  ISO15693ErrorCode readSingleBlock(uint8_t *uid, uint8_t blockNo, uint8_t *blockData, uint8_t blockSize);
  ISO15693ErrorCode writeSingleBlock(uint8_t *uid, uint8_t blockNo, uint8_t *blockData, uint8_t blockSize);
  ISO15693ErrorCode readMultipleBlock(uint8_t *uid, uint8_t blockNo, uint8_t numBlock, uint8_t *blockData,
                                      uint8_t blockSize);

  ISO15693ErrorCode getSystemInfo(uint8_t *uid, uint8_t *blockSize, uint8_t *numBlocks);

  // ICODE SLIX2 specific commands, see https://www.nxp.com/docs/en/data-sheet/SL2S2602.pdf
  ISO15693ErrorCode getRandomNumber(uint8_t *randomData);
  ISO15693ErrorCode setPassword(uint8_t identifier, uint8_t *password, uint8_t *random);
  ISO15693ErrorCode enablePrivacy(uint8_t *password, uint8_t *random);
  // helpers
  ISO15693ErrorCode enablePrivacyMode(uint8_t *password);
  ISO15693ErrorCode disablePrivacyMode(uint8_t *password);
  /*
   * Helper functions
   */
 public:
  bool setupRF();
  const char *strerror(ISO15693ErrorCode errno);

 protected:
  // TODO: I don't think this is needed since they come from the underlying spi component
  //  GPIOPin *cs_{nullptr};  // same as NSS
  GPIOPin *rst_pin_{nullptr};
  // GPIOPin *mosi_{nullptr};
  // GPIOPin *miso_{nullptr};
  // GPIOPin *sck_{nullptr};
  GPIOPin *bsy_pin_{nullptr};
};

#endif /* PN5180ISO15693_H */
}  // namespace pn5180_ISO15693
}  // namespace esphome