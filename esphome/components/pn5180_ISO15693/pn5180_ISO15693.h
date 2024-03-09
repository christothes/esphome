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
#pragma once

#include "esphome/core/component.h"
#include "esphome/components/pn5180_ISO15693/pn5180.h"

#include <cinttypes>
#include <vector>

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

class PN5180ISO15693BinarySensor;

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
  bool powerdown();
  void on_shutdown() override { this->setRF_off(); }

  void register_tag(PN5180ISO15693BinarySensor *tag) { this->binary_sensors_.push_back(tag); }
  void register_ontag_trigger(nfc::NfcOnTagTrigger *trig) { this->triggers_ontag_.push_back(trig); }
  void register_ontagremoved_trigger(nfc::NfcOnTagTrigger *trig) { this->triggers_ontagremoved_.push_back(trig); }

  void add_on_finished_write_callback(std::function<void()> callback) {
    this->on_finished_write_callback_.add(std::move(callback));
  }

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

  std::unique_ptr<nfc::NfcTag> read_tag_(std::vector<uint8_t> &uid);
  // bool format_tag_(std::vector<uint8_t> &uid);
  // bool clean_tag_(std::vector<uint8_t> &uid);
  // bool write_tag_(std::vector<uint8_t> &uid, nfc::NdefMessage *message);

  // std::unique_ptr<nfc::NfcTag> read_mifare_classic_tag_(std::vector<uint8_t> &uid);
  // bool read_mifare_classic_block_(uint8_t block_num, std::vector<uint8_t> &data);
  // bool write_mifare_classic_block_(uint8_t block_num, std::vector<uint8_t> &data);
  // bool auth_mifare_classic_block_(std::vector<uint8_t> &uid, uint8_t block_num, uint8_t key_num, const uint8_t *key);
  // bool format_mifare_classic_mifare_(std::vector<uint8_t> &uid);
  // bool format_mifare_classic_ndef_(std::vector<uint8_t> &uid);
  // bool write_mifare_classic_tag_(std::vector<uint8_t> &uid, nfc::NdefMessage *message);

  // std::unique_ptr<nfc::NfcTag> read_mifare_ultralight_tag_(std::vector<uint8_t> &uid);
  // bool read_mifare_ultralight_bytes_(uint8_t start_page, uint16_t num_bytes, std::vector<uint8_t> &data);
  // bool is_mifare_ultralight_formatted_(const std::vector<uint8_t> &page_3_to_6);
  // uint16_t read_mifare_ultralight_capacity_();
  // bool find_mifare_ultralight_ndef_(const std::vector<uint8_t> &page_3_to_6, uint8_t &message_length,
  //                                   uint8_t &message_start_index);
  // bool write_mifare_ultralight_page_(uint8_t page_num, std::vector<uint8_t> &write_data);
  // bool write_mifare_ultralight_tag_(std::vector<uint8_t> &uid, nfc::NdefMessage *message);
  // bool clean_mifare_ultralight_();

  bool updates_enabled_{false};
  bool requested_read_{false};
  uint32_t update_cnt_{0};
  std::vector<PN5180ISO15693BinarySensor *> binary_sensors_;
  std::vector<nfc::NfcOnTagTrigger *> triggers_ontag_;
  std::vector<nfc::NfcOnTagTrigger *> triggers_ontagremoved_;
  std::vector<uint8_t> current_uid_;
  nfc::NdefMessage *next_task_message_to_write_;
  uint32_t rd_start_time_{0};
  // enum PN5180ReadReady rd_ready_ { WOULDBLOCK };
  enum NfcTask {
    READ = 0,
    CLEAN,
    FORMAT,
    WRITE,
  } next_task_;
  CallbackManager<void()> on_finished_write_callback_;
};

class PN5180ISO15693BinarySensor : public binary_sensor::BinarySensor {
 public:
  void set_uid(const std::vector<uint8_t> &uid) { uid_ = uid; }

  bool process(std::vector<uint8_t> &data);

  void on_scan_end() {
    if (!this->found_) {
      this->publish_state(false);
    }
    this->found_ = false;
  }

 protected:
  std::vector<uint8_t> uid_;
  bool found_{false};
};

class PN5180ISO15693OnFinishedWriteTrigger : public Trigger<> {
 public:
  explicit PN5180ISO15693OnFinishedWriteTrigger(PN5180ISO15693 *parent) {
    parent->add_on_finished_write_callback([this]() { this->trigger(); });
  }
};

template<typename... Ts>
class PN5180ISO15693IsWritingCondition : public Condition<Ts...>, public Parented<PN5180ISO15693> {
 public:
  bool check(Ts... x) override { return this->is_writing(); }
};

}  // namespace pn5180_ISO15693
}  // namespace esphome