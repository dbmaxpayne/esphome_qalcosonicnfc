// NAME: qalcosonicnfc.cpp
//
// DESC: ESPHome component for reading Axioma Qalcosonic water meters via NFC.
//
// Copyright (c) 2025 by Mark Hermann. All rights reserved.
//
// This file is part of the esphome_qalcosonicnfc component.
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
#ifdef USE_ARDUINO

#include "esphome/core/defines.h"
#include "esphome/core/log.h"
#include "qalcosonicnfc.h"
#include "PN5180ISO15693.h"
#include "PN5180Debug.h"

namespace esphome {
namespace qalcosonicnfc {

static const char *const TAG = "qalcosonicnfc";

QalcosonicNfc::QalcosonicNfc(InternalGPIOPin *pn5180_mosi_pin, InternalGPIOPin *pn5180_miso_pin, InternalGPIOPin *pn5180_sck_pin, InternalGPIOPin *pn5180_nss_pin, InternalGPIOPin *pn5180_busy_pin, InternalGPIOPin *pn5180_rst_pin) {
        this->MOSI_ = pn5180_mosi_pin;
        this->MISO_ = pn5180_miso_pin;
        this->SCK_ = pn5180_sck_pin;
        this->NSS_ = pn5180_nss_pin;
        this->BUSY_ = pn5180_busy_pin;
        this->RST_ = pn5180_rst_pin;
        //PN5180ISO15693 nfc_(this->NSS_->get_pin(), this->BUSY_->get_pin(), this->RST_->get_pin());
        nfc_ = new PN5180ISO15693(this->NSS_->get_pin(), this->BUSY_->get_pin(), this->RST_->get_pin());
        //nfc_ = PN5180ISO15693(1, 2, 3);
}

void QalcosonicNfc::setup() {
  ESP_LOGI(TAG, "setup");

  this->errorFlag = false;

  this->nfc_->begin();
  this->nfc_->reset();
  
  ESP_LOGI(TAG, "Reading version info...");
  uint8_t productVersion[2];
  this->nfc_->readEEprom(PRODUCT_VERSION, productVersion, sizeof(productVersion));
  ESP_LOGI(TAG, "Product Version: %u.%u", productVersion[1], productVersion[0]);
  if (0xff == productVersion[1]) { // if product version 255, the initialization failed
    ESP_LOGE(TAG, "Initialization failed! Marking as failed");
    //delay(1000);
    //esp_restart();
    mark_failed();
  }
  
  uint8_t firmwareVersion[2];
  this->nfc_->readEEprom(FIRMWARE_VERSION, firmwareVersion, sizeof(firmwareVersion));
  ESP_LOGI(TAG, "Firmware Version: %u.%u", firmwareVersion[1], firmwareVersion[0]);

  uint8_t eepromVersion[2];
  this->nfc_->readEEprom(EEPROM_VERSION, eepromVersion, sizeof(eepromVersion));
  ESP_LOGI(TAG, "EEPROM Version: %u.%u", eepromVersion[1], eepromVersion[0]);
  
  //this->nfc_->end();
  
}

void QalcosonicNfc::loop() {
  
}

void QalcosonicNfc::update() {
  ESP_LOGD(TAG, "Update cycle has been started");
  
  this->nfc_->setupRF();
  
  if (this->errorFlag) {
    ESP_LOGD(TAG, "Error flag is set.");
    uint32_t irqStatus = this->nfc_->getIRQStatus();
    this->nfc_->printIRQStatus(irqStatus);

    if (0 == (RX_SOF_DET_IRQ_STAT & irqStatus)) {
      ESP_LOGI(TAG, "*** Water meter not found or did not reply!");
    }
    
    this->nfc_->reset();
    this->nfc_->setupRF();

    this->errorFlag = false;
  }
  
  //this->nfc_->begin();
  //this->nfc_->reset();
  
  ESP_LOGI(TAG, "Water meter found. Getting inventory...");
  uint8_t uid[8];
  ISO15693ErrorCode rc = this->nfc_->getInventory(uid);
  if (ISO15693_EC_OK != rc) {
    ESP_LOGE(TAG, "Error in getInventory: %s", this->nfc_->ISO15693ErrorCodeToStr(rc));
    this->errorFlag = true;
    this->nfc_->setRF_off();
    return;
  }
  ESP_LOGD(TAG, "Inventory successful, UID=%s", getFormattedHexString(" ", sizeof(uid), uid).c_str());
  
  ESP_LOGI(TAG, "Getting water meter infos");
  uint8_t *readBuffer; // Buffer for any data that is received
  uint16_t responseLength; // Stores the actual length of the received data

  // List of initialization commands needed to get the final data from the meter
  std::vector<std::vector<uint8_t>> commands = {{ 0x22, 0xAE, 0x02 , uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], uid[6], uid[7], 0x0D, 0x01 },
                                                { 0x22, 0xAD, 0x02 , uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], uid[6], uid[7], 0x0D },
                                                { 0x22, 0xAA, 0x02 , uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], uid[6], uid[7], 0x04, 0x10, 0x40, 0xFE, 0x3E, 0x16 },
                                                { 0x22, 0xAB, 0x02 , uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], uid[6], uid[7] },
                                                { 0x22, 0xAC, 0x02 , uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], uid[6], uid[7], 0x00, 0x01 },
                                                { 0x22, 0xAA, 0x02 , uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], uid[6], uid[7], 0x09, 0x68, 0x04, 0x04, 0x68, 0x73, 0xFE, 0x50, 0x00, 0xC1, 0x16 },
                                                { 0x22, 0xAB, 0x02 , uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], uid[6], uid[7] },
                                                { 0x22, 0xAC, 0x02 , uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], uid[6], uid[7], 0x00, 0x01 },
                                                { 0x22, 0xAA, 0x02 , uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], uid[6], uid[7], 0x04, 0x10, 0x7B, 0xFE, 0x79, 0x16 },
                                                { 0x22, 0xAB, 0x02 , uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], uid[6], uid[7] },
                                                { 0x22, 0xAC, 0x02 , uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], uid[6], uid[7], 0x00, 0x57 }
                                              };

  // Loop through all the commands and discard the returned data except for the last one
  for (int i=0; i<commands.size(); i++) {
    ESP_LOGD(TAG, "Sending command=%s", getFormattedHexString(" ", commands[i].size(), commands[i].data()).c_str());
    rc = this->nfc_->issueISO15693Command(commands[i].data(), commands[i].size(), &readBuffer, &responseLength);
    if (ISO15693_EC_OK != rc) {
      ESP_LOGE(TAG, "Error: %s", this->nfc_->ISO15693ErrorCodeToStr(rc));
      this->errorFlag = true;
      this->nfc_->setRF_off();
      return;
  }
    ESP_LOGD(TAG, "NFC Status: %s", this->nfc_->ISO15693ErrorCodeToStr(rc));
    ESP_LOGD(TAG, "Received data=%s", getFormattedHexString(" ", responseLength, readBuffer).c_str());
  }

   // Generate the final water meter usage from the returned buffer
   uint32_t waterUsage = uint32_t((unsigned char)(readBuffer[57]) << 24 |
                                (unsigned char)(readBuffer[56]) << 16 |
                                (unsigned char)(readBuffer[55]) << 8 |
                                (unsigned char)(readBuffer[54]));
  ESP_LOGI(TAG, "Water Usage: %uL / %9.3fm3", waterUsage, waterUsage/1000.0f);
  this->water_usage_sensor_->publish_state(waterUsage/1000.0f);
  this->raw_data_sensor_->publish_state(getFormattedHexString("", responseLength, readBuffer).c_str());
  
  this->nfc_->setRF_off();
  this->nfc_->printIRQStatus(this->nfc_->getIRQStatus());
  
  ESP_LOGD(TAG, "Update cycle finished");
  
  //this->nfc_->end();
}

}  // namespace qalcosonicnfc
}  // namespace esphome

#endif
