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
#include "esphome/components/watchdog/watchdog.h"
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
    nfc_ = new PN5180ISO15693(this->NSS_->get_pin(), this->BUSY_->get_pin(), this->RST_->get_pin());
}

bool QalcosonicNfc::st25MailboxEnable() {
    ESP_LOGD(TAG, "ST25: Enabling Mailbox");
    if (!this->st25WriteDynConfig(ST25_MB_CTRL_DYN, ST25_MB_EN)) return false;
    if (!st25MailboxGetState()) return false;
    ESP_LOGD(TAG, "ST25 Mailbox Status: %02X", this->readBuffer[1]);
    if (!(this->readBuffer[1] & ST25_MB_EN)) {
        ESP_LOGE(TAG, "ST25: Mailbox could not be enabled");
        return false;
    }
    
    return true;
}

bool QalcosonicNfc::st25MailboxGetState() {
    ESP_LOGD(TAG, "ST25: Getting mailbox status");
    if (!this->st25ReadDynConfig(ST25_MB_CTRL_DYN)) return false;
    
    return true;
}

bool QalcosonicNfc::st25WriteDynConfig(uint8_t pointerAddress, uint8_t registerValue) {
    // Command structure: Request Flags (1 Byte) + 0xAEh + IC Mfg Code (1 Byte) + UID (8 Bytes) + Pointer Address (1 Byte) + Register Value (1 Byte)
    uint8_t command[] = {ST25_REQUEST_FLAGS, ST25_WRITE_DYN_CONFIG, ST25_MFG_CODE, 1, 2, 3, 4, 5, 6, 7, 8, pointerAddress, registerValue};
    for(int i=0; i<8; i++) command[3 + i] = this->meterUid[i]; // Fill UID fields
    ESP_LOGD(TAG, "ST25: Writing Dynamic Configuration. Pointer Address: %02X, Register Value: %02X", pointerAddress, registerValue);
    ISO15693ErrorCode rc = this->nfc_->issueISO15693Command(command, sizeof(command)/sizeof(command[0]), &this->readBuffer, &this->responseLength);
    if (ISO15693_EC_OK != rc) return false;
    
    return true;
}

bool QalcosonicNfc::st25ReadDynConfig(uint8_t pointerAddress) {
    // Command structure: Request Flags (1 Byte) + 0xADh + IC Mfg Code (1 Byte) + UID (8 Bytes) + Pointer Address (1 Byte)
    uint8_t command[] = {ST25_REQUEST_FLAGS, ST25_READ_DYN_CONFIG, ST25_MFG_CODE, 1, 2, 3, 4, 5, 6, 7, 8, pointerAddress};
    for(int i=0; i<8; i++) command[3 + i] = this->meterUid[i]; // Fill UID fields
    ESP_LOGD(TAG, "ST25: Reading Dynamic Configuration. Pointer Address: %02X", pointerAddress);
    ISO15693ErrorCode rc = this->nfc_->issueISO15693Command(command, sizeof(command)/sizeof(command[0]), &this->readBuffer, &this->responseLength);
    if (ISO15693_EC_OK != rc) return false;
    ESP_LOGD(TAG, "ST25: Received Dynamic Configuration. Address: %02X, value=%02X", pointerAddress, this->readBuffer[1]);
    
    return true;
}

bool QalcosonicNfc::st25WriteMessage(uint8_t *message, uint8_t messageLength) {
    // Command structure: Request Flags (1 Byte) + 0xAAh + IC Mfg Code (1 Byte) + UID (8 Bytes) + MsgLength (1 Byte) + Message Data (MsgLength + 1 Byte)
    uint8_t command[1 + 1 + 1 + 8 + 1 + messageLength];
    command[0] = ST25_REQUEST_FLAGS;
    command[1] = ST25_WRITE_MESSAGE;
    command[2] = ST25_MFG_CODE;
    for (int i=0; i<8; i++) command[i+3] = this->meterUid[i];
    command[11] = messageLength - 1;
    for (int i=0; i<messageLength; i++) command[i+12] = message[i];
    
    ISO15693ErrorCode rc = this->nfc_->issueISO15693Command(command, sizeof(command)/sizeof(command[0]), &this->readBuffer, &this->responseLength);
    if (ISO15693_EC_OK != rc) return false;
    
    return true;
}

bool QalcosonicNfc::st25GetMessageLength() {
    // Command structure: Request Flags (1 Byte) + 0xABh + IC Mfg Code (1 Byte) + UID (8 Bytes)
    uint8_t command[1 + 1 + 1 + 8];
    command[0] = ST25_REQUEST_FLAGS;
    command[1] = ST25_READ_MSG_LENGTH;
    command[2] = ST25_MFG_CODE;
    for (int i=0; i<8; i++) command[i+3] = this->meterUid[i];
    
    ISO15693ErrorCode rc = this->nfc_->issueISO15693Command(command, sizeof(command)/sizeof(command[0]), &this->readBuffer, &this->responseLength);
    if (ISO15693_EC_OK != rc) return false;
    
    return true;
}

bool QalcosonicNfc::st25GetMessage() {
    // Command structure: Request Flags (1 Byte) + 0xACh + IC Mfg Code (1 Byte) + UID (8 Bytes) + MBPointer (1 Byte) + MsgLength (1 Byte)
    
    this->st25GetMessageLength();
    
    uint8_t command[1 + 1 + 1 + 8 + 1 + 1];
    command[0] = ST25_REQUEST_FLAGS;
    command[1] = ST25_READ_MSG;
    command[2] = ST25_MFG_CODE;
    for (int i=0; i<8; i++) command[i+3] = this->meterUid[i];
    command[11] = 0x00; // MBPointer
    command[12] = this->readBuffer[1]; // MsgLength
    
    ISO15693ErrorCode rc = this->nfc_->issueISO15693Command(command, sizeof(command)/sizeof(command[0]), &this->readBuffer, &this->responseLength);
    if (ISO15693_EC_OK != rc) return false;
    ESP_LOGD(TAG, "Received data=%s", getFormattedHexString(" ", this->responseLength, this->readBuffer).c_str());
    
    return true;
}

bool QalcosonicNfc::issueMeterCommand(uint8_t *qalcosonicCmd, uint8_t qalcosonicCmdLen) {
    uint8_t finalCommand[qalcosonicCmdLen + 2];
    uint8_t finalCommandCrc = 0;
    for(int i=0; i<qalcosonicCmdLen; i++) {
        finalCommand[i] = qalcosonicCmd[i];
        if (i>0) finalCommandCrc = finalCommandCrc + qalcosonicCmd[i];
    }
    finalCommand[qalcosonicCmdLen] = finalCommandCrc;
    finalCommand[qalcosonicCmdLen + 1] = QALCOSONIC_CMD_TERMINATOR;
    
    if (!this->st25WriteMessage(finalCommand, qalcosonicCmdLen + 2)) return false;
    if (!this->st25GetMessage()) return false;
  
    return true;
}

void QalcosonicNfc::setup() {
    ESP_LOGD(TAG, "setup");
    
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
  
  ESP_LOGI(TAG, "Water meter found. Getting inventory...");
  ISO15693ErrorCode rc = this->nfc_->getInventory(this->meterUid);
  if (ISO15693_EC_OK != rc) {
    ESP_LOGE(TAG, "Error in getInventory: %s", this->nfc_->ISO15693ErrorCodeToStr(rc));
    this->errorFlag = true;
    this->nfc_->setRF_off();
    return;
  }
  ESP_LOGI(TAG, "Inventory successful, UID=%s", getFormattedHexString(" ", 8, this->meterUid).c_str());
  
  // Get energy harvesting state
  // This is done to drain the main battery as little as possible
  // If pin V_EH is actually connected to anything has not been verified
  // At least this should save energy consumed by the ST25 chip itself
  if(!this->st25ReadDynConfig(ST25_EH_CTRL_DYN)) {
    this->nfc_->setRF_off();
    return;
  }
  if(~this->readBuffer[1] & ST25_EH_EN || ~this->readBuffer[1] & ST25_EH_ON || ~this->readBuffer[1] & ST25_FIELD_ON || ~this->readBuffer[1] & ST25_VCC_ON) {
      ESP_LOGE(TAG, "ST25: Energy harvesting is disabled or not available!");
      ESP_LOGE(TAG, "ST25: EH_EN=%u, EH_ON=%u, FIELD_ON=%u, VCC_ON=%u", this->readBuffer[1] & ST25_EH_EN, (this->readBuffer[1] & ST25_EH_ON) >> 1, (readBuffer[1] & ST25_FIELD_ON) >> 2, (this->readBuffer[1] & ST25_VCC_ON) >> 3);
      this->nfc_->setRF_off();
      return;
  }
  ESP_LOGD(TAG, "ST25: Energy harvesting state: EH_EN=%u, EH_ON=%u, FIELD_ON=%u, VCC_ON=%u", this->readBuffer[1] & ST25_EH_EN, (this->readBuffer[1] & ST25_EH_ON) >> 1, (readBuffer[1] & ST25_FIELD_ON) >> 2, (this->readBuffer[1] & ST25_VCC_ON) >> 3);
  
  if(!this->st25MailboxEnable()) {
      this->nfc_->setRF_off();
      return;
  }
  
  ESP_LOGI(TAG, "Getting water meter infos");

   uint8_t commandResetApp[] = {0x10, 0x40, 0xFE};
   uint8_t commandGetData1[] = {0x10, 0x7B, 0xFE};
   
   if(!issueMeterCommand(commandResetApp, sizeof(commandResetApp)/sizeof(commandResetApp[0]))) {
      this->nfc_->setRF_off();
      return;
    }
   if(!issueMeterCommand(commandGetData1, sizeof(commandGetData1)/sizeof(commandGetData1[0]))) {
      this->nfc_->setRF_off();
      return;
    }

   // Generate the final water meter usage from the returned buffer
   uint32_t waterUsage = uint32_t((unsigned char)(this->readBuffer[57]) << 24 |
                                (unsigned char)(this->readBuffer[56]) << 16 |
                                (unsigned char)(this->readBuffer[55]) << 8 |
                                (unsigned char)(this->readBuffer[54]));
  ESP_LOGI(TAG, "Water Usage: %uL / %9.3fm3", waterUsage, waterUsage/1000.0f);
  this->water_usage_sensor_->publish_state(waterUsage/1000.0f);
  this->raw_data_sensor_->publish_state(getFormattedHexString("", responseLength, readBuffer).c_str());
  
  this->nfc_->setRF_off();
  this->nfc_->printIRQStatus(this->nfc_->getIRQStatus());
  
  ESP_LOGD(TAG, "Update cycle finished");
}

}  // namespace qalcosonicnfc
}  // namespace esphome

#endif
