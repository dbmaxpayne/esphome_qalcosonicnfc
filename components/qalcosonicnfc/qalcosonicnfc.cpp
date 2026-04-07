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
#include "esphome/core/defines.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "qalcosonicnfc.h"
#include "PN5180ISO15693.h"
#include "PN5180Debug.h"

namespace esphome {
namespace qalcosonicnfc {

static const char *const TAG = "qalcosonicnfc";
void testFunc () {
    ESP_LOGI(TAG, TAG);
}
QalcosonicNfc::QalcosonicNfc(GPIOPin *mosi, GPIOPin *miso, GPIOPin *sck, GPIOPin *nss, GPIOPin *busy, GPIOPin *rst) {
    this->MOSI_ = mosi;
    this->MISO_ = miso;
    this->SCK_ = sck;
    this->NSS_ = nss;
    this->BUSY_ = busy;
    this->RST_ = rst;
    this->errorCount = 0;

    nfc_ = new PN5180ISO15693(this->MOSI_, this->MISO_, this->SCK_, this->NSS_, this->BUSY_, this->RST_);
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
    finalCommand[qalcosonicCmdLen + 1] = MBUS_FRAME_TERMINATOR;
    
    if (!this->st25WriteMessage(finalCommand, qalcosonicCmdLen + 2)) return false;
    if (!this->st25GetMessage()) return false;
  
    return true;
}

void QalcosonicNfc::setup() {
    ESP_LOGD(TAG, "setup");
    
    this->errorFlag = false;
    
    this->nfc_->begin();
    if (!this->nfc_->reset()) {
        mark_failed(LOG_STR("No communication to PN5180"));
        return;
    }
    
    ESP_LOGI(TAG, "Reading version info...");
    uint8_t productVersion[2];
    this->nfc_->readEEprom(PRODUCT_VERSION, productVersion, sizeof(productVersion));
    ESP_LOGI(TAG, "Product Version: %u.%u", productVersion[1], productVersion[0]);
    if (0xff == productVersion[1]) { // if product version 255, the initialization failed
        ESP_LOGE(TAG, "Initialization failed! Marking as failed");
        //delay(1000);
        //esp_restart();
        mark_failed(LOG_STR("Incorrect PN5180 product version"));
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
  
  if (!this->nfc_->setupRF()) return;
  
  if (this->errorFlag) {
    this->errorCount++;
    ESP_LOGD(TAG, "Error flag is set. Consecutive errors: %u", this->errorCount);
    uint32_t irqStatus = this->nfc_->getIRQStatus();
    this->nfc_->printIRQStatus(irqStatus);

    if (0 == (RX_SOF_DET_IRQ_STAT & irqStatus)) {
      ESP_LOGI(TAG, "*** Water meter not found or did not reply!");
    }
    
    if (this->errorCount > 5) publishSensorsAsFailed();
    
    if (!this->nfc_->reset()) return;
    if (!this->nfc_->setupRF()) return;

    this->errorFlag = false;
  }
  else {
      this->errorCount = 0;
      this->status_clear_error();
  }
  
  App.feed_wdt();
  ESP_LOGI(TAG, "Scanning for water meter...");
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
  
  App.feed_wdt();
  if(!this->st25MailboxEnable()) {
      this->nfc_->setRF_off();
      return;
  }
  
  ESP_LOGI(TAG, "Getting water meter infos");

   uint8_t commandResetApp[] = {0x10, 0x40, 0xFE};
   uint8_t commandGetData1[] = {0x10, 0x7B, 0xFE};
   
   App.feed_wdt();
   if(!issueMeterCommand(commandResetApp, sizeof(commandResetApp)/sizeof(commandResetApp[0]))) {
      this->nfc_->setRF_off();
      return;
    }
    App.feed_wdt();
   if(!issueMeterCommand(commandGetData1, sizeof(commandGetData1)/sizeof(commandGetData1[0]))) {
      this->nfc_->setRF_off();
      return;
    }
   
   if(!this->validateMbusFrame()) {
      this->nfc_->setRF_off();
      return;
    }
   
   this->publishSensors();
   
  
  this->nfc_->setRF_off();
  this->nfc_->printIRQStatus(this->nfc_->getIRQStatus());
  
  ESP_LOGD(TAG, "Update cycle finished");
}

void QalcosonicNfc::publishSensors() {
    if (responseLength < 10) {
        ESP_LOGE(TAG, "Frame too short");
        return;
    }

    // start frame
    int start_idx = -1;
    for (int i = 0; i <= responseLength - 6; i++) {
        if (readBuffer[i] == 0x68 && 
            readBuffer[i + 3] == 0x68 && 
            readBuffer[i + 1] == readBuffer[i + 2]) {
            start_idx = i;
            break;
        }
    }
    if (start_idx == -1) {
        ESP_LOGE(TAG, "Valid M-Bus frame not found (missing 0x68 L L 0x68 pattern)");
        return;
    }

    uint8_t l_field = readBuffer[start_idx + 1];

    // check length based on start frame
    int total_frame_length = 4 + l_field + 2;
    if (start_idx + total_frame_length > responseLength) {
        ESP_LOGE(TAG, "Incomplete M-Bus frame (L-field: %d, buffer size: %d)", l_field, responseLength);
        return;
    }

    // checksum check
    uint8_t calculated_cs = 0;
    for (int i = start_idx + 4; i < start_idx + 4 + l_field; i++) {
        calculated_cs += readBuffer[i];
    }
    uint8_t received_cs = readBuffer[start_idx + 4 + l_field];
    uint8_t stop_byte = readBuffer[start_idx + 4 + l_field + 1];
    if (calculated_cs != received_cs) {
        ESP_LOGE(TAG, "M-Bus Checksum failed (calculated: %02x, received: %02x)", calculated_cs, received_cs);
        return;
    } else {
        ESP_LOGD(TAG, "M-Bus Checksum OK (calculated: %02x, received: %02x)", calculated_cs, received_cs);
    }

    if (stop_byte != 0x16) {
        ESP_LOGE(TAG, "M-Bus Stop byte missing, found: %02x", stop_byte);
        return;
    }

    uint8_t ci_field = readBuffer[start_idx + 6];
    int payload_offset = start_idx + 7; 
    
    if (ci_field == 0x72 || ci_field == 0x7A) {
        payload_offset += 12;
    }

    uint8_t *buf = this->readBuffer + payload_offset;
    uint8_t *end_buf = this->readBuffer + start_idx + 4 + l_field;

    while (buf < end_buf) {
        uint8_t dif = *buf++;

        // end of user data
        if (dif == 0x0F || dif == 0x1F) {
            break;
        }

        // skip DIF extensions
        while ((buf[-1] & 0x80) && buf < end_buf) {
            buf++; 
        }
        
        if (buf >= end_buf) break;

        uint8_t vif = *buf++;
        uint8_t vife = 0;

        if (vif & 0x80) {
            if (buf >= end_buf) break;
            vife = *buf++;
        }

        // skip further VIF extensions
        while ((buf[-1] & 0x80) && buf < end_buf) {
            buf++; 
        }

        ESP_LOGD(TAG, "DIF:%02x VIF:%02x VIFE:%02x", dif, vif, vife);

        // calculate data size
        uint8_t data_length_type = dif & 0x0F;
        uint8_t data_size = 0;
        switch (data_length_type) {
            case 0x00: data_size = 0; break; // No data
            case 0x01: data_size = 1; break; // 8-bit integer
            case 0x02: data_size = 2; break; // 16-bit integer
            case 0x03: data_size = 3; break; // 24-bit integer
            case 0x04: data_size = 4; break; // 32-bit integer
            case 0x05: data_size = 4; break; // 32-bit real
            case 0x06: data_size = 6; break; // 48-bit integer
            case 0x07: data_size = 8; break; // 64-bit integer
            case 0x08: data_size = 0; break; // Selection for Readout
            case 0x09: data_size = 1; break; // 2 digit BCD
            case 0x0A: data_size = 2; break; // 4 digit BCD
            case 0x0B: data_size = 3; break; // 6 digit BCD
            case 0x0C: data_size = 4; break; // 8 digit BCD
            case 0x0D:
                // variable length data; the first byte of data dictates the length of the string
                data_size = (buf < end_buf) ? (*buf + 1) : 0; 
                break; 
            case 0x0E: data_size = 6; break; // 12 digit BCD
            case 0x0F: data_size = 0; break; // Special Functions
        }

        // Bounds check before reading memory block
        if (buf + data_size > end_buf) {
            ESP_LOGW(TAG, "Data record exceeds frame payload limits");
            break;
        }

        switch (vif << 8 | vife) {
            case 0x1300: {
                int32_t waterUsage = int32_t(buf[3] << 24 | buf[2] << 16 | buf[1] << 8 | buf[0]);
                ESP_LOGI(TAG, "Water Usage: %iL / %1.3fm³", waterUsage, waterUsage/1000.0f);
                this->water_usage_sensor_->publish_state(waterUsage/1000.0f);
                break;
            }

            case 0x933b: {
                int32_t waterUsage = int32_t(buf[3] << 24 | buf[2] << 16 | buf[1] << 8 | buf[0]);
                ESP_LOGI(TAG, "Water Usage (Only Positive): %iL / %1.3fm³", waterUsage, waterUsage/1000.0f);
                this->water_usage_positive_sensor_->publish_state(waterUsage/1000.0f);
                break;
            }

            case 0x933c: {
                int32_t waterUsage = int32_t(buf[3] << 24 | buf[2] << 16 | buf[1] << 8 | buf[0]);
                ESP_LOGI(TAG, "Water Usage (Only Negative): %iL / %1.3fm³", waterUsage, waterUsage/1000.0f);
                this->water_usage_negative_sensor_->publish_state(waterUsage/1000.0f);
                break;
            }

            case 0x3b00: {
                int16_t waterFlow = int16_t(buf[1] << 8 | buf[0]);
                ESP_LOGI(TAG, "Water Flow: %iL / %1.3fm³", waterFlow, waterFlow/1000.0f);
                this->water_flow_sensor_->publish_state(waterFlow/1000.0f);
                break;
            }

            case 0x5900: {
                int16_t flowTemperature = int16_t(buf[1] << 8 | buf[0]);
                ESP_LOGI(TAG, "Water Temperature: %2.1f°C", flowTemperature/100.0f);
                this->water_temperature_sensor_->publish_state(flowTemperature/100.0f);
                break;
            }

            case 0x6600: {
                int16_t externalTemperature = int16_t(buf[1] << 8 | buf[0]);
                ESP_LOGI(TAG, "External Temperature: %2.1f°C", externalTemperature/10.0f);
                this->external_temperature_sensor_->publish_state(externalTemperature/10.0f);
                break;
            }

            case 0x6d00: {
                int32_t minute = buf[0] & 0x3F;
                int32_t hour = buf[1] & 0x1F;
                int32_t day = buf[2] & 0x1F;
                int32_t month = buf[3] & 0x0F;
                int32_t year = (buf[2] >> 5 | (buf[3] >> 1) & 0xF8) + 2000;
                char str_timepoint[32];
                if (!this->timezone_.empty()) {
                    setenv("TZ", this->timezone_.c_str(), 1);
                    tzset();
                    ESP_LOGI(TAG, "using timezone %s", this->timezone_.c_str());
                    struct tm timeinfo = {};
                    timeinfo.tm_year = year - 1900;
                    timeinfo.tm_mon = month - 1;
                    timeinfo.tm_mday = day;
                    timeinfo.tm_hour = hour;
                    timeinfo.tm_min = minute;
                    timeinfo.tm_sec = 0;
                    timeinfo.tm_isdst = 0;
                    mktime(&timeinfo);
                    strftime(str_timepoint, sizeof(str_timepoint), "%Y-%m-%dT%H:%M:%S%z", &timeinfo);
                } else {
                    // fallback if timezone is not set
                    snprintf(str_timepoint, sizeof(str_timepoint), "%04u-%02u-%02u %02u:%02u", year, month, day, hour, minute);
                }
                ESP_LOGI(TAG, "Timepoint: %s", str_timepoint);
                this->timepoint_sensor_->publish_state(str_timepoint);

                // publish the timepoint string as raw data without any date math
                // may be offset from the current time by one hour because the meter does not switch to/from DST
                char str_timepoint_raw[17];
                snprintf(str_timepoint_raw, sizeof(str_timepoint_raw), "%04u-%02u-%02u %02u:%02u", year, month, day, hour, minute);
                ESP_LOGI(TAG, "Timepoint raw: %s", str_timepoint_raw);
                this->timepoint_sensor_raw_->publish_state(str_timepoint_raw);
                break;
            }

            case 0xfd74: {
                uint8_t batteryPercentage = buf[0];
                ESP_LOGI(TAG, "Battery Percentage: %u%%", batteryPercentage);
                this->battery_level_sensor_->publish_state(batteryPercentage);
                break;
            }

            case 0x7800:
                {
                    uint32_t serialNumber = 0;
                    // serial numbers are usually sent as 4-byte BCD (DIF = 0x0C)
                    if ((dif & 0x0F) == 0x0C) { 
                        serialNumber = (buf[3] >> 4) * 10000000 + (buf[3] & 0x0F) * 1000000 +
                                       (buf[2] >> 4) * 100000 + (buf[2] & 0x0F) * 10000 +
                                       (buf[1] >> 4) * 1000 + (buf[1] & 0x0F) * 100 +
                                       (buf[0] >> 4) * 10 + (buf[0] & 0x0F);
                    } else { 
                        // fallback if the meter sends it as a standard binary integer
                        serialNumber = uint32_t(buf[3] << 24 | buf[2] << 16 | buf[1] << 8 | buf[0]);
                    }
                    ESP_LOGI(TAG, "Serial Number: %08u", serialNumber);
                    char str_serial_number[9]; 
                    snprintf(str_serial_number, sizeof(str_serial_number), "%08u", serialNumber);
                    this->serial_number_sensor_->publish_state(str_serial_number);
                    break;
                }

            case 0xfd17:
                {
                    // This should be the correct byte order the way I understand the M-Bus and Qalcosonic W1 documentation.
                    // I don't have any errors on my meter, so I can't test it.
                    // The byte order should correspond to the four error digits on the meter's display.
                    char str_error[12]; 
                    snprintf(str_error, sizeof(str_error), "%02X %02X %02X %02X", buf[0], buf[1], buf[2], buf[3]);
                    ESP_LOGI(TAG, "Error Flags Raw: %s", str_error);
                    this->error_flags_raw_->publish_state(str_error);

                    this->error_reconfiguration_warning_->publish_state(buf[0] & (1 << 1));
                    this->error_no_consumption_->publish_state(buf[0] & (1 << 2));
                    this->error_damage_meter_housing_->publish_state(buf[0] & (1 << 3));
                    this->error_calculator_hardware_failure_->publish_state(buf[0] & (1 << 4));

                    this->error_leakage_->publish_state(buf[1] & (1 << 1));
                    this->error_burst_->publish_state(buf[1] & (1 << 2));
                    this->error_optical_communication_->publish_state(buf[1] & (1 << 3));
                    this->error_low_battery_->publish_state(buf[1] & (1 << 4));

                    this->error_software_failure_->publish_state(buf[2] & (1 << 3));
                    this->error_hardware_failure_->publish_state(buf[2] & (1 << 4));

                    this->error_no_signal_->publish_state(buf[3] & (1 << 1));
                    this->error_reverse_flow_->publish_state(buf[3] & (1 << 2));
                    this->error_flow_rate_->publish_state(buf[3] & (1 << 3));
                    this->error_freeze_alert_->publish_state(buf[3] & (1 << 4));
                    break;
                }

            case 0x2400:
                {
                    uint32_t operatingTimeSec = uint32_t(buf[3] << 24 | buf[2] << 16 | buf[1] << 8 | buf[0]);
                    ESP_LOGI(TAG, "Operating Time: %u seconds (%.1f days)", operatingTimeSec, operatingTimeSec / 86400.0f);
                    this->operating_time_sensor_->publish_state(operatingTimeSec);
                    break;
                }

            case 0x2000:
                {
                    uint32_t onTimeSec = uint32_t(buf[3] << 24 | buf[2] << 16 | buf[1] << 8 | buf[0]);
                    ESP_LOGI(TAG, "On Time: %u seconds (%.1f days)", onTimeSec, onTimeSec / 86400.0f);
                    this->on_time_sensor_->publish_state(onTimeSec);
                    break;
                }
        }

        buf += data_size;
    }

    this->raw_data_sensor_->publish_state(getFormattedHexString("", responseLength, readBuffer).c_str());
}

void QalcosonicNfc::publishSensorsAsFailed() {
    this->water_usage_sensor_->publish_state(NAN);
    this->water_usage_positive_sensor_->publish_state(NAN);
    this->water_usage_negative_sensor_->publish_state(NAN);
    this->water_flow_sensor_->publish_state(NAN);
    this->water_temperature_sensor_->publish_state(NAN);
    this->external_temperature_sensor_->publish_state(NAN);
    this->battery_level_sensor_->publish_state(NAN);
    this->operating_time_sensor_->publish_state(NAN);
    this->on_time_sensor_->publish_state(NAN);
    this->raw_data_sensor_->publish_state("");
    this->serial_number_sensor_->publish_state("");
    this->error_flags_raw_->publish_state("");
    this->error_reconfiguration_warning_->publish_state(false);
    this->error_no_consumption_->publish_state(false);
    this->error_damage_meter_housing_->publish_state(false);
    this->error_calculator_hardware_failure_->publish_state(false);
    this->error_leakage_->publish_state(false);
    this->error_burst_->publish_state(false);
    this->error_optical_communication_->publish_state(false);
    this->error_low_battery_->publish_state(false);
    this->error_software_failure_->publish_state(false);
    this->error_hardware_failure_->publish_state(false);
    this->error_no_signal_->publish_state(false);
    this->error_reverse_flow_->publish_state(false);
    this->error_flow_rate_->publish_state(false);
    this->error_freeze_alert_->publish_state(false);

    this->status_set_error();
}

bool QalcosonicNfc::validateMbusFrame() {
    if (this->readBuffer[1] != MBUS_LONG_FRAME_START) { ESP_LOGE(TAG, "Incorrect MBUS frame start. Expected=%02X, got=%02X", MBUS_LONG_FRAME_START, this->readBuffer[1]); return false; }
    uint8_t mbusStartOfPayload = 5;
    uint8_t mbusFrameLength    = this->readBuffer[2];
    uint8_t mbusEndOfPayload   = mbusStartOfPayload + mbusFrameLength;
    uint8_t mbusEndOfFrame     = MBUS_LONG_FRAME_ADDITIONAL_BYTES + mbusFrameLength;
    uint8_t mbusCRCPos         = mbusEndOfFrame - 1;
    uint8_t totalMessageLength = mbusEndOfFrame + 2; // Leading 00 (response flags) & trailing F0 (unknown)
    if (totalMessageLength != this->responseLength) { ESP_LOGE(TAG, "Incorrect responseLength. Expected=%02X, got=%02X", totalMessageLength, this->responseLength); return false; }
    if (this->readBuffer[mbusEndOfFrame] != MBUS_FRAME_TERMINATOR) { ESP_LOGE(TAG, "Incorrect MBUS frame terminator. Expected=%02X, got=%02X", MBUS_FRAME_TERMINATOR, this->readBuffer[mbusEndOfFrame]); return false; }
    uint8_t CRC = 0;
    for(int i=mbusStartOfPayload; i<mbusEndOfPayload; i++) CRC = CRC + this->readBuffer[i];
    if (this->readBuffer[mbusCRCPos] != CRC) { ESP_LOGE(TAG, "Incorrect MBUS frame CRC. Expected=%02X, got=%02X", CRC, this->readBuffer[mbusCRCPos]); return false; }
    
    return true;
}

}  // namespace qalcosonicnfc
}  // namespace esphome
