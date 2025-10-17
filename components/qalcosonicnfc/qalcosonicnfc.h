// NAME: qalcosonicnfc.h
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
#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include <esphome/core/hal.h>
#include "PN5180ISO15693.h"

// ST25DV04K command definitions
#define ST25_WRITE_MESSAGE       (0xAA)
#define ST25_READ_MSG_LENGTH     (0xAB)
#define ST25_READ_MSG            (0xAC)
#define ST25_READ_DYN_CONFIG     (0xAD)
#define ST25_WRITE_DYN_CONFIG    (0xAE)

#define ST25_MFG_CODE            (0x02)
#define ST25_REQUEST_FLAGS       (0x22) // High Data Rate + Address flag

#define ST25_MB_CTRL_DYN         (0x0D) // Dynamic Control Register Address indicating the mailbox status
#define ST25_MB_EN               (1)
#define ST25_HOST_PUT_MSG        (2)
#define ST25_RF_PUT_MSG          (4)
#define ST25_RFU                 (8)
#define ST25_HOST_MISS_MSG      (16)
#define ST25_RF_MISS_MSG        (32)
#define ST25_HOST_CURRENT_MSG   (64)
#define ST25_RF_CURRENT_MSG     (128)

#define ST25_EH_CTRL_DYN         (0x02) // Dynamic Control Register Address indicating energy harvesting status
#define ST25_EH_EN               (1)    // Is energy harvesting enabled (even after boot)?
#define ST25_EH_ON               (2)    // Is energy harvesting enabled now?
#define ST25_FIELD_ON            (4)    // Is RF field detected
#define ST25_VCC_ON              (8)

#define MBUS_LONG_FRAME_START             (0x68)
#define MBUS_LONG_FRAME_ADDITIONAL_BYTES  (0x06) // Start + Len + Len + Start + CRC + Terminator
#define MBUS_SHORT_FRAME_START            (0x10)
#define MBUS_FRAME_TERMINATOR             (0x16)

namespace esphome {
namespace qalcosonicnfc {

class QalcosonicNfc : public esphome::PollingComponent {
 protected:
  GPIOPin *MOSI_;
  GPIOPin *MISO_;
  GPIOPin *SCK_;
  GPIOPin *NSS_;
  GPIOPin *BUSY_;
  GPIOPin *RST_;
  PN5180ISO15693* nfc_;
  bool errorFlag;
  uint8_t meterUid[8];
  uint8_t *readBuffer; // Buffer for any data that is received
  uint16_t responseLength; // Stores the actual length of the received data
  void showIRQStatus(uint32_t irqStatus);
  bool st25MailboxEnable();
  bool st25MailboxGetState();
  bool st25WriteDynConfig(uint8_t pointerAddress, uint8_t registerValue);
  bool st25ReadDynConfig(uint8_t pointerAddress);
  bool st25WriteMessage(uint8_t *message, uint8_t messageLength);
  bool st25GetMessageLength();
  bool st25GetMessage();
  bool issueMeterCommand(uint8_t *qalcosonicCmd, uint8_t qalcosonicCmdLen);
  bool validateMbusFrame();
  sensor::Sensor *water_usage_sensor_{nullptr};
  sensor::Sensor *water_flow_sensor_{nullptr};
  sensor::Sensor *water_temperature_sensor_{nullptr};
  sensor::Sensor *battery_level_sensor_{nullptr};
  text_sensor::TextSensor *raw_data_sensor_{nullptr};

 public:
  QalcosonicNfc(GPIOPin *mosi, GPIOPin *miso, GPIOPin *sck, GPIOPin *nss, GPIOPin *busy, GPIOPin *rst);
  void set_water_usage_sensor(sensor::Sensor *water_usage_sensor) { water_usage_sensor_ = water_usage_sensor; }
  void set_water_flow_sensor(sensor::Sensor *water_flow_sensor) { water_flow_sensor_ = water_flow_sensor; }
  void set_water_temperature_sensor(sensor::Sensor *water_temperature_sensor) { water_temperature_sensor_ = water_temperature_sensor; }
  void set_battery_level_sensor(sensor::Sensor *battery_level_sensor) { battery_level_sensor_ = battery_level_sensor; }
  void set_raw_data_sensor(text_sensor::TextSensor *raw_data_sensor) { raw_data_sensor_ = raw_data_sensor; }
  void setup() override;
  void loop() override;
  void update() override;

 private:
};

}  // namespace qalcosonicnfc
}  // namespace esphome
