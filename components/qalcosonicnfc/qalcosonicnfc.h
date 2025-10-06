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

#ifdef USE_ARDUINO

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include <esphome/core/hal.h>
#include "PN5180ISO15693.h"

namespace esphome {
namespace qalcosonicnfc {

class QalcosonicNfc : public esphome::PollingComponent {
 protected:
  InternalGPIOPin *MOSI_;
  InternalGPIOPin *MISO_;
  InternalGPIOPin *SCK_;
  InternalGPIOPin *NSS_;
  InternalGPIOPin *BUSY_;
  InternalGPIOPin *RST_;
  PN5180ISO15693* nfc_;
  bool errorFlag;
  void showIRQStatus(uint32_t irqStatus);
  sensor::Sensor *water_usage_sensor_{nullptr};
  text_sensor::TextSensor *raw_data_sensor_{nullptr};

 public:
  QalcosonicNfc(InternalGPIOPin *pn5180_mosi_pin, InternalGPIOPin *pn5180_miso_pin, InternalGPIOPin *pn5180_sck_pin, InternalGPIOPin *pn5180_nss_pin, InternalGPIOPin *pn5180_busy_pin, InternalGPIOPin *pn5180_rst_pin);
  void set_water_usage_sensor(sensor::Sensor *water_usage_sensor) { water_usage_sensor_ = water_usage_sensor; }
  void set_raw_data_sensor(text_sensor::TextSensor *raw_data_sensor) { raw_data_sensor_ = raw_data_sensor; }
  void setup() override;
  void loop() override;
  void update() override;

 private:
};

}  // namespace qalcosonicnfc
}  // namespace esphome

#endif
