// NAME: PN5180ISO15693.h
//
// DESC: ISO15693 protocol on NXP Semiconductors PN5180 module for ESPHome. Modified by Mark Hermann.
//
// Copyright (c) 2018 by Andreas Trappmann. All rights reserved.
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
#ifndef PN5180ISO15693_H
#define PN5180ISO15693_H

#include "PN5180.h"

namespace esphome {

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

class PN5180ISO15693 : public PN5180 {

public:
  PN5180ISO15693(GPIOPin *mosi_pin, GPIOPin *miso_pin, GPIOPin *sck_pin, GPIOPin *nss_pin, GPIOPin *busy_pin, GPIOPin *rst_pin);
  
  ISO15693ErrorCode issueISO15693Command(uint8_t *cmd, uint8_t cmdLen, uint8_t **resultPtr);
  ISO15693ErrorCode issueISO15693Command(uint8_t *cmd, uint8_t cmdLen, uint8_t **resultPtr, uint16_t *responseLength);
  ISO15693ErrorCode getInventory(uint8_t *uid);

  ISO15693ErrorCode readSingleBlock(uint8_t *uid, uint8_t blockNo, uint8_t *blockData, uint8_t blockSize);
  ISO15693ErrorCode writeSingleBlock(uint8_t *uid, uint8_t blockNo, uint8_t *blockData, uint8_t blockSize);

  ISO15693ErrorCode getSystemInfo(uint8_t *uid, uint8_t *blockSize, uint8_t *numBlocks);
  void printAfiId(uint8_t afiId);
   
  /*
   * Helper functions
   */
  bool setupRF();
  //const __FlashStringHelper *ISO15693ErrorCodeToStr(ISO15693ErrorCode errorCode);
  const char *ISO15693ErrorCodeToStr(ISO15693ErrorCode errorCode);
};

} //esphome

#endif /* PN5180ISO15693_H */
