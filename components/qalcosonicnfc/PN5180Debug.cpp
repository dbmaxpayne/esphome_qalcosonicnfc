// NAME: PN5180Debug.cpp
//
// DESC: Helper functions for debugging. Modified by Mark Hermann.
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
#include <inttypes.h>
#include "PN5180Debug.h"
#include <string>
#include <Arduino.h>
#include "esphome/core/log.h"

std::string getFormattedHexString(const char* separator, uint8_t bufLen, uint8_t *bufPointer)
  {
      uint8_t useSeparator = 0;
      if (strcmp(separator,"") != 0) useSeparator = 1;
      
  char logBuf[bufLen * (2 + useSeparator) + 1];
  for (size_t i = 0; i < bufLen; i++) {
        snprintf(logBuf + i*(2 + useSeparator), 4, "%02X%s", bufPointer[i], separator);
    }
    
    return logBuf;
  }
  
std::string printDataString(uint8_t bufLen, uint8_t *bufPointer)
  {
  char logBuf[bufLen + 1];
  for (size_t i = 0; i < bufLen; i++) {
      if (isPrintable(bufPointer[i])) {
        snprintf(logBuf + i, 2, "%c", bufPointer[i]);
      }
      else snprintf(logBuf + i, 2, "%s", ".");
    }
    
    return logBuf;
  }
