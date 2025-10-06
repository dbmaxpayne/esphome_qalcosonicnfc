// NAME: PN5180Debug.h
//
// DESC: Helper methods for debugging. Modified by Mark Hermann.
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
#ifndef PN5180DEBUG_H
#define PN5180DEBUG_H

#include "esphome/core/log.h"
#include <string>

extern std::string getFormattedHexString(const char* separator, uint8_t bufLen, uint8_t *bufPointer);
extern std::string printDataString(uint8_t bufLen, uint8_t *bufPointer);

#endif /* PN5180DEBUG_H */
