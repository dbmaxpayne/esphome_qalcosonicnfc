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
//#define DEBUG 1

#include <Arduino.h>
#include "PN5180ISO15693.h"
#include "esphome/core/log.h"
#include "PN5180Debug.h"

static const char *const TAG = "PN5180ISO15693";

PN5180ISO15693::PN5180ISO15693(uint8_t SSpin, uint8_t BUSYpin, uint8_t RSTpin)
              : PN5180(SSpin, BUSYpin, RSTpin) {
}

/*
 * Inventory, code=01
 *
 * Request format: SOF, Req.Flags, Inventory, AFI (opt.), Mask len, Mask value, CRC16, EOF
 * Response format: SOF, Resp.Flags, DSFID, UID, CRC16, EOF
 *
 */
ISO15693ErrorCode PN5180ISO15693::getInventory(uint8_t *uid) {
  //                     Flags,  CMD, maskLen
  uint8_t inventory[] = { 0x26, 0x01, 0x00 };
  //                        |\- inventory flag + high data rate
  //                        \-- 1 slot: only one card, no AFI field present
  ESP_LOGD(TAG, "Get Inventory...");

  for (int i=0; i<8; i++) {
    uid[i] = 0;
  }

  uint8_t *readBuffer;
  ISO15693ErrorCode rc = issueISO15693Command(inventory, sizeof(inventory), &readBuffer);
  if (ISO15693_EC_OK != rc) {
    return rc;
  }

  for (int i=0; i<8; i++) {
    uid[i] = readBuffer[2+i];
  }
  ESP_LOGD(TAG, "Response flags: %02X, Data Storage Format ID: %02X, UID: %s", readBuffer[0], readBuffer[1], getFormattedHexString(" ", 8, &readBuffer[2]).c_str());

  return ISO15693_EC_OK;
}

/*
 * Read single block, code=20
 *
 * Request format: SOF, Req.Flags, ReadSingleBlock, UID (opt.), BlockNumber, CRC16, EOF
 * Response format:
 *  when ERROR flag is set:
 *    SOF, Resp.Flags, ErrorCode, CRC16, EOF
 *
 *     Response Flags:
  *    xxxx.3xx0
  *         |||\_ Error flag: 0=no error, 1=error detected, see error field
  *         \____ Extension flag: 0=no extension, 1=protocol format is extended
  *
  *  If Error flag is set, the following error codes are defined:
  *    01 = The command is not supported, i.e. the request code is not recognized.
  *    02 = The command is not recognized, i.e. a format error occurred.
  *    03 = The option is not supported.
  *    0F = Unknown error.
  *    10 = The specific block is not available.
  *    11 = The specific block is already locked and cannot be locked again.
  *    12 = The specific block is locked and cannot be changed.
  *    13 = The specific block was not successfully programmed.
  *    14 = The specific block was not successfully locked.
  *    A0-DF = Custom command error codes
 *
 *  when ERROR flag is NOT set:
 *    SOF, Flags, BlockData (len=blockLength), CRC16, EOF
 */
ISO15693ErrorCode PN5180ISO15693::readSingleBlock(uint8_t *uid, uint8_t blockNo, uint8_t *blockData, uint8_t blockSize) {
  //                            flags, cmd, uid,             blockNo
  uint8_t readSingleBlock[] = { 0x22, 0x20, 1,2,3,4,5,6,7,8, blockNo }; // UID has LSB first!
  //                              |\- high data rate
  //                              \-- no options, addressed by UID
  for (int i=0; i<8; i++) {
    readSingleBlock[2+i] = uid[i];
  }

  ESP_LOGD(TAG, "Read Single Block #%u, size=%u: %s", blockNo, blockSize, getFormattedHexString(" ", sizeof(readSingleBlock), readSingleBlock).c_str());

  uint8_t *resultPtr;
  ISO15693ErrorCode rc = issueISO15693Command(readSingleBlock, sizeof(readSingleBlock), &resultPtr);
  if (ISO15693_EC_OK != rc) {
    return rc;
  }

  ESP_LOGD(TAG, "Value=%s [%s]", getFormattedHexString(" ", blockSize, &resultPtr[1]).c_str(), printDataString(blockSize, &resultPtr[1]).c_str());

  return ISO15693_EC_OK;
}

/*
 * Write single block, code=21
 *
 * Request format: SOF, Requ.Flags, WriteSingleBlock, UID (opt.), BlockNumber, BlockData (len=blcokLength), CRC16, EOF
 * Response format:
 *  when ERROR flag is set:
 *    SOF, Resp.Flags, ErrorCode, CRC16, EOF
 *
 *     Response Flags:
  *    xxxx.3xx0
  *         |||\_ Error flag: 0=no error, 1=error detected, see error field
  *         \____ Extension flag: 0=no extension, 1=protocol format is extended
  *
  *  If Error flag is set, the following error codes are defined:
  *    01 = The command is not supported, i.e. the request code is not recognized.
  *    02 = The command is not recognized, i.e. a format error occurred.
  *    03 = The option is not supported.
  *    0F = Unknown error.
  *    10 = The specific block is not available.
  *    11 = The specific block is already locked and cannot be locked again.
  *    12 = The specific block is locked and cannot be changed.
  *    13 = The specific block was not successfully programmed.
  *    14 = The specific block was not successfully locked.
  *    A0-DF = Custom command error codes
 *
 *  when ERROR flag is NOT set:
 *    SOF, Resp.Flags, CRC16, EOF
 */
ISO15693ErrorCode PN5180ISO15693::writeSingleBlock(uint8_t *uid, uint8_t blockNo, uint8_t *blockData, uint8_t blockSize) {
  //                            flags, cmd, uid,             blockNo
  uint8_t writeSingleBlock[] = { 0x22, 0x21, 1,2,3,4,5,6,7,8, blockNo }; // UID has LSB first!
  //                               |\- high data rate
  //                               \-- no options, addressed by UID

  uint8_t writeCmdSize = sizeof(writeSingleBlock) + blockSize;
  uint8_t *writeCmd = (uint8_t*)malloc(writeCmdSize);
  uint8_t pos = 0;
  writeCmd[pos++] = writeSingleBlock[0];
  writeCmd[pos++] = writeSingleBlock[1];
  for (int i=0; i<8; i++) {
    writeCmd[pos++] = uid[i];
  }
  writeCmd[pos++] = blockNo;
  for (int i=0; i<blockSize; i++) {
    writeCmd[pos++] = blockData[i];
  }

  ESP_LOGD(TAG, "Write Single Block #%u, size=%u: %s", blockNo, blockSize, getFormattedHexString(" ", writeCmdSize, writeCmd).c_str());

  uint8_t *resultPtr;
  ISO15693ErrorCode rc = issueISO15693Command(writeCmd, writeCmdSize, &resultPtr);
  if (ISO15693_EC_OK != rc) {
    free(writeCmd);
    return rc;
  }

  free(writeCmd);
  return ISO15693_EC_OK;
}

/*
 * Get System Information, code=2B
 *
 * Request format: SOF, Req.Flags, GetSysInfo, UID (opt.), CRC16, EOF
 * Response format:
 *  when ERROR flag is set:
 *    SOF, Resp.Flags, ErrorCode, CRC16, EOF
 *
 *     Response Flags:
  *    xxxx.3xx0
  *         |||\_ Error flag: 0=no error, 1=error detected, see error field
  *         \____ Extension flag: 0=no extension, 1=protocol format is extended
  *
  *  If Error flag is set, the following error codes are defined:
  *    01 = The command is not supported, i.e. the request code is not recognized.
  *    02 = The command is not recognized, i.e. a format error occurred.
  *    03 = The option is not supported.
  *    0F = Unknown error.
  *    10 = The specific block is not available.
  *    11 = The specific block is already locked and cannot be locked again.
  *    12 = The specific block is locked and cannot be changed.
  *    13 = The specific block was not successfully programmed.
  *    14 = The specific block was not successfully locked.
  *    A0-DF = Custom command error codes
  *
 *  when ERROR flag is NOT set:
 *    SOF, Flags, InfoFlags, UID, DSFID (opt.), AFI (opt.), Other fields (opt.), CRC16, EOF
 *
 *    InfoFlags:
 *    xxxx.3210
 *         |||\_ DSFID: 0=DSFID not supported, DSFID field NOT present; 1=DSFID supported, DSFID field present
 *         ||\__ AFI: 0=AFI not supported, AFI field not present; 1=AFI supported, AFI field present
 *         |\___ VICC memory size:
 *         |        0=Information on VICC memory size is not supported. Memory size field is present. ???
 *         |        1=Information on VICC memory size is supported. Memory size field is present.
 *         \____ IC reference:
 *                  0=Information on IC reference is not supported. IC reference field is not present.
 *                  1=Information on IC reference is supported. IC reference field is not present.
 *
 *    VICC memory size:
 *      xxxb.bbbb nnnn.nnnn
 *        bbbbb - Block size is expressed in number of bytes, on 5 bits, allowing to specify up to 32 bytes i.e. 256 bits.
 *        nnnn.nnnn - Number of blocks is on 8 bits, allowing to specify up to 256 blocks.
 *
 *    IC reference: The IC reference is on 8 bits and its meaning is defined by the IC manufacturer.
 */
ISO15693ErrorCode PN5180ISO15693::getSystemInfo(uint8_t *uid, uint8_t *blockSize, uint8_t *numBlocks) {
  uint8_t sysInfo[] = { 0x22, 0x2b, 1,2,3,4,5,6,7,8 };  // UID has LSB first!
  for (int i=0; i<8; i++) {
    sysInfo[2+i] = uid[i];
  }


  ESP_LOGD(TAG, "Get System Information %s", getFormattedHexString(" ", sizeof(sysInfo), sysInfo).c_str());

  uint8_t *readBuffer;
  ISO15693ErrorCode rc = issueISO15693Command(sysInfo, sizeof(sysInfo), &readBuffer);
  if (ISO15693_EC_OK != rc) {
    return rc;
  }

  for (int i=0; i<8; i++) {
    uid[i] = readBuffer[2+i];
  }

  ESP_LOGD(TAG, "UID %s", getFormattedHexString(" ", 8, uid).c_str());

  uint8_t *p = &readBuffer[10];

  uint8_t infoFlags = readBuffer[1];
  if (infoFlags & 0x01) { // DSFID flag
    uint8_t dsfid = *p++;
    ESP_LOGD(TAG, "DSFID=0x%02X", dsfid); // Data storage format identifier
  }
  else ESP_LOGD(TAG, "No DSFID");

  if (infoFlags & 0x02) { // AFI flag
    uint8_t afi = *p++;
    printAfiId(afi); // Application family identifier
  }
  else ESP_LOGD(TAG, "No AFI");

  if (infoFlags & 0x04) { // VICC Memory size
    *numBlocks = *p++;
    *blockSize = *p++;
    *blockSize = (*blockSize) & 0x1f;

    *blockSize = *blockSize + 1; // range: 1-32
    *numBlocks = *numBlocks + 1; // range: 1-256
    uint16_t viccMemSize = (*blockSize) * (*numBlocks);

    ESP_LOGD(TAG, "VICC MemSize=%i BlockSize=%u NumBlocks=%u", viccMemSize, *blockSize, *numBlocks);
  }
  else ESP_LOGD(TAG, "No VICC memory size");

  if (infoFlags & 0x08) { // IC reference
    uint8_t icRef = *p++;
    ESP_LOGD(TAG, "IC Ref=0x%02X", icRef);
  }
  else ESP_LOGD(TAG, "No IC ref");

  return ISO15693_EC_OK;
}

void PN5180ISO15693::printAfiId(uint8_t afiId) {
  std::string readableAfiId = "";
  switch (afiId >> 4) {
      case 0: readableAfiId = "All families"; break;
      case 1: readableAfiId = "Transport"; break;
      case 2: readableAfiId = "Financial"; break;
      case 3: readableAfiId = "Identification"; break;
      case 4: readableAfiId = "Telecommunication"; break;
      case 5: readableAfiId = "Medical"; break;
      case 6: readableAfiId = "Multimedia"; break;
      case 7: readableAfiId = "Gaming"; break;
      case 8: readableAfiId = "Data storage"; break;
      case 9: readableAfiId = "Item management"; break;
      case 10: readableAfiId = "Express parcels"; break;
      case 11: readableAfiId = "Postal services"; break;
      case 12: readableAfiId = "Airline bags"; break;
      default: readableAfiId = "Unknown"; break;
    }
  
  ESP_LOGD(TAG, "AFI ID=0x%02X [%s]", afiId, readableAfiId.c_str());
}

/*
 * ISO 15693 - Protocol
 *
 * General Request Format:
 *  SOF, Req.Flags, Command code, Parameters, Data, CRC16, EOF
 *
 *  Request Flags:
 *    xxxx.3210
 *         |||\_ Subcarrier flag: 0=single sub-carrier, 1=two sub-carrier
 *         ||\__ Datarate flag: 0=low data rate, 1=high data rate
 *         |\___ Inventory flag: 0=no inventory, 1=inventory
 *         \____ Protocol extension flag: 0=no extension, 1=protocol format is extended
 *
 *  If Inventory flag is set:
 *    7654.xxxx
 *     ||\_ AFI flag: 0=no AFI field present, 1=AFI field is present
 *     |\__ Number of slots flag: 0=16 slots, 1=1 slot
 *     \___ Option flag: 0=default, 1=meaning is defined by command description
 *
 *  If Inventory flag is NOT set:
 *    7654.xxxx
 *     ||\_ Select flag: 0=request shall be executed by any VICC according to Address_flag
 *     ||                1=request shall be executed only by VICC in selected state
 *     |\__ Address flag: 0=request is not addressed. UID field is not present.
 *     |                  1=request is addressed. UID field is present. Only VICC with UID shall answer
 *     \___ Option flag: 0=default, 1=meaning is defined by command description
 *
 * General Response Format:
 *  SOF, Resp.Flags, Parameters, Data, CRC16, EOF
 *
 *  Response Flags:
 *    xxxx.3210
 *         |||\_ Error flag: 0=no error, 1=error detected, see error field
 *         ||\__ RFU: 0
 *         |\___ RFU: 0
 *         \____ Extension flag: 0=no extension, 1=protocol format is extended
 *
 *  If Error flag is set, the following error codes are defined:
 *    01 = The command is not supported, i.e. the request code is not recognized.
 *    02 = The command is not recognized, i.e. a format error occurred.
 *    03 = The option is not supported.
 *    0F = Unknown error.
 *    10 = The specific block is not available.
 *    11 = The specific block is already locked and cannot be locked again.
 *    12 = The specific block is locked and cannot be changed.
 *    13 = The specific block was not successfully programmed.
 *    14 = The specific block was not successfully locked.
 *    A0-DF = Custom command error codes
 *
 *  Function return values:
 *    0 = OK
 *   -1 = No card detected
 *   >0 = Error code
 */
ISO15693ErrorCode PN5180ISO15693::issueISO15693Command(uint8_t *cmd, uint8_t cmdLen, uint8_t **resultPtr) {
  ESP_LOGD(TAG, "Issue Command 0x%02X", cmd[1]);

  sendData(cmd, cmdLen);
  delay(10);
  uint32_t status = getIRQStatus();
  if (0 == (status & RX_SOF_DET_IRQ_STAT)) {
    return EC_NO_CARD;
  }
  while (0 == (status & RX_IRQ_STAT)) {
    delay(10);
    status = getIRQStatus();
  }

  uint32_t rxStatus;
  readRegister(RX_STATUS, &rxStatus);

  uint16_t len = (uint16_t)(rxStatus & 0x000001ff);
  ESP_LOGD(TAG, "RX-Status=0x%02X, len=%u", rxStatus, len);

 *resultPtr = readData(len);
  if (0L == *resultPtr) {
    ESP_LOGE(TAG, "*** ERROR in readData!");
    return ISO15693_EC_UNKNOWN_ERROR;
  }

  ESP_LOGD(TAG, "Read=%s:", getFormattedHexString(" ", len, *resultPtr).c_str());

  uint32_t irqStatus = getIRQStatus();
  if (0 == (RX_SOF_DET_IRQ_STAT & irqStatus)) { // no card detected
     clearIRQStatus(TX_IRQ_STAT | IDLE_IRQ_STAT);
     return EC_NO_CARD;
  }

  uint8_t responseFlags = (*resultPtr)[0];
  if (responseFlags & (1<<0)) { // error flag
    uint8_t errorCode = (*resultPtr)[1];
    ESP_LOGE(TAG, "ERROR code=0x%02X - %s", errorCode, ISO15693ErrorCodeToStr((ISO15693ErrorCode)errorCode));
    if (errorCode >= 0xA0) { // custom command error codes
      return ISO15693_EC_CUSTOM_CMD_ERROR;
    }
    else return (ISO15693ErrorCode)errorCode;
  }
  if (responseFlags & (1<<3)) { // extendsion flag
    ESP_LOGD(TAG, "Extension flag is set!");
  }
  clearIRQStatus(RX_SOF_DET_IRQ_STAT | IDLE_IRQ_STAT | TX_IRQ_STAT | RX_IRQ_STAT);
  return ISO15693_EC_OK;
}

ISO15693ErrorCode PN5180ISO15693::issueISO15693Command(uint8_t *cmd, uint8_t cmdLen, uint8_t **resultPtr, uint16_t *responseLength) {
  ESP_LOGD(TAG, "Issue Command 0x%02X", cmd[1]);

  sendData(cmd, cmdLen);
  delay(10);
  uint32_t status = getIRQStatus();
  if (0 == (status & RX_SOF_DET_IRQ_STAT)) {
    return EC_NO_CARD;
  }
  while (0 == (status & RX_IRQ_STAT)) {
    delay(10);
    status = getIRQStatus();
  }

  uint32_t rxStatus;
  readRegister(RX_STATUS, &rxStatus);
  
  *responseLength = (uint16_t)(rxStatus & 0x000001ff);
  ESP_LOGD(TAG, "RX-Status=0x%02X, len=%u", rxStatus, *responseLength);

 *resultPtr = readData(*responseLength);
  if (0L == *resultPtr) {
    ESP_LOGE(TAG, "*** ERROR in readData!");
    return ISO15693_EC_UNKNOWN_ERROR;
  }

  ESP_LOGD(TAG, "Read=%s:", getFormattedHexString(" ", *responseLength, *resultPtr).c_str());

  uint32_t irqStatus = getIRQStatus();
  if (0 == (RX_SOF_DET_IRQ_STAT & irqStatus)) { // no card detected
     clearIRQStatus(TX_IRQ_STAT | IDLE_IRQ_STAT);
     return EC_NO_CARD;
  }

  uint8_t responseFlags = (*resultPtr)[0];
  if (responseFlags & (1<<0)) { // error flag
    uint8_t errorCode = (*resultPtr)[1];
    ESP_LOGE(TAG, "ERROR code=0x%02X - %s", errorCode, ISO15693ErrorCodeToStr((ISO15693ErrorCode)errorCode));
    if (errorCode >= 0xA0) { // custom command error codes
      return ISO15693_EC_CUSTOM_CMD_ERROR;
    }
    else return (ISO15693ErrorCode)errorCode;
  }

  if (responseFlags & (1<<3)) { // extendsion flag
    ESP_LOGD(TAG, "Extension flag is set!");
  }

  clearIRQStatus(RX_SOF_DET_IRQ_STAT | IDLE_IRQ_STAT | TX_IRQ_STAT | RX_IRQ_STAT);
  return ISO15693_EC_OK;
}

bool PN5180ISO15693::setupRF() {
  ESP_LOGD(TAG, "Loading RF-Configuration...");
  if (!loadRFConfig(0x0d, 0x8d)) {  // ISO15693 parameters
    ESP_LOGE(TAG, "Error loading RF-Configuration");
    return false;
  }

  ESP_LOGD(TAG, "Turning ON RF field...");
  if (!setRF_on()) {
    ESP_LOGE(TAG, "Error turning ON RF field");
    return false;
  }

  writeRegisterWithAndMask(SYSTEM_CONFIG, 0xfffffff8);  // Idle/StopCom Command
  writeRegisterWithOrMask(SYSTEM_CONFIG, 0x00000003);   // Transceive Command

  return true;
}

const __FlashStringHelper *PN5180ISO15693::ISO15693ErrorCodeToStr(ISO15693ErrorCode errorCode) {
  switch (errorCode) {
    case EC_NO_CARD: return F("No card detected!");
    case ISO15693_EC_OK: return F("OK!");
    case ISO15693_EC_NOT_SUPPORTED: return F("Command is not supported!");
    case ISO15693_EC_NOT_RECOGNIZED: return F("Command is not recognized!");
    case ISO15693_EC_OPTION_NOT_SUPPORTED: return F("Option is not supported!");
    case ISO15693_EC_UNKNOWN_ERROR: return F("Unknown error!");
    case ISO15693_EC_BLOCK_NOT_AVAILABLE: return F("Specified block is not available!");
    case ISO15693_EC_BLOCK_ALREADY_LOCKED: return F("Specified block is already locked!");
    case ISO15693_EC_BLOCK_IS_LOCKED: return F("Specified block is locked and cannot be changed!");
    case ISO15693_EC_BLOCK_NOT_PROGRAMMED: return F("Specified block was not successfully programmed!");
    case ISO15693_EC_BLOCK_NOT_LOCKED: return F("Specified block was not successfully locked!");
    default:
      if ((errno >= 0xA0) && (errno <= 0xDF)) {
        return F("Custom command error code!");
      }
      else return F("Undefined error code in ISO15693!");
  }
}
