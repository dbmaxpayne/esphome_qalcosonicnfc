// NAME: PN5180.cpp
//
// DESC: Implementation of PN5180 class. Modified by Mark Hermann.
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

#include <Arduino.h>
#include "esphome/core/log.h"
#include "PN5180.h"
#include "PN5180Debug.h"

// PN5180 1-Byte Direct Commands
// see 11.4.3.3 Host Interface Command List
#define PN5180_WRITE_REGISTER           (0x00)
#define PN5180_WRITE_REGISTER_OR_MASK   (0x01)
#define PN5180_WRITE_REGISTER_AND_MASK  (0x02)
#define PN5180_READ_REGISTER            (0x04)
#define PN5180_WRITE_EEPROM             (0x06)
#define PN5180_READ_EEPROM              (0x07)
#define PN5180_SEND_DATA                (0x09)
#define PN5180_READ_DATA                (0x0A)
#define PN5180_SWITCH_MODE              (0x0B)
#define PN5180_LOAD_RF_CONFIG           (0x11)
#define PN5180_RF_ON                    (0x16)
#define PN5180_RF_OFF                   (0x17)

static const char *const TAG = "PN5180";
uint8_t PN5180::readBuffer[508];

PN5180::PN5180(uint8_t SSpin, uint8_t BUSYpin, uint8_t RSTpin) {
  PN5180_NSS = SSpin;
  PN5180_BUSY = BUSYpin;
  PN5180_RST = RSTpin;

  /*
   * 11.4.1 Physical Host Interface
   * The interface of the PN5180 to a host microcontroller is based on a SPI interface,
   * extended by signal line BUSY. The maximum SPI speed is 7 Mbps and fixed to CPOL
   * = 0 and CPHA = 0.
   */
  // Settings for PN5180: 7Mbps, MSB first, SPI_MODE0 (CPOL=0, CPHA=0)
  PN5180_SPI_SETTINGS = SPISettings(7000000, MSBFIRST, SPI_MODE0);
}

void PN5180::begin() {
  pinMode(PN5180_NSS, OUTPUT);
  pinMode(PN5180_BUSY, INPUT);
  pinMode(PN5180_RST, OUTPUT);

  digitalWrite(PN5180_NSS, HIGH); // disable
  digitalWrite(PN5180_RST, HIGH); // no reset

  SPI.begin();
  ESP_LOGD(TAG, "SPI pinout: SS=%u MOSI=%u MISO=%u SCK=%u", SS, MOSI, MISO, SCK);
}

void PN5180::end() {
  digitalWrite(PN5180_NSS, HIGH); // disable
  SPI.end();
}

/*
 * WRITE_REGISTER - 0x00
 * This command is used to write a 32-bit value (little endian) to a configuration register.
 * The address of the register must exist. If the condition is not fulfilled, an exception is
 * raised.
 */
bool PN5180::writeRegister(uint8_t reg, uint32_t value) {
  uint8_t *p = (uint8_t*)&value;

  ESP_LOGD(TAG, "Write Register 0x%02X:, value (LSB first)=%s", reg, getFormattedHexString(" ", 4, p).c_str());

  /*
  For all 4 byte command parameter transfers (e.g. register values), the payload
  parameters passed follow the little endian approach (Least Significant Byte first).
   */
  uint8_t buf[6] = { PN5180_WRITE_REGISTER, reg, p[0], p[1], p[2], p[3] };

  SPI.beginTransaction(PN5180_SPI_SETTINGS);
  transceiveCommand(buf, 6);
  SPI.endTransaction();

  return true;
}

/*
 * WRITE_REGISTER_OR_MASK - 0x01
 * This command modifies the content of a register using a logical OR operation. The
 * content of the register is read and a logical OR operation is performed with the provided
 * mask. The modified content is written back to the register.
 * The address of the register must exist. If the condition is not fulfilled, an exception is
 * raised.
 */
bool PN5180::writeRegisterWithOrMask(uint8_t reg, uint32_t mask) {
  uint8_t *p = (uint8_t*)&mask;
  
  ESP_LOGD(TAG, "Write Register 0x%02X: with OR mask, value (LSB first)=%s", reg, getFormattedHexString(" ", 4, p).c_str());

  uint8_t buf[6] = { PN5180_WRITE_REGISTER_OR_MASK, reg, p[0], p[1], p[2], p[3] };

  SPI.beginTransaction(PN5180_SPI_SETTINGS);
  transceiveCommand(buf, 6);
  SPI.endTransaction();

  return true;
}

/*
 * WRITE _REGISTER_AND_MASK - 0x02
 * This command modifies the content of a register using a logical AND operation. The
 * content of the register is read and a logical AND operation is performed with the provided
 * mask. The modified content is written back to the register.
 * The address of the register must exist. If the condition is not fulfilled, an exception is
 * raised.
 */
bool PN5180::writeRegisterWithAndMask(uint8_t reg, uint32_t mask) {
  uint8_t *p = (uint8_t*)&mask;

  ESP_LOGD(TAG, "Write Register 0x%02X: with AND mask, value (LSB first)=%s", reg, getFormattedHexString(" ", 4, p).c_str());

  uint8_t buf[6] = { PN5180_WRITE_REGISTER_AND_MASK, reg, p[0], p[1], p[2], p[3] };

  SPI.beginTransaction(PN5180_SPI_SETTINGS);
  transceiveCommand(buf, 6);
  SPI.endTransaction();

  return true;
}

/*
 * READ_REGISTER - 0x04
 * This command is used to read the content of a configuration register. The content of the
 * register is returned in the 4 byte response.
 * The address of the register must exist. If the condition is not fulfilled, an exception is
 * raised.
 */
bool PN5180::readRegister(uint8_t reg, uint32_t *value) {
  ESP_LOGD(TAG, "Reading register %02X", reg);

  uint8_t cmd[2] = { PN5180_READ_REGISTER, reg };

  SPI.beginTransaction(PN5180_SPI_SETTINGS);
  transceiveCommand(cmd, 2, (uint8_t*)value, 4);
  SPI.endTransaction();

  ESP_LOGD(TAG, "Register value=%02X", *value);

  return true;
}

/*
 * READ_EEPROM - 0x07
 * This command is used to read data from EEPROM memory area. The field 'Address'
 * indicates the start address of the read operation. The field Length indicates the number
 * of bytes to read. The response contains the data read from EEPROM (content of the
 * EEPROM); The data is read in sequentially increasing order starting with the given
 * address.
 * EEPROM Address must be in the range from 0 to 254, inclusive. Read operation must
 * not go beyond EEPROM address 254. If the condition is not fulfilled, an exception is
 * raised.
 */
bool PN5180::readEEprom(uint8_t addr, uint8_t *buffer, int len) {
  if ((addr > 254) || ((addr+len) > 254)) {
    ESP_LOGE(TAG, "ERROR: Reading beyond addr 254!\n");
    return false;
  }

  ESP_LOGD(TAG, "Reading EEPROM at 0x%02X, size=", addr, len);

  uint8_t cmd[3] = { PN5180_READ_EEPROM, addr, len };

  SPI.beginTransaction(PN5180_SPI_SETTINGS);
  transceiveCommand(cmd, 3, buffer, len);
  SPI.endTransaction();

  ESP_LOGD(TAG, "EEPROM values:%s", getFormattedHexString(" ", len, buffer).c_str());

  return true;
}

/*
 * SEND_DATA - 0x09
 * This command writes data to the RF transmission buffer and starts the RF transmission.
 * The parameter ‘Number of valid bits in last Byte’ indicates the exact number of bits to be
 * transmitted for the last byte (for non-byte aligned frames).
 * Precondition: Host shall configure the Transceiver by setting the register
 * SYSTEM_CONFIG.COMMAND to 0x3 before using the SEND_DATA command, as
 * the command SEND_DATA is only writing data to the transmission buffer and starts the
 * transmission but does not perform any configuration.
 * The size of ‘Tx Data’ field must be in the range from 0 to 260, inclusive (the 0 byte length
 * allows a symbol only transmission when the TX_DATA_ENABLE is cleared).‘Number of
 * valid bits in last Byte’ field must be in the range from 0 to 7. The command must not be
 * called during an ongoing RF transmission. Transceiver must be in ‘WaitTransmit’ state
 * with ‘Transceive’ command set. If the condition is not fulfilled, an exception is raised.
 */
bool PN5180::sendData(uint8_t *data, int len, uint8_t validBits) {
  if (len > 260) {
    ESP_LOGE(TAG, "ERROR: sendData with more than 260 bytes is not supported!");
    return false;
  }

  ESP_LOGD(TAG, "Send data (len=%i): %s", len, getFormattedHexString(" ", len, data).c_str());

  uint8_t buffer[len+2];
  buffer[0] = PN5180_SEND_DATA;
  buffer[1] = validBits; // number of valid bits of last byte are transmitted (0 = all bits are transmitted)
  for (int i=0; i<len; i++) {
    buffer[2+i] = data[i];
  }

  writeRegisterWithAndMask(SYSTEM_CONFIG, 0xfffffff8);  // Idle/StopCom Command
  writeRegisterWithOrMask(SYSTEM_CONFIG, 0x00000003);   // Transceive Command
  /*
   * Transceive command; initiates a transceive cycle.
   * Note: Depending on the value of the Initiator bit, a
   * transmission is started or the receiver is enabled
   * Note: The transceive command does not finish
   * automatically. It stays in the transceive cycle until
   * stopped via the IDLE/StopCom command
   */

  PN5180TransceiveStat transceiveState = getTransceiveState();
  if (PN5180_TS_WaitTransmit != transceiveState) {
    ESP_LOGE(TAG, "*** ERROR: Transceiver not in state WaitTransmit!?");
    return false;
  }

  SPI.beginTransaction(PN5180_SPI_SETTINGS);
  transceiveCommand(buffer, len+2);
  SPI.endTransaction();

  return true;
}

/*
 * READ_DATA - 0x0A
 * This command reads data from the RF reception buffer, after a successful reception.
 * The RX_STATUS register contains the information to verify if the reception had been
 * successful. The data is available within the response of the command. The host controls
 * the number of bytes to be read via the SPI interface.
 * The RF data had been successfully received. In case the instruction is executed without
 * preceding an RF data reception, no exception is raised but the data read back from the
 * reception buffer is invalid. If the condition is not fulfilled, an exception is raised.
 */
uint8_t * PN5180::readData(int len, uint8_t *buffer /* = NULL */) {
  if (len > 508) {
    ESP_LOGE(TAG, "*** FATAL: Reading more than 508 bytes is not supported!");
    return 0L;
  }
  if (NULL == buffer) {
    buffer = readBuffer;
  }

  ESP_LOGD(TAG, "Reading Data (len=%i)", len);

  uint8_t cmd[2] = { PN5180_READ_DATA, 0x00 };

  SPI.beginTransaction(PN5180_SPI_SETTINGS);
  transceiveCommand(cmd, 2, buffer, len);
  SPI.endTransaction();

  ESP_LOGD(TAG, "Data read: %s", getFormattedHexString(" ", len, buffer).c_str());

  return readBuffer;
}

/*
 * LOAD_RF_CONFIG - 0x11
 * Parameter 'Transmitter Configuration' must be in the range from 0x0 - 0x1C, inclusive. If
 * the transmitter parameter is 0xFF, transmitter configuration is not changed.
 * Field 'Receiver Configuration' must be in the range from 0x80 - 0x9C, inclusive. If the
 * receiver parameter is 0xFF, the receiver configuration is not changed. If the condition is
 * not fulfilled, an exception is raised.
 * The transmitter and receiver configuration shall always be configured for the same
 * transmission/reception speed. No error is returned in case this condition is not taken into
 * account.
 *
 * Transmitter: RF   Protocol          Speed     Receiver: RF    Protocol    Speed
 * configuration                       (kbit/s)  configuration               (kbit/s)
 * byte (hex)                                    byte (hex)
 * ----------------------------------------------------------------------------------------------
 * ->0D              ISO 15693 ASK100  26        8D              ISO 15693   26
 *   0E              ISO 15693 ASK10   26        8E              ISO 15693   53
 */
bool PN5180::loadRFConfig(uint8_t txConf, uint8_t rxConf) {
  ESP_LOGD(TAG, "Load RF-Config: txConf=%u, rxConf=%u", txConf, rxConf);

  uint8_t cmd[3] = { PN5180_LOAD_RF_CONFIG, txConf, rxConf };

  SPI.beginTransaction(PN5180_SPI_SETTINGS);
  transceiveCommand(cmd, 3);
  SPI.endTransaction();

  return true;
}

/*
 * RF_ON - 0x16
 * This command is used to switch on the internal RF field. If enabled the TX_RFON_IRQ is
 * set after the field is switched on.
 */
bool PN5180::setRF_on() {
  ESP_LOGD(TAG, "Set RF ON");

  uint8_t cmd[2] = { PN5180_RF_ON, 0x00 };

  SPI.beginTransaction(PN5180_SPI_SETTINGS);
  transceiveCommand(cmd, 2);
  SPI.endTransaction();

  while (0 == (TX_RFON_IRQ_STAT & getIRQStatus())); // wait for RF field to set up
  clearIRQStatus(TX_RFON_IRQ_STAT);
  return true;
}

/*
 * RF_OFF - 0x17
 * This command is used to switch off the internal RF field. If enabled, the TX_RFOFF_IRQ
 * is set after the field is switched off.
 */
bool PN5180::setRF_off() {
  ESP_LOGD(TAG, "Set RF OFF");

  uint8_t cmd[2] { PN5180_RF_OFF, 0x00 };

  SPI.beginTransaction(PN5180_SPI_SETTINGS);
  transceiveCommand(cmd, 2);
  SPI.endTransaction();

  while (0 == (TX_RFOFF_IRQ_STAT & getIRQStatus())); // wait for RF field to shut down
  clearIRQStatus(TX_RFOFF_IRQ_STAT);
  return true;
}

//---------------------------------------------------------------------------------------------

/*
11.4.3.1 A Host Interface Command consists of either 1 or 2 SPI frames depending whether the
host wants to write or read data from the PN5180. An SPI Frame consists of multiple
bytes.

All commands are packed into one SPI Frame. An SPI Frame consists of multiple bytes.
No NSS toggles allowed during sending of an SPI frame.

For all 4 byte command parameter transfers (e.g. register values), the payload
parameters passed follow the little endian approach (Least Significant Byte first).

Direct Instructions are built of a command code (1 Byte) and the instruction parameters
(max. 260 bytes). The actual payload size depends on the instruction used.
Responses to direct instructions contain only a payload field (no header).
All instructions are bound to conditions. If at least one of the conditions is not fulfilled, an exception is
raised. In case of an exception, the IRQ line of PN5180 is asserted and corresponding interrupt
status register contain information on the exception.
*/

/*
 * A Host Interface Command consists of either 1 or 2 SPI frames depending whether the
 * host wants to write or read data from the PN5180. An SPI Frame consists of multiple
 * bytes.
 * All commands are packed into one SPI Frame. An SPI Frame consists of multiple bytes.
 * No NSS toggles allowed during sending of an SPI frame.
 * For all 4 byte command parameter transfers (e.g. register values), the payload
 * parameters passed follow the little endian approach (Least Significant Byte first).
 * The BUSY line is used to indicate that the system is BUSY and cannot receive any data
 * from a host. Recommendation for the BUSY line handling by the host:
 * 1. Assert NSS to Low
 * 2. Perform Data Exchange
 * 3. Wait until BUSY is high
 * 4. Deassert NSS
 * 5. Wait until BUSY is low
 * If there is a parameter error, the IRQ is set to ACTIVE and a GENERAL_ERROR_IRQ is set.
 */
bool PN5180::transceiveCommand(uint8_t *sendBuffer, size_t sendBufferLen, uint8_t *recvBuffer, size_t recvBufferLen) {
  ESP_LOGD(TAG, "Sending SPI frame: %s", getFormattedHexString(" ", sendBufferLen, sendBuffer).c_str());
  // 0.
  auto wait = 10;
  while (wait-- && LOW != digitalRead(PN5180_BUSY)) delay(1); // wait until busy is low
  if (LOW != digitalRead(PN5180_BUSY)) { ESP_LOGE(TAG, "Failed to wait for PN5180_BUSY pin to get low"); return false; }
  // 1.
  digitalWrite(PN5180_NSS, LOW); delay(2);
  // 2.
  for (uint8_t i=0; i<sendBufferLen; i++) {
    SPI.transfer(sendBuffer[i]);
  }
  // 3.
  wait = 10;
  while(wait-- && HIGH != digitalRead(PN5180_BUSY)) delay(1);  // wait until BUSY is high
  if (HIGH != digitalRead(PN5180_BUSY)) { ESP_LOGE(TAG, "Failed to wait for PN5180_BUSY pin to get high"); return false; }
  // 4.
  digitalWrite(PN5180_NSS, HIGH); delay(1);
  // 5.
  wait = 10;
  while (wait-- && LOW != digitalRead(PN5180_BUSY)) delay(1); // wait unitl BUSY is low
  if (LOW != digitalRead(PN5180_BUSY)) { ESP_LOGE(TAG, "Failed to wait for PN5180_BUSY pin to get low"); return false; }

  // check, if write-only
  //
  if ((0 == recvBuffer) || (0 == recvBufferLen)) return true;
  ESP_LOGD(TAG, "Receiving SPI frame...");

  // 1.
  digitalWrite(PN5180_NSS, LOW); delay(2);
  // 2.
  for (uint8_t i=0; i<recvBufferLen; i++) {
    recvBuffer[i] = SPI.transfer(0xff);
  }
  // 3.
  wait = 10;
  while (wait-- && HIGH != digitalRead(PN5180_BUSY));  // wait until BUSY is high
  if (HIGH != digitalRead(PN5180_BUSY)) { ESP_LOGE(TAG, "Failed to wait for PN5180_BUSY pin to get high"); return false; }
  // 4.
  digitalWrite(PN5180_NSS, HIGH); delay(1);
  // 5.
  wait = 10;
  while (wait-- && LOW != digitalRead(PN5180_BUSY));  // wait until BUSY is low
  if (LOW != digitalRead(PN5180_BUSY)) { ESP_LOGE(TAG, "Failed to wait for PN5180_BUSY pin to get high"); return false; }

  ESP_LOGD(TAG, "Received SPI frame: %s", getFormattedHexString(" ", recvBufferLen, recvBuffer).c_str());

  return true;
}

/*
 * Reset NFC device
 */
void PN5180::reset() {
  digitalWrite(PN5180_RST, LOW);  // at least 10us required
  delay(10);
  digitalWrite(PN5180_RST, HIGH); // 2ms to ramp up required
  delay(10);

  while (0 == (IDLE_IRQ_STAT & getIRQStatus())); // wait for system to start up

  clearIRQStatus(0xffffffff); // clear all flags
}

/**
 * @name  getInterrrupt
 * @desc  read interrupt status register and clear interrupt status
 */
uint32_t PN5180::getIRQStatus() {
  ESP_LOGD(TAG, "Read IRQ-Status register...");

  uint32_t irqStatus;
  readRegister(IRQ_STATUS, &irqStatus);

  this->printIRQStatus(irqStatus);

  return irqStatus;
}

bool PN5180::clearIRQStatus(uint32_t irqMask) {
  ESP_LOGD(TAG, "Clear IRQ-Status with mask=0x%02X:", irqMask);

  return writeRegister(IRQ_CLEAR, irqMask);
}

void PN5180::printIRQStatus(uint32_t irqStatus) {
  std::string readableIRQStatus = "";
  if (irqStatus & (1<< 0)) readableIRQStatus += "RQ ";
  if (irqStatus & (1<< 1)) readableIRQStatus += "TX ";
  if (irqStatus & (1<< 2)) readableIRQStatus += "IDLE ";
  if (irqStatus & (1<< 3)) readableIRQStatus += "MODE_DETECTED ";
  if (irqStatus & (1<< 4)) readableIRQStatus += "CARD_ACTIVATED ";
  if (irqStatus & (1<< 5)) readableIRQStatus += "STATE_CHANGE ";
  if (irqStatus & (1<< 6)) readableIRQStatus += "RFOFF_DET ";
  if (irqStatus & (1<< 7)) readableIRQStatus += "RFON_DET ";
  if (irqStatus & (1<< 8)) readableIRQStatus += "TX_RFOFF ";
  if (irqStatus & (1<< 9)) readableIRQStatus += "TX_RFON ";
  if (irqStatus & (1<<10)) readableIRQStatus += "RF_ACTIVE_ERROR ";
  if (irqStatus & (1<<11)) readableIRQStatus += "TIMER0 ";
  if (irqStatus & (1<<12)) readableIRQStatus += "TIMER1 ";
  if (irqStatus & (1<<13)) readableIRQStatus += "TIMER2 ";
  if (irqStatus & (1<<14)) readableIRQStatus += "RX_SOF_DET ";
  if (irqStatus & (1<<15)) readableIRQStatus += "RX_SC_DET ";
  if (irqStatus & (1<<16)) readableIRQStatus += "TEMPSENS_ERROR ";
  if (irqStatus & (1<<17)) readableIRQStatus += "GENERAL_ERROR ";
  if (irqStatus & (1<<18)) readableIRQStatus += "HV_ERROR ";
  if (irqStatus & (1<<19)) readableIRQStatus += "LPCD ";
  
  ESP_LOGD(TAG, "IRQ-Status 0x%08X: [%s]", irqStatus, readableIRQStatus.c_str());
}


/*
 * Get TRANSCEIVE_STATE from RF_STATUS register
 */
#ifdef DEBUG
extern void showIRQStatus(uint32_t);
#endif

PN5180TransceiveStat PN5180::getTransceiveState() {
  ESP_LOGD(TAG, "Get Transceive state...");

  uint32_t rfStatus;
  if (!readRegister(RF_STATUS, &rfStatus)) {
    printIRQStatus(getIRQStatus());
    ESP_LOGE(TAG, "ERROR reading RF_STATUS register.");
    return PN5180TransceiveStat(0);
  }

  /*
   * TRANSCEIVE_STATEs:
   *  0 - idle
   *  1 - wait transmit
   *  2 - transmitting
   *  3 - wait receive
   *  4 - wait for data
   *  5 - receiving
   *  6 - loopback
   *  7 - reserved
   */
  uint8_t state = ((rfStatus >> 24) & 0x07);
  ESP_LOGD(TAG, "TRANSCEIVE_STATE=0x%02X", state);

  return PN5180TransceiveStat(state);
}
