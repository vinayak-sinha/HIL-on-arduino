#pragma once
#include <Arduino.h>
#include "tdc_contract.h"

// Write spi_controller.cpp. Use SPI.h with SPISettings(1000000, MSBFIRST, SPI_MODE0).
// Every transaction: beginTransaction, CS low, pause INTERBYTE_US, then bytes with
// INTERBYTE_US between each, pause, CS high, endTransaction.
// (The peripheral is an AVR that reloads SPDR in an interrupt; it can't take back-to-back bytes.)

const SPISettings mySetting(100000, MSBFIRST, SPI_MODE0);
// Set CS pins as outputs (idle HIGH) and call SPI.begin().
void spiControllerBegin(uint8_t csA, uint8_t csB) {
  pinMode(csA, OUTPUT);
  pinMode(csB, OUTPUT);

  digitalWrite(csA, HIGH);
  digitalWrite(csB, HIGH);

  SPI.begin();
}

// Send CMD_WRITE(addr), then val.
void writeReg(uint8_t cs, uint8_t addr, uint8_t val) {
  SPI.beginTransaction(mySetting);

  digitalWrite(cs, LOW);
  delayMicroseconds(INTERBYTE_US);

  SPI.transfer(CMD_WRITE(addr));
  delayMicroseconds(INTERBYTE_US);

  SPI.transfer(val);
  delayMicroseconds(INTERBYTE_US);

  digitalWrite(cs,HIGH);

  spi.endTransaction();
}

// Send CMD_WRITE(addr), then n bytes (auto-increment is set by CMD_WRITE).
void writeBurst(uint8_t cs, uint8_t addr, const uint8_t *d, uint8_t n) {
  SPI.beginTransaction(mySetting);

  digitalWrite(cs, LOW);
  delayMicroseconds(INTERBYTE_US);

  SPI.transfer(CMD_WRITE(addr));
  delayMicroseconds(INTERBYTE_US);

  for(uint8_t i = 0; i < n; i++) {
    SPI.transfer(d[i]);
    delayMicroseconds(INTERBYTE_US);
  }

  digitalWrite(cs,HIGH);

  SPI.endTransaction();
}

// Send CMD_READ(addr), then one dummy byte (0x00); return the byte clocked back with it.
// The byte received while sending the command is ignored.
uint8_t readReg(uint8_t cs, uint8_t addr) {
  SPI.beginTransaction(mySetting);

  digitalWrite(cs, LOW);
  delayMicroseconds(INTERBYTE_US);

  SPI.transfer(CMD_READ(addr));
  delayMicroseconds(INTERBYTE_US);

  uint8_t val = SPI.transfer(0x00);
  delayMicroseconds(INTERBYTE_US);

  digitalWrite(cs, HIGH);

  SPI.endTransaction();

  return val;
}

// Read TIME1..CALIBRATION2 in ONE transaction: send CMD_READ(REG_TIME1), then
// 39 dummy bytes (40 bytes total). The reply to the command byte is ignored.
// The 39 bytes that follow are 13 registers x 3 bytes, MSB first.
// out[0]=TIME1, out[1]=CLOCK_COUNT1, out[2]=TIME2, ... out[11]=CAL1, out[12]=CAL2.
// Bit 23 of each register is parity: mask it off (keep bits 22:0).
void readResult(uint8_t cs, uint32_t out[N_RESULT_REGS]) {
  SPI.beginTransaction(mySetting);

  digitalWrite(cs, LOW);
  delayMicroseconds(INTERBYTE_US);
  
  SPI.transfer(CMD_READ(REG_TIME1));
  delayMicroseconds(INTERBYTE_US);

  for (int i = 0; i < N_RESULT_REGS; i++) {
    uint32_t value = 0;

    for (int j = 0; j < 3; j++) {
      uint8_t byteVal = SPI.transfer(0x00);
      delayMicroseconds(INTERBYTE_US);
      value = (value << 8) | byteVal;
    }

    out[i] = value & 0x7FFFFF;
  }

  digitalWrite(cs, HIGH);

  SPI.endTransaction();
}