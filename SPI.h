/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 *
 * 2012-04-29 rick@kimballsoftware.com - added msp430 support.
 * 2013-06-02 spirilis@linux.com - added 9-bit SPI support.
 *
 */

#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

#include <Energia.h>
#include <inttypes.h>

#if defined(__MSP430_HAS_EUSCI_B0__) || defined(__MSP430_HAS_USCI__) || defined(__MSP430_HAS_USI__)
#include "utility/spi_430.h"
#endif

#define SPI_MODE0 0
#define SPI_MODE1 1
#define SPI_MODE2 2
#define SPI_MODE3 4

class SPIClass {
public:
  inline static uint8_t transfer(uint8_t _data);
  inline static uint16_t transfer9(uint16_t _data);

  // SPI Configuration methods

  static void begin(); // Default
  static void end();

  static void setBitOrder(uint8_t);
  static void setDataMode(uint8_t);
  static void setClockDivider(uint8_t);

  inline static void attachInterrupt();
  inline static void detachInterrupt();
};

extern SPIClass SPI;

uint8_t SPIClass::transfer(uint8_t _data) {
    return spi_send(_data);
}

uint16_t SPIClass::transfer9(uint16_t _data) {
    return spi_send9(_data);
}

void SPIClass::attachInterrupt() {
    /* undocumented in Arduino 1.0 */
}

void SPIClass::detachInterrupt() {
    /* undocumented in Arduino 1.0 */
}

#endif
