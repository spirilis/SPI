/**
 * File: eusci_spi.c - msp430 USCI SPI implementation
 *
 * EUSCI flavor implementation by Robert Wessels <robertinant@yahoo.com>
 * Copyright (c) 2012 by Rick Kimball <rick@kimballsoftware.com>
 * spi abstraction api for msp430
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 *
 */

#include <msp430.h>
#include <stdint.h>
#include "spi_430.h"

#ifdef __MSP430_HAS_EUSCI_B0__

/**
 * USCI flags for various the SPI MODEs
 *
 * Note: The msp430 UCCKPL tracks the CPOL value. However,
 * the UCCKPH flag is inverted when compared to the CPHA
 * value described in Motorola documentation.
 */

#define SPI_MODE_0 (UCCKPH)		/* CPOL=0 CPHA=0 */
#define SPI_MODE_1 (0)			/* CPOL=0 CPHA=1 */
#define SPI_MODE_2 (UCCKPL | UCCKPH)	/* CPOL=1 CPHA=0 */
#define SPI_MODE_3 (UCCKPL)		/* CPOL=1 CPHA=1 */

#define SPI_MODE_MASK (UCCKPL | UCCKPH)

/**
 * spi_initialize() - Configure USCI UCB0 for SPI mode
 *
 * P2.0 - CS (active low)
 * P1.5 - SCLK
 * P1.6 - MISO aka SOMI
 * P1.7 - MOSI aka SIMO
 *
 */
void spi_initialize(void)
{
	/* Put USCI in reset mode, source USCI clock from SMCLK. */
	UCB0CTLW0 = UCSWRST | UCSSEL_2;

	/* SPI in master MODE 0 - CPOL=0 SPHA=0. */
	UCB0CTLW0 |= SPI_MODE_0 | UCMSB | UCSYNC | UCMST;

	/* P1.6 as SIMO and P1.7 as SOMI. */
	P1SEL1 |= BIT6 | BIT7;

	/* P2.2 as SCK. */
	P2SEL1 |= BIT2;

	/* Set initial speed to 4MHz. */
	UCB0BR0 = SPI_CLOCK_DIV4 & 0xFF;
	UCB0BR1 = (SPI_CLOCK_DIV4 >> 8 ) & 0xFF;

	/* Release USCI for operation. */
	UCB0CTLW0 &= ~UCSWRST;
}

/**
 * spi_disable() - put USCI into reset mode.
 */
void spi_disable(void)
{
	/* Put USCI in reset mode. */
	UCB0CTLW0 |= UCSWRST;
}

/**
 * spi_send() - send a byte and recv response.
 */
uint8_t spi_send(const uint8_t _data)
{
	/* Wait for previous tx to complete. */
	while (!(UCB0IFG & UCTXIFG))
		;

	/* Setting TXBUF clears the TXIFG flag. */
	UCB0TXBUF = _data;

	/* Wait for a rx character? */
	while (!(UCB0IFG & UCRXIFG))
		;

	/* Reading clears RXIFG flag. */
	return UCB0RXBUF;
}

/**
 * spi_send9_9thbit() - helper function for spi_send9() to consolidate the bitbang
 * implementation for all 4 SPI modes.
 */
uint8_t spi_send9_9thbit(const uint16_t _data9th, uint8_t spimode)
{
	uint8_t p1dir_save=P1DIR, p1out_save=P1OUT, p1ren_save=P1REN;
	uint8_t p2dir_save=P2DIR, p2out_save=P2OUT, p2ren_save=P2REN;
	uint8_t retb=0;

	if (spimode == SPI_MODE_0) {
		P1REN &= ~(BIT6 | BIT7); // P1.6 = MOSI, P1.7 = MISO
		P2REN &= ~BIT2;  // 2.2 = SCK
		P1OUT &= ~(BIT6 | BIT7);
		P2OUT &= ~BIT2;
		P1DIR = (p1dir_save & ~(BIT6 | BIT7)) | BIT6;
		P2DIR |= BIT2;
		P1SEL1 &= ~(BIT6 | BIT7);
		P2SEL1 &= ~BIT2;

		if (_data9th)
			P1OUT |= BIT6;
		P2OUT |= BIT2;
		if (P1IN & BIT7)
			retb = 1;
		P2OUT &= ~BIT2;

		P1SEL1 |= BIT6 | BIT7;
		P2SEL1 |= BIT2;
	}

	if (spimode == SPI_MODE_1) {
		P1REN &= ~(BIT6 | BIT7); // P1.6 = MOSI, P1.7 = MISO
		P2REN &= ~BIT2;  // 2.2 = SCK
		P1OUT &= ~(BIT6 | BIT7);
		P2OUT &= ~BIT2;
		P1DIR = (p1dir_save & ~(BIT6 | BIT7)) | BIT6;
		P2DIR |= BIT2;
		P1SEL1 &= ~(BIT6 | BIT7);
		P2SEL1 &= ~BIT2;

		P2OUT |= BIT2;
		if (_data9th)
			P1OUT |= BIT6;
		P2OUT &= ~BIT2;
		if (P1IN & BIT7)
			retb = 1;

		P1SEL1 |= BIT6 | BIT7;
		P2SEL1 |= BIT2;
	}

	if (spimode == SPI_MODE_2) {
		P1REN &= ~(BIT6 | BIT7); // P1.6 = MOSI, P1.7 = MISO
		P2REN &= ~BIT2;  // 2.2 = SCK
		P1OUT &= ~(BIT6 | BIT7);
		P2OUT |= BIT2;
		P1DIR = (p1dir_save & ~(BIT6 | BIT7)) | BIT6;
		P2DIR |= BIT2;
		P1SEL1 &= ~(BIT6 | BIT7);
		P2SEL1 &= ~BIT2;

		if (_data9th)
			P1OUT |= BIT6;
		P2OUT &= ~BIT2;
		if (P1IN & BIT7)
			retb = 1;
		P2OUT |= BIT2;

		P1SEL1 |= BIT6 | BIT7;
		P2SEL1 |= BIT2;
	}

	if (spimode == SPI_MODE_3) {
		P1REN &= ~(BIT6 | BIT7); // P1.6 = MOSI, P1.7 = MISO
		P2REN &= ~BIT2;  // 2.2 = SCK
		P1OUT &= ~(BIT6 | BIT7);
		P2OUT |= BIT2;
		P1DIR = (p1dir_save & ~(BIT6 | BIT7)) | BIT6;
		P2DIR |= BIT2;
		P1SEL1 &= ~(BIT6 | BIT7);
		P2SEL1 &= ~BIT2;

		P2OUT &= ~BIT2;
		if (_data9th)
			P1OUT |= BIT6;
		P2OUT |= BIT2;
		if (P1IN & BIT7)
			retb = 1;

		P1SEL1 |= BIT6 | BIT7;
		P2SEL1 |= BIT2;
	}

	P1DIR = p1dir_save; P1OUT = p1out_save; P1REN = p1ren_save;
	P2DIR = p1dir_save; P2OUT = p1out_save; P2REN = p1ren_save;
	return retb;
}

/**
 * spi_send9() - send a 9-bit byte and recv response.
 */
uint16_t spi_send9(const uint16_t _data)
{
	uint16_t retw=0;
	uint8_t spi_mode_snoop = UCB0CTLW0 & SPI_MODE_MASK;
	uint8_t spi_bitorder_snoop = UCB0CTLW0 & UCMSB;

	/* Wait for previous tx to complete. */
	while (!(UCB0IFG & UCTXIFG))
		;

	// MSB first?
	if (spi_bitorder_snoop) {
		if (spi_send9_9thbit(_data & 0x0100, spi_mode_snoop))
			retw |= 0x0100;
	}

	/* Setting TXBUF clears the TXIFG flag. */
	UCB0TXBUF = (uint8_t) (_data & 0x00FF);

	/* Wait for a rx character? */
	while (!(UCB0IFG & UCRXIFG))
		;

	/* Reading clears RXIFG flag. */
	retw |= UCB0RXBUF;

	// Else LSB first...
	if (!spi_bitorder_snoop) {
		if (spi_send9_9thbit(_data & 0x0100, spi_mode_snoop))
			retw |= 0x0100;
	}

	return retw;
}

/***SPI_MODE_0
 * spi_set_divisor() - set new clock divider for USCI.
 *
 * USCI speed is based on the SMCLK divided by BR0 and BR1.
 *
 */
void spi_set_divisor(const uint16_t clkdiv)
{
	/* Hold UCB0 in reset. */
	UCB0CTLW0 |= UCSWRST;

	UCB0BR0 = clkdiv & 0xFF;
	UCB0BR1 = (clkdiv >> 8 ) & 0xFF;

	/* Release for operation. */
	UCB0CTLW0 &= ~UCSWRST;
}

/**
 * spi_set_bitorder(LSBFIRST=0 | MSBFIRST=1).
 */
void spi_set_bitorder(const uint8_t order)
{
	/* Hold UCB0 in reset. */
	UCB0CTLW0 |= UCSWRST;

	UCB0CTLW0 = (UCB0CTLW0 & ~UCMSB) | ((order == 1 /*MSBFIRST*/) ? UCMSB : 0); /* MSBFIRST = 1 */

	/* Release for operation. */
	UCB0CTLW0 &= ~UCSWRST;
}

/**
 * spi_set_datamode() - mode 0 - 3.
 */
void spi_set_datamode(const uint8_t mode)
{
	/* Hold UCB0 in reset. */
	UCB0CTL1 |= UCSWRST;
	switch(mode) {
	case 0: /* SPI_MODE0 */
		UCB0CTLW0 = (UCB0CTLW0 & ~SPI_MODE_MASK) | SPI_MODE_0;
		break;
	case 1: /* SPI_MODE1 */
		UCB0CTLW0 = (UCB0CTLW0 & ~SPI_MODE_MASK) | SPI_MODE_1;
		break;
	case 2: /* SPI_MODE2 */
		UCB0CTLW0 = (UCB0CTLW0 & ~SPI_MODE_MASK) | SPI_MODE_2;
		break;
	case 4: /* SPI_MODE3 */
		UCB0CTLW0 = (UCB0CTLW0 & ~SPI_MODE_MASK) | SPI_MODE_3;
		break;
	default:
		break;
	}

	/* Release for operation. */
	UCB0CTL1 &= ~UCSWRST;
}
#endif
