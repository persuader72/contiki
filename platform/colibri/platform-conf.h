/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/**
 * \file
 *         Platform configuration for the wismote platform.
 */

#ifndef __PLATFORM_CONF_H__
#define __PLATFORM_CONF_H__

#define TMOTE_MYMOTE 1
/*
 * Definitions below are dictated by the hardware and not really
 * changeable!
 */

#define PROCESS_CONF_NO_PROCESS_NAMES 1

#define PLATFORM_HAS_LEDS   1
#define PLATFORM_HAS_BUTTON 0

/* CPU target speed in Hz */
#define F_CPU  7372800uL
//#define F_CPU 16000000uL

/* Our clock resolution, this is the same as Unix HZ. */
#define ACLK_FROM_REFO 1
#define CLOCK_CONF_SECOND 128UL
#define RTIMER_CONF_SECOND (4096U*8)

#define BAUD2UBR(baud) (baud)

#define CCIF
#define CLIF

#define HAVE_STDINT_H
#include "msp430def.h"

// --------------------
// --- INFO MEMORY ----
// --------------------

#define INFOMEM_START 				0x1800
#define INFOMEM_BLOCK_SIZE 			0x80

#define INFOMEM_NODE_ID				0x0000 /* - 0x0004 */
#define INFOMEM_NODE_ID_SIZE		4
#define INFOMEM_MAC_ADDR			0x0004 /* - 0x000C */
#define INFOMEM_MAC_ADDR_SIZE		8
#define INFOMEM_RIME_ADDR			0x000C /* - 0x000F */
#define INFOMEM_RIME_ADDR_SIZE		4

/* Types for clocks and uip_stats */
typedef unsigned short uip_stats_t;
typedef unsigned long clock_time_t;
typedef unsigned long off_t;

/* the low-level radio driver */
//#define NETSTACK_CONF_RADIO   cc2520_driver

#define ROM_ERASE_UNIT_SIZE  512
#define XMEM_ERASE_UNIT_SIZE (64*1024L)


#define CFS_CONF_OFFSET_TYPE    long


/* Use the first 64k of external flash for node configuration */
#define NODE_ID_XMEM_OFFSET     (0 * XMEM_ERASE_UNIT_SIZE)

/* Use the second 64k of external flash for codeprop. */
#define EEPROMFS_ADDR_CODEPROP  (1 * XMEM_ERASE_UNIT_SIZE)

#define CFS_XMEM_CONF_OFFSET    (2 * XMEM_ERASE_UNIT_SIZE)
#define CFS_XMEM_CONF_SIZE      (1 * XMEM_ERASE_UNIT_SIZE)

#define CFS_RAM_CONF_SIZE 4096




/*
 * SPI bus - M25P80 external flash configuration.
 */

#define FLASH_PWR       //3       /* P4.3 Output */
#define FLASH_CS        //4       /* P4.4 Output */
#define FLASH_HOLD      //7       /* P4.7 Output */

/* Enable/disable flash access to the SPI bus (active low). */

#define SPI_FLASH_ENABLE()  //( P4OUT &= ~BV(FLASH_CS) )
#define SPI_FLASH_DISABLE() //( P4OUT |=  BV(FLASH_CS) )

#define SPI_FLASH_HOLD()               // ( P4OUT &= ~BV(FLASH_HOLD) )
#define SPI_FLASH_UNHOLD()              //( P4OUT |=  BV(FLASH_HOLD) )

// at25f512b flash
// flash spi port is shared with mrf49xa port.
//

#define AT25F512B_CS         7       /* P1.7 Output */
#define AT25F512B_PORT(type) P1##type /* P1.7 Output */

#define AT25F512B_SPI_ENABLE()  ( AT25F512B_PORT(OUT) &= ~BV(AT25F512B_CS) )
#define AT25F512B_SPI_DISABLE() ( AT25F512B_PORT(OUT) |=  BV(AT25F512B_CS) )


//
//  MRF49XA configuration and macros
//

// P4.2 - Output: SPI Chip Select (CS_N)
#define MRF49XA_CSN_PORT(type)      P3##type
#define MRF49XA_CSN_PIN             2

// P2.5 - Output: FSK/DATA/FSEL_N (CS_N)
#define MRF49XA_FSELN_PORT(type)      P2##type
#define MRF49XA_FSELN_PIN             5

#define MRF49XA_SPI_ENABLE()     (MRF49XA_CSN_PORT(OUT) &= ~BV(MRF49XA_CSN_PIN)) // ENABLE CSn (active low)
#define MRF49XA_SPI_DISABLE()    (MRF49XA_CSN_PORT(OUT) |=  BV(MRF49XA_CSN_PIN))  // DISABLE CSn (active low)
#define MRF49XA_SPI_IS_ENABLED() ((MRF49XA_CSN_PORT(OUT) & BV(MRF49XA_CSN_PIN)) != BV(MRF49XA_CSN_PIN))


// SPI bus configuration
// SPI input/output registers
#define SPI_TXBUF UCA0TXBUF
#define SPI_RXBUF UCA0RXBUF

#define SPI_WAITFOREOTx()    while ((UCA0STAT & UCBUSY) != 0)  // USART0 Tx ready?
#define SPI_WAITFOREORx()    while ((UCA0IFG & UCRXIFG) == 0)  // USART0 Rx ready?
#define SPI_WAITFORTxREADY() while ((UCA0IFG & UCTXIFG) == 0)  // USART0 Tx buffer ready?

#define MOSI           3 // P3.3 - Output: SPI Master out - slave in (MOSI)
#define MISO           4 // P3.4 - Input:  SPI Master in - slave out (MISO)
#define SCK            7 // P2.7 - Output: SPI Serial Clock (SCLK)

#define MRF49XA_DIO_PIN 4
#define MRF49XA_DIO_PORT(type)      P2##type

// P2.6 - Input: IRQ pin
#define MRF49XA_IRQ_PORT(type)      P2##type
#define MRF49XA_IRQ_PIN             6
#define MRF49XA_IRQ_VECTOR PORT2_VECTOR


// P2.6 - Input: INT pin
/*#define MRF49XA_INT_PORT(type)      P2##type
#define MRF49XA_INT_PIN             5
#define MRF49XA_FIFOP_PIN 5*/

/* FIFOP on external interrupt 0. */
#define MRF49XA_ENABLE_FIFOP_INT()          do { P2IE  |=  BV(MRF49XA_IRQ_PIN); } while (0)
#define MRF49XA_DISABLE_FIFOP_INT()         do { P2IE  &= ~BV(MRF49XA_IRQ_PIN); } while (0)
#define MRF49XA_CLEAR_FIFOP_INT()           do { P2IFG &= ~BV(MRF49XA_IRQ_PIN); } while (0)
#define MRF49XA_EDGE_FALL_INT()             do { P2IES |=  BV(MRF49XA_IRQ_PIN); } while (0)


/*// Colibri module general purpose pin definition
#define COLIBRI_GPIO_PINS 2

#define PIN0SEL   P1SEL
#define PIN0DIR   P1DIR
#define PIN0BIT   BIT1

#define PIN1SEL   P1SEL
#define PIN1DIR   P1DIR
#define PIN1BIT   BIT2*/

#endif /* __PLATFORM_CONF_H__ */
