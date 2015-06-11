#ifndef M25PE16_H
#define M25PE16_H

#include "contiki.h"
#include "dev/spi.h"

#define RDY_BUSY 0x01
#define CRC_BLOCK 32

//#define M25PE16_CS         2       	/* P3.1 Output */
//#define M25PE16_PORT(type) P4##type 	/* P3.1 Output */

//#define M25PE16_SPI_ENABLE()  ( M25PE16_PORT(OUT) &= ~BV(M25PE16_CS) )
//#define M25PE16_SPI_DISABLE() ( M25PE16_PORT(OUT) |=  BV(M25PE16_CS) )

typedef enum _m25pe16b_instr {
	m25pe16b_WritePage		= 0x0A,
	m25pe16b_ProgramPage	= 0x02,

	m25pe16b_ReadPage  		= 0x03,
	m25pe16b_HSReadPage  	= 0x0B,
	m25pe16b_PageErase		= 0xDB,  //256byte
	m25pe16b_SubSectErase	= 0x20,  //4k
	m25pe16b_SectErase		= 0xD8,  //64k

	m25pe16b_EraseBulk 		= 0xC7,

	m25pe16b_Rdsr      		= 0x05,
	m25pe16b_Wrsr      		= 0x01,
	m25pe16b_Wren      		= 0x06,
	m25pe16b_Wrdi      		= 0x04,
	m25pe16b_RdID      		= 0x9F,
	m25pe16b_DP        		= 0xb9,
	m25pe16b_ResumeDP  		= 0xAB

//	at25f512b_Erase4k   	= 0x20,
//	at25f512b_Erase32k  	= 0xd8,
//	at25f512b_WrOtp     	= 0x9b,
//	at25f512b_ReadOtp   	= 0x77,
//	at25f512b_getCrc    	= 0x00,

} m25pe16b_instr ;

typedef enum _m25pe16b_dpMode{
	m25pe16b_DPon        = 0x01,
	m25pe16b_DPoff       = 0x00,
} m25pe16b_dpMode;


//***********************************************************************/
// Additional SPI Macros for the XXX */
//************************************************************************/
// Write to a register in the M25PE16

//write a single byte to spi asserting and deasserting CS
#define M25PE16_SPI_BYTE_CMD(data) \
  do {  \
	SPI_WAITFORTx_BEFORE(); \
	M25PE16_SPI_ENABLE(); \
    SPI_WRITE(data); \
    M25PE16_SPI_DISABLE(); \
  } while(0)

//start spi write asserting CS and leave asserted
#define M25PE16_SPI_START(data) \
  do {  \
	SPI_WAITFORTx_BEFORE(); \
	M25PE16_SPI_ENABLE(); \
    SPI_WRITE(data); \
  } while(0)

//byte to spi leaving cs asserted
#define M25PE16_SPI_BYTE_WRITE(data) \
  do { \
	SPI_WAITFORTx_BEFORE(); \
    SPI_WRITE(data); \
  } while(0)

/* Read a rbyte from spi*/
#define ECHO_SDI 0
#if ECHO_SDI
/* Read a rbyte from spi*/
#define M25PE16_SPI_BYTE_READ(data) \
  do { \
	SPI_WAITFORTx_BEFORE(); \
    SPI_WRITE(*data); \
    *data = (SPI_RXBUF); \
  } while(0)
#else
#define M25PE16_SPI_BYTE_READ(data) \
  do { \
	SPI_WAITFORTx_BEFORE(); \
    SPI_WRITE(0); \
    *data = (SPI_RXBUF); \
  } while(0)
#endif



//stop spi after byte write deasserting CS
#define M25PE16_SPI_STOP(data) \
  do { \
	SPI_WAITFORTx_BEFORE(); \
    SPI_WRITE(data); \
    M25PE16_SPI_DISABLE(); \
  } while(0)

#define M25PE16_WT_BUSY(busy,timeout) \
do{ \
	busy = m25pe16_Readsr() & RDY_BUSY; \
	timeout--; \
}while(busy && timeout) \


#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

uint8_t m25pe16_read(uint32_t address, uint8_t *buff, uint8_t len);
uint8_t m25pe16_write(uint32_t address, uint8_t *buff, uint8_t len);
uint8_t m25pe16_erase(m25pe16b_instr blockSize, uint32_t address);
// void m25pe16_rdOTP(uint32_t address, uint8_t *buff, uint8_t len);
// void m25pe16_wrOTP(uint32_t address, uint8_t *buff, uint8_t len);
uint8_t m25pe16_Readsr();
void m25pe16_ReadID(uint8_t *id);
void m25pe16_DP(m25pe16b_dpMode mode);
uint16_t m25pe16b_crc(uint32_t address, uint32_t len);

#endif /* M25PE16_H */
