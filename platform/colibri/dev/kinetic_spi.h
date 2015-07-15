#ifndef KINETIC_SPI_COMMON_H_
#define KINETIC_SPI_COMMON_H_

// ************************ common SPI macros **********************************
//byte to spi leaving cs asserted
#define K_SPI_BYTE_WRITE(data) \
  do { \
	SPI_WAITFORTx_BEFORE(); \
    SPI_WRITE(data); \
  } while(0)

#define K_SPI_BYTE_READ(data) \
  do { \
	SPI_WAITFORTx_BEFORE(); \
    SPI_WRITE(*data); \
    *data = (SPI_RXBUF); \
  } while(0)

#endif
