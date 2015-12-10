#ifndef LIS3MDL_MAG_H_
#define LIS3MDL_MAG_H_

#include "contiki.h"
#include "dev/spi.h"
typedef union _magCmdWord_t
{   uint8_t val;
    struct
    {   							// bits   description
    	uint8_t add:6;             	// address to selected register
    	uint8_t multipleRead:1; 	// allow multiple read
    	uint8_t rw:1;				// allow read (1=rW) or write (0=Rw)
    } bitval;
}magCmdWord_t;

// ************************ MAGNETOMETER SPI macros **********************************
//start spi write asserting CS and leave asserted
#define K_MAG_SPI_START(data) \
  do {  \
	SPI_WAITFORTx_BEFORE(); \
    K_MAG_SPI_ENABLE(); \
    SPI_WRITE(data); \
  } while(0)


//stop spi after byte write deasserting CS
#define K_MAG_SPI_STOP(data) \
  do { \
	SPI_WAITFORTx_BEFORE(); \
    SPI_WRITE(data); \
    K_MAG_SPI_DISABLE(); \
  } while(0)


#define K_MAG_SPI_READ_STOP(data) \
  do { \
	SPI_WAITFORTx_BEFORE(); \
    SPI_WRITE(*data); \
    *data = (SPI_RXBUF); \
     K_MAG_SPI_DISABLE(); \
  } while(0)

#define LIS3MDL_READ  1
#define LIS3MDL_WRITE 0

#define LIS3MDL_SINGLE   0
#define LIS3MDL_MULTIPLE 1

#define MAGTYPXYMIN 0			// typical XY values interval when MAG sensor is steady and stable
#define MAGTYPXYMAX 3

#define MAGTYPZMIN 0.1			//typical Z values interval when MAG sensor is steady and stable
#define MAGTYPZMAX 1


// TOTAL SRAM ON MSP430F5310 IS 6 KBYTE
#define MAGBUFF 1				// TOTAL SAMPLES FOR MAG
#define MAGTESTSAMPLES 32		// MAX SAMPLES IN ORDER TO TEST THE MAG DATA


typedef enum _magRegisters_t{
	MagAddWhoAmI        = 0x0F,
	MagAddrCtrlReg1		= 0x20,
	MagAddrCtrlReg2 	= 0x21,
	MagAddrCtrlReg3 	= 0x22,
	MagAddrCtrlReg4 	= 0x23,
	MagAddrCtrlReg5 	= 0x24,

	MagAddrStatusReg	= 0x27,
	MagAddrOUTX_L		= 0x28,
	MagAddrOUTX_H		= 0x29,
	MagAddrOUTY_L		= 0x2A,
	MagAddrOUTY_H		= 0x2B,
	MagAddrOUTZ_L		= 0x2C,
	MagAddrOUTZ_H		= 0x2D,

	MagAddrIntCfgReg	= 0x30,
}magRegisters_t;


typedef union _MAGctrlReg1_t
{
    uint8_t val;
    struct
    {          						// bits description (bit0 to bit7)
    								//
    	uint8_t SelfTest:1;     	// self test mode 1:enabled 0:disabled
    	uint8_t Set0Thisbit:1;		// Always set this bit to 0
    	uint8_t DatatRate:3;		// ODR data rate selection default 000:0.625Hz - 001:1.25Hz ...111=80Hz
    	uint8_t XYmodeSel:2;		// axis power mode    00: low pow 01: medium pow 10:high pow 11:ultrahigh pow
    	uint8_t TempSensEn:1;		// Temper sensor 0:disabled 1:enabled
  	} bitval;
} MAGctrlReg1_t;

typedef enum _magDataRate_t{
	magDataRate_0Hz625 = 0,
	magDataRate_1Hz25  = 1,
	//TODO: verificare con datasheet
	magDataRate_2Hz5  = 2,
	magDataRate_5Hz  = 3,
	magDataRate_10Hz  = 4,
	magDataRate_20Hz  = 5,
	magDataRate_40Hz  = 6,
	magDataRate_80Hz  = 7
}magDataRate_t;



typedef union _MAGctrlReg2_t
{
    uint8_t val;
    struct
    {          						// bits description (bit0 to bit7)
    								//
    	uint8_t Set0Thisbit1:1;		// Always set this bit to 0
    	uint8_t Set0Thisbit2:1;		// Always set this bit to 0
    	uint8_t SoftRes:1;			// Soft reset 0:disabled 1:enabled
    	uint8_t Reboot:1;			// Reboot mode 0:disabled 1:enabled
    	uint8_t Set0Thisbit3:1;		// Always set this bit to 0
    	uint8_t MagFullScale:2;		// Set full scale 00:±4, 01:±8, 10:±12, 11:±16 gauss
    	uint8_t Set0Thisbit4:1;		// Always set this bit to 0
  	} bitval;
} MAGctrlReg2_t;

#define OM_CONTINOUS 0
#define OM_SINGLE    1
#define OM_POWERDOWN 3

typedef union _MAGctrlReg3_t
{
    uint8_t val;
    struct
    {          							// bits description (bit0 to bit7)
    									//
    		uint8_t OperModeSel:2;		// Operative conversion mode selection 00:continuous 01: single   10-11: powerdown
    		uint8_t SPImode:1;      	// SPI mode 0:4wire 1:3wire
    		uint8_t Set0Thisbit1:1;		// Always set this bit to 0
    		uint8_t Set0Thisbit2:1;		// Always set this bit to 0
    		uint8_t LowPow:1;			// 1:set 0.625Hz into DatatRate reg1  0:data rate is set by ctrlreg1
    		uint8_t Set0Thisbit3:1;		// Always set this bit to 0
    		uint8_t Set0Thisbit4:1;		// Always set this bit to 0

  	} bitval;
} MAGctrlReg3_t;

typedef union _MAGctrlReg4_t
{
    uint8_t val;
    struct
    {          						// bits description (bit0 to bit7)
    								//
    	uint8_t Set0Thisbit1:1;		// Always set this bit to 0
    	uint8_t BigLitEnd:1;		// Big-Little endian selection 0:SLb lower address 1:MSb lower address
    	uint8_t ZmodeSel:2;			// axis power mode    00: low pow 01: medium pow 10:high pow 11:ultrahigh pow
    	uint8_t Set0Thisbit2:1;		// Always set this bit to 0
    	uint8_t Set0Thisbit3:1;		// Always set this bit to 0
    	uint8_t Set0Thisbit4:1;		// Always set this bit to 0
    	uint8_t Set0Thisbit5:1;		// Always set this bit to 0
  	} bitval;
} MAGctrlReg4_t;


typedef union _MAGctrlReg5_t
{
    uint8_t val;
    struct
    {          						// bits description (bit0 to bit7)
    								//
    	uint8_t Set0Thisbit1:1;		// Always set this bit to 0
    	uint8_t Set0Thisbit2:1;		// Always set this bit to 0
    	uint8_t Set0Thisbit3:1;		// Always set this bit to 0
    	uint8_t Set0Thisbit4:1;		// Always set this bit to 0
    	uint8_t Set0Thisbit5:1;		// Always set this bit to 0
    	uint8_t Set0Thisbit6:1;		// Always set this bit to 0
    	uint8_t BlockDataUpd:1;		// block data update 0:continuous 1:output reg not updated until MSB&LSB have been read
    	uint8_t Set0Thisbit7:1;		// Always set this bit to 0

  	} bitval;
} MAGctrlReg5_t;

typedef union _MAGstatusReg_t
{
    uint8_t val;
    struct
    {          					// bits description (bit0 to bit7)
    							//
    	uint8_t XDA:1;       	// if 1 new X-axis data are available else not
    	uint8_t YDA:1; 			// if 1 new Y-axis data are available else not
    	uint8_t ZDA:1; 			// if 1 new Z-axis data are available else not
    	uint8_t ZYXDA:1; 		// if 1 new XYZ-axis set of data are available else not
    	uint8_t XOR:1; 		 	// if 1 new X-axis data has overwritten previous data
    	uint8_t YOR:1; 		 	// if 1 new Y-axis data has overwritten previous data
    	uint8_t ZOR:1; 		 	// if 1 new Z-axis data has overwritten previous data
    	uint8_t ZYXOR:1; 		// if 1 new XYZ-axis set of data has overwritten previous set
    } bitval;
}MAGstatusReg_t;


typedef union _MAGintCfgReg_t
{
    uint8_t val;
    struct
    {          						// bits description (bit0 to bit7)
    								//
    	uint8_t Int1En:1;      		// Interrupt on INT1 0:disabled 1:enabled
    	uint8_t LatchIntReq:1; 	 	// Int Latch 0:int req latched 1:int req not latched
    	uint8_t IntActConf:1;  		// Int active configuration on INT1   0:low 1:high
    	uint8_t Set0Thisbit1:1;		// Always set this bit to 0
    	uint8_t Set0Thisbit2:1;		// Always set this bit to 0
    	uint8_t ZintEn:1;  			// Int active on Z axis 0:disabled 1:enabled
    	uint8_t YintEn:1;  			// Int active on Y axis 0:disabled 1:enabled
    	uint8_t XintEn:1;  			// Int active on X axis 0:disabled 1:enabled
    } bitval;
}MAGintCfgReg_t;

void ReadMagReg(magRegisters_t reg, uint8_t data[], uint16_t len);
void WriteMagReg(magRegisters_t reg, uint8_t data[], uint16_t len);


#endif
