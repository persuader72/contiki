#ifndef L3GD20H_GYRO_H_
#define L3GD20H_GYRO_H_

#include "contiki.h"
#include "dev/spi.h"
typedef union _gyroCmdWord_t
{   uint8_t val;
    struct
    {   							// bits   description
    	uint8_t add:6;             	// address to selected register
    	uint8_t multipleRead:1; 	// allow multiple read
    	uint8_t rw:1;				// allow read (1=rW) or write (0=Rw)
    } bitval;
}gyroCmdWord_t;


// ************************ GYRO SPI macros **********************************
//start spi write asserting CS and leave asserted
#define K_GYRO_SPI_START(data) \
  do {  \
	SPI_WAITFORTx_BEFORE(); \
    K_GYRO_SPI_ENABLE(); \
    SPI_WRITE(data); \
  } while(0)


//stop spi after byte write deasserting CS
#define K_GYRO_SPI_STOP(data) \
  do { \
	SPI_WAITFORTx_BEFORE(); \
    SPI_WRITE(data); \
    K_GYRO_SPI_DISABLE(); \
  } while(0)


#define K_GYRO_SPI_READ_STOP(data) \
  do { \
	SPI_WAITFORTx_BEFORE(); \
    SPI_WRITE(*data); \
    *data = (SPI_RXBUF); \
    K_GYRO_SPI_DISABLE(); \
  } while(0)


#define L3GD20H_READ  1
#define L3GD20H_WRITE 0

#define L3GD20H_SINGLE   0
#define L3GD20H_MULTIPLE 1

#define GYRTYPMIN 0			//typical values interval when GYR sensor is steady and stable
#define GYRTYPMAX 300

// TOTAL SRAM ON MSP430F5310 IS 6 KBYTE
#define GYRBUFF 32		// TOTAL SAMPLES FOR GYRO
#define GYRWTM 16		// GYRO FIFO WATERMARK LEVEL

typedef enum _gyroPwrMode_t{
	gyroPwr_PD = 0,
	gyroPwr_Sleep = 1
}gyroPwrMode_t;

typedef enum _gyrRegisters_t{
	GyrAddrCtrlReg1 = 0x20,
	GyrAddrCtrlReg2 = 0x21,
	GyrAddrCtrlReg3 = 0x22,
	GyrAddrCtrlReg4 = 0x23,
	GyrAddrCtrlReg5 = 0x24,

	GyrAddrOutTemp = 0x26,
	GyrAddrWhoAmI	= 0x0F,
	GyrAddrStatusReg = 0x27,
	GyrAddrOUTX_L	 = 0x28, // X (roll) angular data rate low
	GyrAddrOUTX_H	 = 0x29, // X (roll) angular data rate high
	GyrAddrOUTY_L	 = 0x2A, // Y (pitch) angular data rate low
	GyrAddrOUTY_H	 = 0x2B, // Y (pitch) angular data rate high
	GyrAddrOUTZ_L	 = 0x2C, // Z (yaw) angular data rate low
	GyrAddrOUTZ_H	 = 0x2D, // Z (yaw) angular data rate high

	GyrAddrFIFOCTRLreg	= 0x2E,
	GyrAddrFIFOsrcreg	= 0x2F,

	GyrAddr_IGTHRX_H	= 0x32,
	GyrAddr_IGTHRX_L	= 0x33,
	GyrAddr_IGTHRY_H	= 0x34,
	GyrAddr_IGTHRY_L	= 0x35,
	GyrAddr_IGTHRZ_H	= 0x36,
	GyrAddr_IGTHRZ_L	= 0x37,

	GYRAddrDurReg		= 0x38,	// registry managing int duration & threshold
	GyrAddrLowODRreg	= 0x39,
}gyrRegisters_t;



typedef union _GYRctrlReg1_t
{
    uint8_t val;
    struct
    {          					// bits description (bit0 to bit7)
    							//
    	uint8_t Yen:1;       	// 1:Yaxis Gyro sens is enabled (pitch)
    	uint8_t Xen:1; 			// 1:Xaxis Gyro sens is enabled (roll)  //TODO: verificare la corrisponenza dei bit per X e Y il dateasheet non è chiaro potrebbero essere al contrario
    	uint8_t Zen:1; 			// 1:Zaxis Gyro sens is enabled (yaw)
    	uint8_t PowMode:1; 		// 0:powerdown 1:normal for sleep mode set [PowMode Zen Xen Zen:sleep
    	uint8_t BW:2; 		 	// BandWidth (see table 21) should be 11
    	uint8_t DR_sel:2; 		// Data Rate (see table 21) should be 11
    	} bitval;
} GYRctrlReg1_t;
#define GYR_CTRL_REG1_DEF 0x07

typedef enum _gyroDrSel_t{
	gyroDR_12Hz5 = 0,
	gyroDR_25Hz  = 1,
	gyroDR_50Hz  = 2,
	gyroDR_100Hz  = 3,
	gyroDR_200Hz  = 4,
	gyroDR_400Hz  = 5,
	gyroDR_800Hz  = 6,
}gyroDrSel_t;

typedef union _GYRctrlReg2_t
{
    uint8_t val;
    struct
    {          					// bits description (bit0 to bit7)
    							//
    	uint8_t HPfCutOff:4;    // HighPass Filter CutOff frequency. See table 26
    	uint8_t HPfModeSel:2; 	// HighPassFilter Mode selection. See table 25
    	uint8_t LevTrig:1; 		// 1:Level trigger enabled
    	uint8_t ExtTrig:1; 		// 1 external trigger enabled
     	} bitval;
} GYRctrlReg2_t;

typedef union _GYRctrlReg3_t
{
    uint8_t val;
    struct
    {          					// bits description (bit0 to bit7)
    							//
    	uint8_t Int2Empty:1;    // Raise irq2 on DRDY/INT2 pin. FIFO is empty 1:enabled
    	uint8_t Int2OVR:1; 		// Raise irq2 on DRDY/INT2 pin. FIFO Overrun 1:enabled
    	uint8_t Int2FTH:1; 		// FIFO threshold on DRDY/INT2 pin. samples are more than N 1:enabled
    	uint8_t Int2DRDY:1; 	// Data Ready on DRDY/INT2 pin. 1: enabled
    	uint8_t PP_OD_sel:1; 	// 0:Push-Pull 1: Open Drain
    	uint8_t HL_IntAct:1; 	// Interrupts active on L or H 0:high 1:low
    	uint8_t Int1Boot:1; 	// Boot status available on INT1 0:disabled 1:enabled
    	uint8_t Int1_IG:1; 		// Interrupt enabled on INT1 pin. 0:disabled 1:enabled
     	} bitval;
} GYRctrlReg3_t;
#define GYR_CTRL_REG3_DEF 0x00

typedef union _GYRctrlReg4_t
{
    uint8_t val;
    struct
    {          					// bits description (bit0 to bit7)
    							//
       	uint8_t SPImode:1;      // SPI mode 0:4wire 1:3wire
        uint8_t SelfTest:2;     // Self Test mode 00:disabled 01:ST0 10:-- 11:ST1
        uint8_t IMPen:1;	    // Level Sensitive latch enable 0:disable 1:enable
        uint8_t FullScale:2;    // Fullscale selection (see table default value 00)
        uint8_t BLE:1;      	// Big/Little Endian def 0:LSb at lower address 1:MSb at lower address
        uint8_t BlockDataUpd:1;      	// block data update 0:continuos 1:output reg not updated until MSB&LSB have been read
    } bitval;
} GYRctrlReg4_t;
typedef enum _gyrScale_t{
	gyrScale_245dps = 0,
	gyrScale_500dps = 1,
	gyrScale_2000dps = 2,
}gyrScale_t;

typedef union _GYRctrlReg5_t
{
    uint8_t val;
    struct
    {          					// bits description (bit0 to bit7)
    							//
       	uint8_t OUTconf_sel:2;  //
       	uint8_t INTgen_sel:2;   //
       	uint8_t HPen:1;   		// High Pass Filter 0:disable 1:enable
       	uint8_t StopOnFTH:1;    // 0:max FIFO depth is 32 samples 1:FIFO depth limited by FIFOFTH (0x2E)
       	uint8_t FIFOen:1;   	// 0:FIFO disabled 1:FIFO enabled
       	uint8_t Boot:1;   		// 0:normal mode 1:reboot memory mode
    } bitval;
} GYRctrlReg5_t;

typedef uint8_t GYRReference_t;

typedef union _GYRstatusReg_t
{
    uint8_t val;
    struct
    {          					// bits description (bit0 to bit7)
    							//
    	uint8_t XDA:1;       	// if 1 new X-axis data are available else not
    	uint8_t YDA:1; 			// if 1 new Y-axis data are available else not
    	uint8_t ZDA: 1; 		// if 1 new Z-axis data are available else not
    	uint8_t ZYXDA:1; 		// if 1 new XYZ-axis set of data are available else not
    	uint8_t XOR:1; 		 	// if 1 new X-axis data has overwritten previous data
    	uint8_t YOR:1; 		 	// if 1 new Y-axis data has overwritten previous data
    	uint8_t ZOR:1; 		 	// if 1 new Z-axis data has overwritten previous data
    	uint8_t ZYXOR:1; 		// if 1 new XYZ-axis set of data has overwritten previous set
    } bitval;
}GYRstatusReg_t;

typedef union _GYRFIFOctrlReg_t
{
    uint8_t val;
    struct
    {          					// bits description (bit0 to bit7)
    							//
    uint8_t FIFOMaxSamples:5;   // Watermark Level
    uint8_t FIFOFM:3;      		// FIFO mode 000=bypass 001=FIFO 010=Stream 011=Stream-to-FIFO
     							// FIFO mode 100=bypasstostream 110=dynstream 111=bypass-to-FIFO
    } bitval;
}GYRFIFOctrlReg_t;

typedef enum _gyrFifoMode_t{
	gyrFifo_Bypass = 0,
	gyrFifo_Fifo = 1,
	gyrFifo_Stream = 2,
	gyrFifo_Stream2Fifo = 3,
	gyrFifo_Bypass2Stream = 4,
	gyrFifo_DynamicStream = 6,
	gyrFifo_Bypass2Fifo = 7
}gyrFifoMode_t;

typedef union _GYRFIFOsrcReg_t
{
    uint8_t val;
    struct
    {          					// bits description (bit0 to bit7)
    							//
    	uint8_t FIFONumSampReady:5;  // number of unread samples in FIFO buffer (compare to watermark level)
    	uint8_t FIFOisEmpty:1;      // 1=all samples read and FIFO buffer is empty
    								//  0=FIFO contains unreaded samples
    	uint8_t FIFOisFull:1;       // 1=FIFO buffer is full (32 samples ready to read)
    								// 0=when at least one sample has been read
    	uint8_t FIFOhasNsamples:1;  // 1=FIFO buffer exceeds watermark level (number of samples previously set<32)
    } bitval;
}GYRFIFOsrcReg_t;

typedef union _GYRLowODRReg_t
{
    uint8_t val;
    struct
    {          					// bits description (bit0 to bit7)
    							//
    	uint8_t LowODR:1;  		//
    	uint8_t Set1Thisbit1:1; // This bit must be set to 1
    	uint8_t SW_Res:1;       // Software reset mode 0:normal 1:softreset (return to 0 on next flash boot)
    	uint8_t SPIOnly:1;  	// 0: SPI&I2C enabled 1:SPI enabled i2C disabled
    	uint8_t Set1Thisbit2:1; // This bit must be set to 1
    	uint8_t DRDY_HL:1;  	// DRDY INT2 active level 0:DRDY active high 1:DRDY active low
    	uint8_t unusedbit1:1;  	//
    	uint8_t unusedbit2:1;  	//
    } bitval;
}GYRLowODRReg_t;

typedef union _GYRDurReg_t
{
    uint8_t val;
    struct
    {          					// bits description (bit0 to bit7)
    							//
    	uint8_t IG_Duration:7;  // 7 bit for IntGeneration counting samples
    	uint8_t Wait:1; 		// enable wait counting samples after Int falls under the threshold value

    } bitval;
} GYRDurReg_t;


typedef union _GYRIGMSbTHRXReg_t
{
    uint8_t val;
    struct
    {          					// bits description (bit0 to bit7)
    							//
    	uint8_t MSbTHSX:7;  	// 7 bit MSb for INT threshold X value 000 0000
    	uint8_t DCRM:1; 		// Threshold mode 0:reset 1:decrement

    } bitval;
} GYRIGMSbTHRXReg_t;

typedef union _GYRIGLSbTHRXReg_t
{
    uint8_t val;
    struct
    {          					// bits description (bit0 to bit7)
    							//
    	uint8_t LSbTHSX:8;  	// 8 bit LSb for INT threshold X value 0000 0000

    } bitval;
} GYRIGLSbTHRXReg_t;


typedef union _GYRIGMSbTHRYReg_t
{
    uint8_t val;
    struct
    {          					// bits description (bit0 to bit7)
    							//
    	uint8_t MSbTHSY:7;  	// 7 bit MSb for INT threshold Y value 000 0000
    	uint8_t unusedbit1:1; 	// unused bit

    } bitval;
} GYRIGMSbTHRYReg_t;

typedef union _GYRIGLSbTHRYReg_t
{
    uint8_t val;
    struct
    {          					// bits description (bit0 to bit7)
    							//
    	uint8_t LSbTHSY:8;  	// 8 bit LSb for INT threshold Y value 0000 0000

    } bitval;
} GYRIGLSbTHRYReg_t;


typedef union _GYRIGMSbTHRZReg_t
{
    uint8_t val;
    struct
    {          					// bits description (bit0 to bit7)
    							//
    	uint8_t MSbTHSZ:7;  	// 7 bit MSb for INT threshold Z value 000 0000
    	uint8_t unusedbit1:1; 	// unused bit

    } bitval;
} GYRIGMSbTHRZReg_t;

typedef union _GYRIGLSbTHRZReg_t
{
    uint8_t val;
    struct
    {          					// bits description (bit0 to bit7)
    							//
    	uint8_t LSbTHSZ:8;  	// 8 bit LSb for INT threshold Z value 0000 0000

    } bitval;
} GYRIGLSbTHRZReg_t;

typedef struct _gyrRegs_t{

	GYRctrlReg1_t  GyrCtrlReg1;
	GYRctrlReg2_t  GyrCtrlReg2;
	GYRctrlReg3_t  GyrCtrlReg3;
	GYRctrlReg4_t  GyrCtrlReg4;
	GYRctrlReg5_t  GyrCtrlReg5;
	GYRReference_t GyrReference;

	//GYRFIFOsrcReg_t GyrFIFOsrcReg;
	GYRFIFOctrlReg_t GyrFIFOctrlReg;
	GYRLowODRReg_t   GyrLowODRReg;
}gyrRegs_t;

void gyroPowerMode(gyroPwrMode_t mode);

#endif
