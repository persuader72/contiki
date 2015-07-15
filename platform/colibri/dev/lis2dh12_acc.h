#ifndef LIS2DH12_ACC_H_
#define LIS2DH12_ACC_H_

#include "contiki.h"
#include "dev/spi.h"
typedef union _accCmdWord_t
{   uint8_t val;
    struct
    {   							// bits   description
    	uint8_t add:6;             	// address to selected register
    	uint8_t multipleRead:1; 	// allow multiple read
    	uint8_t rw:1;				// allow read (1=rW) or write (0=Rw)
    } bitval;
}accCmdWord_t;

// ************************ ACCELLEROMETER SPI macros **********************************
//start spi write asserting CS and leave asserted
#define K_ACC_SPI_START(data) \
  do {  \
	SPI_WAITFORTx_BEFORE(); \
    K_ACC_SPI_ENABLE(); \
    SPI_WRITE(data); \
  } while(0)


//stop spi after byte write deasserting CS
#define K_ACC_SPI_STOP(data) \
  do { \
	SPI_WAITFORTx_BEFORE(); \
    SPI_WRITE(data); \
    K_ACC_SPI_DISABLE(); \
  } while(0)


#define K_ACC_SPI_READ_STOP(data) \
  do { \
	SPI_WAITFORTx_BEFORE(); \
    SPI_WRITE(*data); \
    *data = (SPI_RXBUF); \
     K_ACC_SPI_DISABLE(); \
  } while(0)

#define LIS12DH_READ  1
#define LIS12DH_WRITE 0

#define LIS12DH_SINGLE   0
#define LIS12DH_MULTIPLE 1

// TOTAL SRAM ON MSP430F5310 IS 6 KBYTE
#define ACCBUFF 32		// TOTAL SAMPLES FOR ACCELLEROMETER
#define ACCWTM 15		// ACCELLEROMETER FIFO WATERMARK LEVEL

#define ACCTYPMIN 17	// Typical values interval when ACC sensor is in test mode
#define ACCTYPMAX 500	// MAX 360

#define TEST_START_VALID_SAMPLES 8
//#define NORMAL_START_VALID_SAMPLES 0



typedef enum _accRegisters_t{
	AccAddrStatusRegAux = 0x07,
	AccAddrOutTempL     = 0x0C,
	AccAddrOutTempH     = 0x0D,
	AccAddrWhoAmI       = 0x0F,
	AccAddrTempCfgReg	= 0x1F,

	AccAddrCtrlReg1		= 0x20,
	AccAddrCtrlReg3 	= 0x22,
	AccAddrCtrlReg4 	= 0x23,
	AccAddrCtrlReg5 	= 0x24,

	AccAddrStatusReg	= 0x27,
	AccAddrOUTX_L		= 0x28,
	AccAddrOUTX_H		= 0x29,
	AccAddrOUTY_L		= 0x2A,
	AccAddrOUTY_H		= 0x2B,
	AccAddrOUTZ_L		= 0x2C,
	AccAddrOUTZ_H		= 0x2D,

	AccAddrInt1CfgReg	= 0x30,		// movement recognition register with threshold value raise int2
	AccAddrInt1SrcReg	= 0x31,		// read-only register describing int2 events on XYZ threshold
	AccAddrInt1Thres	= 0x32,		// threshold values for int2
	AccAddrInt1DurReg	= 0x33,		// duration value for int2

	AccAddrInt2CfgReg	= 0x34,		// movement recognition register with threshold value raise int2
	AccAddrInt2SrcReg	= 0x35,		// read-only register describing int2 events on XYZ threshold
	AccAddrInt2Thres	= 0x36,		// threshold values for int2
	AccAddrInt2DurReg	= 0x37,		// duration value for int2

	AccAddrFIFOCTRLreg	= 0x2E,
	AccAddrFIFOSRCreg	= 0x2F,
}accRegisters_t;

#define ACCTEMPERCFGREG_DEF 0x00
typedef union _ACCTemperCfgReg_t
{
    uint8_t val;
    struct
    {          						// bits description (bit0 to bit7)
    	uint8_t Always0bit1:1;      // Bit is always 0
    	uint8_t Always0bit2:1;      // Bit is always 0
    	uint8_t Always0bit3:1;      // Bit is always 0
    	uint8_t Always0bit4:1;      // Bit is always 0
    	uint8_t Always0bit5:1;      // Bit is always 0
    	uint8_t Always0bit6:1;      // Bit is always 0
    	uint8_t AccTempEn:2;      	// Enable Temperature sensor 00: disabled 11:enabled 01:?? 10:??
    } bitval;
} ACCTemperCfgReg_t;


typedef union _statusRegAux_t
{
    uint8_t val;
    struct
    {          					// bits description (bit0 to bit7)
    	uint8_t spare0:2;       // 0    OVER VOLTAGE ALLARM
    	uint8_t TDA:1 ; 		//temperature data available
    	uint8_t spare1: 3;
    	uint8_t TOR:1; 			//temperature data overrun
    	uint8_t spare2:1;
    } bitval;
}statusRegAux_t;




typedef union _ACCstatusReg_t
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
}ACCstatusReg_t;

#define FIFOCTRLREG_DEF 0x00
typedef union _AccFIFOctrlReg_t
{
    uint8_t val;
    struct
    {          					  // bits description (bit0 to bit7)
    							  //
    	uint8_t FIFOMaxSamples:5; // Watermark Level
    	uint8_t FIFOTR:1;         // Trigger selection 0=triggers on INT1
    							  // Trigger selection 1=triggers on INT2
    	uint8_t FIFOFM:2;         // FIFO mode 00=bypass 01=FIFO 10=Stream 11=Stream-to-FIFO

    } bitval;
}AccFIFOctrlReg_t;

typedef union _AccFIFOsrcReg_t
{
    uint8_t val;
    struct
    {          					// bits description (bit0 to bit7)
    							//
    uint8_t FIFONumSampReady:5; // number of unread samples in FIFO buffer (compare to watermark level)
    uint8_t FIFOisEmpty:1;      // 1=all samples read and FIFO buffer is empty
    							//  0=FIFO contains unreaded samples
    uint8_t FIFOisFull:1;       // 1=FIFO buffer is full (32 samples ready to read)
    							// 0=when at least one sample has been read
    uint8_t FIFOhasNsamples:1;  // 1=FIFO buffer exceeds watermark level (number of samples previously set<32)
    } bitval;
}AccFIFOsrcReg_t;

#define ACCCTRLREG1_DEF 0x07
typedef enum _DR_sel_t{
	drSel_PowerDown = 0,
	drSel_1Hz       = 1,
	drSel_10Hz      = 2,
	drSel_25Hz      = 3,
	drSel_50Hz      = 4,
	drSel_100Hz     = 5,
	drSel_200Hz     = 6,
	drSel_400Hz     = 7,
	drSel_1620Hz    = 8,
	drSel_1344Hz    = 9,
}DR_sel_t;
typedef union _AccCtrlReg1_t
{
    uint8_t val;
    struct
    {          					// bits description (bit0 to bit7)
    							//
    	uint8_t X_En:1;   	// 1=enable X-axis sensor   0=disable X-axis sensor
    	uint8_t Y_En:1;   	// 1=enable Y-axis sensor   0=disable Y-axis sensor
    	uint8_t Z_En:1;   	// 1=enable Z-axis sensor   0=disable Z-axis sensor
    	uint8_t LowPowEn:1; // 1=enable LowPower Mode   0=Normal Mode
    	uint8_t DR_Sel:4;   // Data Rate Selection (HR-Normal=1,344 KHz 1001
    } bitval;
}AccCtrlReg1_t;

#define ACCCTRLREG3_DEF 0x00
typedef union _AccCtrlReg3_t
{
    uint8_t val;
    struct
    {          					// bits description (bit0 to bit7)
    							//
    	uint8_t spare0:1;       //
    	uint8_t I1_FIFO_OVR:1;   	// FIFO_OVR event rise INT1 - 1=enable 0=disable
    	uint8_t I1_FIFO_WTM:1;   	// FIFO Watermark rise INT1 - 1=enable 0=disable
    	uint8_t I1_DRDY2:1;   		//   - 1=enable 0=disable
    	uint8_t I1_DRDY1:1;   		//   - 1=enable 0=disable
    	uint8_t I1_AOI2:1;   		//   - 1=enable 0=disable
    	uint8_t I1_AOI1:1;   		//   - 1=enable 0=disable
    	uint8_t I1_CLICK:1;   		//   CLICK int on INT1 - 1=enable 0=disable
    } bitval;
}AccCtrlReg3_t;

#define ACCCTRLREG4_DEF 0x00
typedef union _AccCtrlReg4_t
{
    uint8_t val;
    struct
    {          					// bits description (bit0 to bit7)
    							//
    	uint8_t SPImode:1;      // SPI mode 0:4wire 1:3wire
    	uint8_t SelfTest:2;     // Self Test mode 00:disabled 01:enabled ST0 10:enabled ST1 11:--
    	uint8_t HighRes:1;      // High Resolution mode 0:disabled 1:enabled
    	uint8_t FullScale:2;    // Fullscale selection (see table default value 00)
    	uint8_t BLE:1;      	// Big/Little Endian def 0:LSb at lower address 1:MSb at lower address (only in HR mode)
    	uint8_t BlockDataUpd:1;	// block data update 0:continuous 1:output reg not updated until MSB&LSB have been read

    } bitval;
}AccCtrlReg4_t;
#define SCALE_2G_DIV 16

#define ACCCTRLREG5_DEF 0x00
typedef union _AccCtrlReg5_t
{
    uint8_t val;
    struct
    {          					// bits description (bit0 to bit7)
    							//
       	uint8_t D4D_INT2:1;  	//
       	uint8_t LIR_INT2:1;   	//
       	uint8_t D4D_INT1:1;   	// High Pass Filter 0:disable 1:enable
       	uint8_t LIR_INT1:1;    	// 0:max FIFO depth is 32 samples 1:FIFO depth limited by FIFOFTH (0x2E)
       	uint8_t unusedbit1:1;   //
       	uint8_t unusedbit2:1;	//
       	uint8_t FIFOen:1;   	// 0:FIFO disabled 1:FIFO enabled
       	uint8_t Boot:1;   		// 0:normal mode 1:reboot memory mode
    } bitval;
} AccCtrlReg5_t;

#define ACCINT2CFGREG_DEF 0x00
typedef union _AccInt2CfgReg_t
{
    uint8_t val;
    struct
    {          					// bits description (bit0 to bit7)
    							//
       	uint8_t XLowIrqEn:1;  	// Low  Event on X axis threshold      0:disable int 1:enable int
       	uint8_t XHigIrqEn:1;   	// High Event on X axis threshold      0:disable int 1:enable int
       	uint8_t YLowIrqEn:1;   	// Low  Event on Y axis threshold      0:disable int 1:enable int
       	uint8_t YHigIrqEn:1;   	// High Event on Y axis threshold      0:disable int 1:enable int
       	uint8_t ZLowIrqEn:1;	// Low  Event on Z axis threshold      0:disable int 1:enable int
       	uint8_t ZHigIrqEn:1;	// High Event on Z axis threshold      0:disable int 1:enable int
       	uint8_t SixdirEn:1;   	// 6 direction 0:motion 1:position
       	uint8_t BoolEvent:1;    // Boolean configuration between motion and position 0:OR 1:AND
    } bitval;
} AccInt2CfgReg_t;

typedef union _AccInt2SrcReg_t // read-only register
{
    uint8_t val;
    struct
    {          					// bits description (bit0 to bit7)
    							//
       	uint8_t XLowEvent:1;  	// Low  Event on X axis 	0:disable int 	1:low event occurred
       	uint8_t XHigEvent:1;   	// High Event on X axis 	0:disable int 	1:low event occurred
       	uint8_t YLowEvent:1;   	// Low  Event on Y axis 	0:disable int 	1:low event occurred
       	uint8_t YHigEvent:1;   	// High Event on Y axis 	0:disable int 	1:low event occurred
       	uint8_t ZLowEvent:1;	// Low  Event on Z axis 	0:disable int 	1:low event occurred
       	uint8_t ZHigEvent:1;	// High Event on Z axis 	0:disable int 	1:low event occurred
       	uint8_t Int2Active:1;   // Int 2 is active			0: no int already generated 	1: one ore more int has been generated
       	uint8_t Always0bit:1;   // always 0 bit
    } bitval;
} AccInt2SrcReg_t;

#define ACCINT2THRESREG_DEF 0x00
typedef union _AccInt2ThresReg_t //
{
    uint8_t val;
    struct
    {          					// bits description (bit0 to bit7)
    							//
       	uint8_t Int2Thres:7;  	// 7 bit describing int2 threshold
								// 1 LSb = 16 mg @ FS = 2 g
								// 1 LSb = 32 mg @ FS = 4 g
								// 1 LSb = 62 mg @ FS = 8 g
								// 1 LSb = 186 mg @ FS = 16 g
       	uint8_t Always0bit:1;   // Always 0 bit

    } bitval;
} AccInt2ThresReg_t;

typedef union _AccInt2DurReg_t // read-only register
{
    uint8_t val;
    struct
    {          					// bits description (bit0 to bit7)
    							//
       	uint8_t Int2Dur:7;  	// 7 bit describing int2 duration
								// 1 LSb = Int2Dur/ODR
       							// minimum duration of the Int2 event to be recognized
       	uint8_t Always0bit:1;   // Always 0 bit

    } bitval;
} AccInt2DurReg_t;

typedef struct _accRegs_t{
	AccFIFOctrlReg_t AccFIFOctrlReg;
	AccFIFOsrcReg_t AccFIFOsrcReg;
	AccCtrlReg1_t AccCtrlReg1;
	AccCtrlReg3_t AccCtrlReg3;
	AccCtrlReg4_t AccCtrlReg4;
	AccCtrlReg5_t AccCtrlReg5;
	ACCTemperCfgReg_t AccTemperCfgReg;
	int16_t AccTemper;
	uint8_t AccWhoIs;
}accRegs_t;




#endif
