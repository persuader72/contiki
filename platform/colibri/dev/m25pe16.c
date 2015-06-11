/*
   * Copyright (c) 2013, CBL Electronics srl
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
 *
 * This file is part of Colibri platform for the Contiki operating system .
 *
 * $Id: m25pe16b.c,v 1.0 2015/03/20 $
 */

/**
 * \file
 *         m25pe16b library file.
 *         This file contains all necessary functions to control m25pe16b
 *         flash memory.
 *         Memory address space is 24 bit but 32 bit unsigned int is used to
 *         store address information for C compatibility.
 *         Byte interface has been implemented.
 * \author
 *         Andrea Cannavicci <andrea.cannavicci@cblelectronics.com>
 */
#include "m25pe16.h"
#include "stdio.h"

#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while (0)
#endif

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif

/*---------------------------------------------------------------------------*/
/**
 * \brief      	     read data from flash memory
 * \param address    Memory address. Only 24 bit are used
 * \param *buff      uint8_t buffer that will contain read data
 * \param len        number of bytes to read
 * \return           void function
 *
 *             The function check flash memory status register to
 *             verify if flash is busy. After wait flash is not busy
 *             will be read len+2 bytes. Len+1 bytes will be copied
 *             in to buffer
 */

uint8_t m25pe16_read(uint32_t address, uint8_t *buff, uint8_t len){

	uint8_t busy;
	uint8_t timeout = 0xff;
	M25PE16_WT_BUSY(busy,timeout);
	if(timeout==0)
		return FALSE;

	M25PE16_SPI_START((uint8_t)m25pe16b_ReadPage);
	M25PE16_SPI_BYTE_WRITE((uint8_t)(address>>16)&0xff);
	M25PE16_SPI_BYTE_WRITE((uint8_t)(address>>8 )&0xff);
	M25PE16_SPI_BYTE_WRITE((uint8_t)(address    )&0xff);

	uint8_t i;
	for(i=0;i<=len;i++){
		M25PE16_SPI_BYTE_READ(&buff[i]);
	}
	M25PE16_SPI_STOP(0); //è una lettura dummy per chiudere il ciclo
	return TRUE;
}

// WARNING
// m25pe16 can write only 256 bytes at once. If data len exceeds 256 bytes previous data are discarded (circular buffer???)
// ToDo: data should be splitted onto 256byte-len-packets adding a zeropadding data to completely fillup the last packet (if needed)
//
uint8_t m25pe16_write(uint32_t address, uint8_t *buff, uint8_t len){
	uint8_t busy;
	uint8_t timeout = 0xff;
	M25PE16_WT_BUSY(busy,timeout);
	if(timeout == 0)
		return FALSE;

	M25PE16_SPI_BYTE_CMD((uint8_t)m25pe16b_Wren);

	M25PE16_SPI_START((uint8_t)m25pe16b_ProgramPage);
	M25PE16_SPI_BYTE_WRITE((address>>16)&0xff);
	M25PE16_SPI_BYTE_WRITE((address>>8 )&0xff);
	M25PE16_SPI_BYTE_WRITE((address    )&0xff);

	uint8_t i;
	for(i=0;i<=len;i++){
		if(i!=len)
			M25PE16_SPI_BYTE_WRITE(buff[i]);
		else
			M25PE16_SPI_STOP(buff[i]);
	}
	return TRUE;

}

uint8_t m25pe16_erase(m25pe16b_instr blockSize, uint32_t address){
	uint8_t busy;
	uint8_t timeout = 0xff;
	M25PE16_WT_BUSY(busy,timeout);
	if(timeout==0)
		return 0;

	M25PE16_SPI_BYTE_CMD((uint8_t)m25pe16b_Wren);

	if (blockSize == (uint8_t)m25pe16b_EraseBulk){
		M25PE16_SPI_BYTE_CMD((uint8_t)m25pe16b_EraseBulk);
	}
	else{
		M25PE16_SPI_START(blockSize);
		M25PE16_SPI_BYTE_WRITE((address>>16)&0xff);
		M25PE16_SPI_BYTE_WRITE((address>>8 )&0xff);
		M25PE16_SPI_STOP((address    )&0xff);
	}
	return 1;
}

/*
void m25pe16_wrOTP(uint32_t address, uint8_t *buff, uint8_t len){
	uint8_t busy;
	M25PE16_WT_BUSY(busy);

	M25PE16_SPI_BYTE_CMD((uint8_t)m25pe16b_Wren);

	M25PE16_SPI_START((uint8_t)m25pe16b_WrOtp);
	M25PE16_SPI_BYTE_WRITE((address>>16)&0xff);
	M25PE16_SPI_BYTE_WRITE((address>>8 )&0xff);
	M25PE16_SPI_BYTE_WRITE((address    )&0xff);

	uint8_t i;
	for(i=0;i<len;i++){
		if (i!=(len-1))
			M25PE16_SPI_BYTE_WRITE(buff[i]);
		else
			M25PE16_SPI_STOP(buff[i]);
	}
}

void m25pe16_rdOTP(uint32_t address, uint8_t *buff, uint8_t len){
	uint8_t busy;
	M25PE16_WT_BUSY(busy);

	M25PE16_SPI_START((uint8_t)m25pe16b_ReadOtp);
	M25PE16_SPI_BYTE_WRITE((address>>16)&0xff);
	M25PE16_SPI_BYTE_WRITE((address>>8 )&0xff);
	M25PE16_SPI_BYTE_WRITE((address    )&0xff);

	//two dummy bytes to start shift out otp bytes
	M25PE16_SPI_BYTE_WRITE(0);
	M25PE16_SPI_BYTE_WRITE(0);

	uint8_t i;
	for(i=0;i<len;i++){
		M25PE16_SPI_BYTE_READ(&buff[i]);
	}
	M25PE16_SPI_STOP(0);
}
*/

uint8_t m25pe16_Readsr(){
	uint8_t sr;
	M25PE16_SPI_START((uint8_t)m25pe16b_Rdsr);
	M25PE16_SPI_BYTE_READ(&sr);
	M25PE16_SPI_DISABLE();
	return sr;
}

void m25pe16_ReadID(uint8_t *id){
	uint8_t i;

	uint8_t busy;
	uint8_t timeout = 0xff;
	M25PE16_WT_BUSY(busy,timeout);

	M25PE16_SPI_START((uint8_t)m25pe16b_RdID);
	for(i=0;i<4;i++)
		M25PE16_SPI_BYTE_READ(&(id[i]));
	M25PE16_SPI_DISABLE();
	//printf("id: %.4x\n",id);
	return;
}

void m25pe16_DP(m25pe16b_dpMode mode){
	//FIXME: aggiungere un wait??
	if (mode == m25pe16b_DPon)
		M25PE16_SPI_BYTE_CMD((uint8_t)m25pe16b_DP); //entering dp:3us ????? THIS TIMING WAS FOR THE AT25
	else
		M25PE16_SPI_BYTE_CMD((uint8_t)m25pe16b_ResumeDP); //exiting dp: 8us ????? THIS TIMING WAS FOR THE AT25...
}


//NB len espresso in byte esatti (1 vale 1 byte!!)
uint16_t m25pe16b_crc(uint32_t address, uint32_t len){

	uint8_t buff[CRC_BLOCK];
	//memset(buff,0xff,sizeof(buff)); //riempio a ff perchè il CRC usa 16bit!
	uint16_t i=0,j;
	//len +=1;

	uint16_t blocks = (uint16_t)(len)/CRC_BLOCK;
	PRINTF("blocks: %d\n",blocks);
	PRINTF("bytes:  %d\n",len%CRC_BLOCK);


	PRINTF("  i\tbyte\tCRC\n");
	CRCINIRES = 0xFFFF;
	if(blocks){
		for(i=0;i<blocks;i++){
			PRINTF("\n");
				m25pe16_read(address+i*CRC_BLOCK,buff,CRC_BLOCK-1);
				for (j=0;j<CRC_BLOCK;j++){
					CRCDIRB = buff[j];
					//PRINTF("0x%.4X\t0x%.4X\t0x%.4X\n",i*CRC_BLOCK+j,buff[j],CRCINIRES);
					PRINTF("%.2X",buff[j]);
					if (j==15) PRINTF("\n");
				}
			}
	}

	PRINTF("\n");
	if(len%CRC_BLOCK ){
		m25pe16_read(address+i*CRC_BLOCK,buff,(len)%CRC_BLOCK);
		for (j=0;j<len%CRC_BLOCK;j++){
			CRCDIRB = buff[j];
			//PRINTF("0x%.4X\t0x%.4X\t0x%.4X\n",i*CRC_BLOCK+j,buff[j],CRCINIRES);
			PRINTF("%.2X",buff[j]);
			if (j==15) PRINTF("\n");
		}
	}

	//printf("blocchi %d\n",(uint16_t)(len/CRC_BLOCK));
	//printf("bytes %d\n",(uint16_t)((len+1)%CRC_BLOCK));
	PRINTF("\nCRC=0x%.4X\n",CRCINIRES);
	return CRCINIRES;


}
