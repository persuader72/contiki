/* Copyright (c) 2014, CBL Electronics srl
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
 */
#include "contiki.h"
#include "lis3mdl_mag.h"
#include "kinetic_spi.h"

//------------- magnetometer register access functions ---------------
void ReadMagReg(magRegisters_t reg, uint8_t data[], uint16_t len){
	magCmdWord_t cmd;
	cmd.bitval.rw = LIS3MDL_READ;
	cmd.bitval.multipleRead = LIS3MDL_MULTIPLE;
	cmd.bitval.add = reg;
	K_MAG_SPI_START(cmd.val);
	uint16_t i=0;
	for(i=0;i<len-1;i++){
		 K_SPI_BYTE_READ(&(data[i]));
	}
	K_MAG_SPI_READ_STOP(&(data[i]));
}

void WriteMagReg(magRegisters_t reg, uint8_t data[], uint16_t len){
	magCmdWord_t cmd;
	cmd.bitval.rw = LIS3MDL_WRITE;
	cmd.bitval.multipleRead = LIS3MDL_MULTIPLE;
	cmd.bitval.add = reg;
	K_MAG_SPI_START(cmd.val);
	uint16_t i=0;
	for(i=0;i<len-1;i++){
		 K_SPI_BYTE_WRITE((data[i]));
	}
	K_MAG_SPI_STOP((data[i]));
}
