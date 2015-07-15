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
#include "l3gd20h_gyro.h"
#include "kinetic_spi.h"

//------------- gyroscope register access functions ---------------
void ReadGyroReg(gyrRegisters_t reg, uint8_t data[], uint16_t len){
	gyroCmdWord_t cmd;
	cmd.bitval.rw = L3GD20H_READ;
	cmd.bitval.multipleRead = L3GD20H_MULTIPLE;
	cmd.bitval.add = reg;
	K_GYRO_SPI_START(cmd.val);
	uint16_t i=0;
	for(i=0;i<len-1;i++){
		K_SPI_BYTE_READ(&(data[i]));
	}
	K_GYRO_SPI_READ_STOP(&(data[i]));
}

void WriteGyroReg(gyrRegisters_t reg, uint8_t data[], uint16_t len){
	gyroCmdWord_t cmd;
	cmd.bitval.rw = L3GD20H_WRITE;
	cmd.bitval.multipleRead = L3GD20H_MULTIPLE;
	cmd.bitval.add = reg;
	K_GYRO_SPI_START(cmd.val);
	uint16_t i=0;
	for(i=0;i<len-1;i++){
		K_SPI_BYTE_WRITE((data[i]));
	}
	K_GYRO_SPI_STOP((data[i]));
}

void gyroPowerMode(gyroPwrMode_t mode){
    GYRctrlReg1_t GyrCtrlReg1;
    GyrCtrlReg1.val = 0;
    GyrCtrlReg1.bitval.PowMode = 0;
    GyrCtrlReg1.bitval.Xen = 0;
    GyrCtrlReg1.bitval.Yen = 0;
    GyrCtrlReg1.bitval.Zen = 0;
//    GyrCtrlReg1.bitval.PowMode = mode;

    WriteGyroReg(GyrAddrCtrlReg1, &GyrCtrlReg1.val, 1);
}
