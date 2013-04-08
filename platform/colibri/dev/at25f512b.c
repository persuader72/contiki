
#include "dev/at25f512b.h"
#include "stdio.h"


void at52f512_read(uint32_t address, uint8_t *buff, uint8_t len){

	uint8_t busy;
	AT25F512_WT_BUSY(busy);

	AT25F512_SPI_START((uint8_t)at25f512b_ReadPage);
	AT25F512_SPI_BYTE_WRITE((uint8_t)(address>>16)&0xff);
	AT25F512_SPI_BYTE_WRITE((uint8_t)(address>>8 )&0xff);
	AT25F512_SPI_BYTE_WRITE((uint8_t)(address    )&0xff);

	uint8_t i;
	for(i=0;i<len+1;i++){
		AT25F512_SPI_BYTE_READ(&buff[i]);
	}
	AT25F512_SPI_STOP(0);
}

void at52f512_write(uint32_t address, uint8_t *buff, uint8_t len){

	uint8_t busy;
	AT25F512_WT_BUSY(busy);

	AT25F512_SPI_BYTE_CMD((uint8_t)at25f512b_Wren);

	AT25F512_SPI_START((uint8_t)at25f512b_WritePage);
	AT25F512_SPI_BYTE_WRITE((address>>16)&0xff);
	AT25F512_SPI_BYTE_WRITE((address>>8 )&0xff);
	AT25F512_SPI_BYTE_WRITE((address    )&0xff);

	uint8_t i;
	for(i=0;i<len+1;i++){
		if(i!=len)
			AT25F512_SPI_BYTE_WRITE(buff[i]);
		else
			AT25F512_SPI_STOP(buff[i]);
	}

}

void at25f512_erase(at25f512b_instr blockSize, uint32_t address){
	uint8_t busy;
	AT25F512_WT_BUSY(busy);

	AT25F512_SPI_BYTE_CMD((uint8_t)at25f512b_Wren);

	if (blockSize == (uint8_t)at25f512b_EraseBulk){
		AT25F512_SPI_BYTE_CMD((uint8_t)at25f512b_EraseBulk);
	}
	else{
		AT25F512_SPI_START(blockSize);
		AT25F512_SPI_BYTE_WRITE((address>>16)&0xff);
		AT25F512_SPI_BYTE_WRITE((address>>8 )&0xff);
		AT25F512_SPI_STOP((address    )&0xff);
	}

}


void at25f512_wrOTP(uint32_t address, uint8_t *buff, uint8_t len){
	uint8_t busy;
	AT25F512_WT_BUSY(busy);

	AT25F512_SPI_BYTE_CMD((uint8_t)at25f512b_Wren);

	AT25F512_SPI_START((uint8_t)at25f512b_WrOtp);
	AT25F512_SPI_BYTE_WRITE((address>>16)&0xff);
	AT25F512_SPI_BYTE_WRITE((address>>8 )&0xff);
	AT25F512_SPI_BYTE_WRITE((address    )&0xff);

	uint8_t i;
	for(i=0;i<len;i++){
		if (i!=(len-1))
			AT25F512_SPI_BYTE_WRITE(buff[i]);
		else
			AT25F512_SPI_STOP(buff[i]);
	}
}

void at25f512_rdOTP(uint32_t address, uint8_t *buff, uint8_t len){
	uint8_t busy;
	AT25F512_WT_BUSY(busy);

	AT25F512_SPI_START((uint8_t)at25f512b_ReadOtp);
	AT25F512_SPI_BYTE_WRITE((address>>16)&0xff);
	AT25F512_SPI_BYTE_WRITE((address>>8 )&0xff);
	AT25F512_SPI_BYTE_WRITE((address    )&0xff);

	//two dummy bytes to start shift out otp bytes
	AT25F512_SPI_BYTE_WRITE(0);
	AT25F512_SPI_BYTE_WRITE(0);

	uint8_t i;
	for(i=0;i<len;i++){
		AT25F512_SPI_BYTE_READ(&buff[i]);
	}
	AT25F512_SPI_STOP(0);
}

uint8_t at25f512_Rdsr(){
	uint8_t sr;
	AT25F512_SPI_START((uint8_t)at25f512b_Rdsr);
	AT25F512_SPI_BYTE_READ(&sr);
	AT25F512B_SPI_DISABLE();
	return sr;
}

uint16_t at25f512_RdID(){
	uint16_t id;
	uint8_t i;

	uint8_t busy;
	AT25F512_WT_BUSY(busy);

	AT25F512_SPI_START((uint8_t)at25f512b_RdID);
	for(i=0;i<2;i++)
		AT25F512_SPI_BYTE_READ((((uint8_t*)&id)+i));
	AT25F512B_SPI_DISABLE();
	//printf("id: %.4x\n",id);
	return id;
}

void at25f512_DP(at25f512b_dpMode mode){
	//FIXME: aggiungere un wait??
	if (mode == at25f512b_DPon)
		AT25F512_SPI_BYTE_CMD((uint8_t)at25f512b_DP); //entering dp:3us
	else
		AT25F512_SPI_BYTE_CMD((uint8_t)at25f512b_ResumeDP); //exiting dp: 8us
}

uint16_t at25f512b_crc(uint32_t address, uint32_t len){





	uint8_t buff[CRC_BLOCK];
	//memset(buff,0xff,sizeof(buff)); //riempio a ff perchè il CRC usa 16bit!
	int i=0,j;
	len +=1;

	uint16_t blocks = (len+1)/CRC_BLOCK;
	PRINTF("blocks: %d\n",blocks);
	PRINTF("bytes:  %d\n",len%CRC_BLOCK);


	PRINTF("  i\tbyte\tCRC\n");
	CRCINIRES = 0xFFFF;
	if(blocks){
		for(i=0;i<blocks;i++){
			PRINTF("\n");
				at52f512_read(address+i,buff,CRC_BLOCK);
				for (j=0;j<CRC_BLOCK;j++){
					CRCDIRB = buff[j];
					PRINTF("+ 0x%.2X\t0x%.4X\t0x%.4X\n",i+j,buff[j],CRCINIRES);
				}
			}
	}

	PRINTF("\n");
	if(len%CRC_BLOCK ){
		at52f512_read(address+i*CRC_BLOCK,buff,(len-1)%CRC_BLOCK);
		for (j=0;j<len%CRC_BLOCK;j++){
			CRCDIRB = buff[j];
			PRINTF("- 0x%.2X\t0x%.4X\t0x%.4X\n",i*CRC_BLOCK+j,buff[j],CRCINIRES);
		}
	}

	//printf("blocchi %d\n",(uint16_t)(len/CRC_BLOCK));
	//printf("bytes %d\n",(uint16_t)((len+1)%CRC_BLOCK));
	printf("CRC=0x%.4X\n",CRCINIRES);
	return CRCINIRES;


}
