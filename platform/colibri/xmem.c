#include "contiki.h"
#include "dev/xmem.h"
#include "at25f512b.h"
#if HW_TYPE==3
#include "m25pe16.h"
#endif

void xmem_init(void) {
}

int xmem_pread(void *_p, int size, unsigned long offset) {
	if(offset >= 0xFFFF00) at25f512_rdOTP(offset & 0xFF, _p, size);
	else
#if HW_TYPE==3
		if(size) //-1 is for protocol compatibilty with at52f512
			m25pe16_read(offset, _p, size-1);
#else

		at52f512_read(offset,(uint8_t *)_p,size);
#endif
	return size;
}

int xmem_pwrite(const void *_buf, int size, unsigned long addr) {
	//printf("size wr:0x%.2x\n",size);
	if(addr >= 0xFFFF00) at25f512_wrOTP(addr & 0xFF,(uint8_t *)_buf,size);
	else
#if HW_TYPE==3
		if(size) //-1 is for protocol compatibilty with at52f512
			m25pe16_write(addr,(uint8_t *)_buf,size-1);
#else
		at52f512_write(addr,(uint8_t *)_buf,size);
#endif
	//at25f512_RdID(); //wait until write end
	return size;
}

int xmem_erase(long size, unsigned long addr) {
#if HW_TYPE==3
	m25pe16b_instr blockSize;
	switch (size)
	{
	case 0x00:
		blockSize=m25pe16b_SubSectErase; //4k  nota: supportato solo a m25pe16
		break;
	case 0x01:
		blockSize=m25pe16b_SectErase;  //64k
		break;
	case 0x02:
		blockSize=m25pe16b_EraseBulk;
		break;
	default:
		return 0;
	}
	m25pe16_erase(blockSize, addr);
#else
	at25f512b_instr blockSize;
	switch (size)
	{
	case 0x00:
		blockSize=at25f512b_Erase4k;
		break;
	case 0x01:
		blockSize=at25f512b_Erase32k;
		break;
	case 0x02:
		blockSize=at25f512b_EraseBulk;
		break;
	default:
		return 0;
	}
	at25f512_erase(blockSize,addr);
	return size;
#endif
}
