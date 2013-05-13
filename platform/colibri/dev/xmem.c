#include "contiki.h"
#include "dev/xmem.h"
#include "at25f512b.h"

void xmem_init(void) {
}

int xmem_pread(void *_p, int size, unsigned long offset) {
	at52f512_read(offset,(uint8_t *)_p,size);
	return size;
}

int xmem_pwrite(const void *_buf, int size, unsigned long addr) {
	at52f512_write(addr,(uint8_t *)_buf,size);
	at25f512_RdID(); //wait until write end
	return size;
}

int xmem_erase(long size, unsigned long addr) {
	at25f512b_instr blockSize;
	if(size == 0x1000) {
		blockSize=at25f512b_Erase4k;
	} else if(size == 0x8000) {
		blockSize=at25f512b_Erase32k;
	} else if(size >= 0x100000) {
		blockSize=at25f512b_EraseBulk;
	} else {
		return 0;
	}
	at25f512_erase(blockSize,addr);
	return size;
}
