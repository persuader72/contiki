#include <stdio.h>

void infomem_read(void *buffer, unsigned int offset, unsigned char size) {
	printf("infomem_read o:%d l:%d\n",offset,size);
}
void infomem_write(void *buffer, unsigned int offset, unsigned char size) {
	printf("infomem_read o:%d l:%d\n",offset,size);
}
