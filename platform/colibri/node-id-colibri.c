#include "sys/node-id.h"
#include "dev/at25f512b.h"
#include "platform-conf.h"

uint32_t node_id_colibri = 0;

void node_id_restore(void) {
	node_id_colibri = 0;

	uint8_t src[4];
	uint8_t *dst = (uint8_t *)&node_id_colibri;

	//at25f512_rdOTP(XMEM_NODEID_BASEADDR, (uint8_t *)&node_id_colibri, 4);
	at25f512_rdOTP(XMEM_NODEID_BASEADDR, src, 4);

	dst[1] = src[3];
	dst[0] = src[2];
	dst[3] = src[1];
	dst[2] = src[0];
}
