/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
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
 * This file is part of the Contiki operating system.
 *
 * $Id: hello-world.c,v 1.1 2006/10/02 21:46:46 adamdunkels Exp $
 */

/**
 * \file
 *         A very simple Contiki application showing how Contiki programs look
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include "net/rime.h"
#include "node-id.h"
#include "random.h"
#include "serial-line.h"
#include <stdio.h>
#include <string.h>

#include <dev/leds.h>

//#define TMOTE_MYMOTE
#ifndef TMOTE_MYMOTE
#include <string.h>
#include "cooja_infomem.h"
#else
#include <dev/infomem.h>
#include <dev/mrf49xa.h>
#endif

#define MAX_NEIGHBORS 32

#define MAX_FRAME_PAYLOAD		64
#define MAX_INFOMEM_BLOCK_SIZE	30
#define MAX_INFOMEM_BUFFER_SIZE 32


/**
 * Commands:
 *
 * 02: Request: Start discovery procedure
 * 03: Reply: Discovery procedure ended
 * 06: Request: Number of neighbors in table
 * 07aa: Reply: Number of neighbors in table  is the aa table size
 * 08aa: Request: Table row aa with neighbor information
 * 09aaaaBBBBccDD: Reply: neighbor information
 * 		aaaa: Remote node network addresss
 * 		BBBB: Remote node received rssi
 * 		cc: Sequence number
 * 		DD: Sequence number gap
 * 0AaaaaBB...: Send a block data to a remote node with address aaaa BB is the payload
 * 0C send multi hop
 * 0E send broadcast
 *
 * 10aaBB: Request: read BB bytes from node info memory starting at offset aa
 * 11aaBBcc...: Reply: cc... are the readed BB bytes from node info memory starting at offset aa.
 * 12aaBBcc...: Request: write BB bytes from node info memory starting at offset aa.
 * 		cc... are the bytes to wrte
 * 13aaBB: Reply: writed BB bytes from node info memory starting at offset aa.
 *
 * 14
 * 15
 */

#define RQS_DISCOVERY_START		0x02
#define RPL_DISCOVERY_END		0x03
#define RQS_ADEVERTISE_YOU		0x04
#define RPL_ADEVERTISE_ME		0x05
#define RQS_NEIGHBORS_COUNT		0x06
#define RPL_NEIGHBORS_COUNT		0x07
#define RQS_NEIGHBOR_GET		0x08
#define RPL_NEIGHBOR_GET		0x09

#define RQS_REMOTE_UNIC_COMMAND 0x0A
#define ACK_REMOTE_UNIC_COMMAND 0x0F
#define RQS_REMOTE_MHOP_COMMAND 0x0C
#define ACK_REMOTE_MHOP_COMMAND 0x0F
#define RQS_REMOTE_BRDC_COMMAND 0x0E
#define ACK_REMOTE_BRDC_COMMAND 0x0F

#define RQS_INFOMEM_READ	    0x10
#define RPL_INFOMEM_READ	    0x11
#define RQS_INFOMEM_WRITE		0x12
#define RPL_INFOMEM_WRITE		0x13

#define RQS_NODEID_FILTER		0x14
#define RPL_NODEID_FILTER		0x15

#define RQS_BOOTLOADER 			0xF0
#define RPL_BOOTLOADER 			0xF1


struct msg_generic {
	uint8_t command;
	uint8_t params[32];
};

struct msg_advertise_me {
	uint8_t command;
	uint8_t seqno;
	uint16_t rssi;
};

/* This structure holds information about neighbors. */
struct neighbor {
  struct neighbor *next;
  rimeaddr_t addr;
  uint16_t last_rssi;
  uint8_t last_seqno;
  uint8_t avg_seqno_gap;
};

struct reply_info_t {
	uint8_t command;
	uint8_t hops_count;
	rimeaddr_t from;
	rimeaddr_t hops[16];
};

MEMB(neighbors_memb, struct neighbor, MAX_NEIGHBORS);
LIST(neighbors_list);

/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process, "Hello world process");
AUTOSTART_PROCESSES(&hello_world_process);
/*---------------------------------------------------------------------------*/

static struct etimer et;
static struct broadcast_conn broadcast;
static struct unicast_conn unicast;
static struct multihop_conn multihop;

uint8_t seqno;
static uint8_t delayedRssi;
static struct reply_info_t replyData;

static void process_request(uint8_t *,uint8_t);

#ifdef TMOTE_MYMOTE
void callBootLoader(){
	clock_wait(3);	// wait 10ms for oscillator to stablize
	dint();
	((void (*)())0x1000)();
	eint();
}
#endif

static uint8_t decode_hex_byte(uint8_t *from) {
	uint8_t b=0;
	if(from[0]>='0'&&from[0]<='9') b=(from[0]-'0')<<4;
	else if(from[0]>='A'&&from[0]<='F') b=(from[0]-'A'+10)<<4;
	else if(from[0]>='a'&&from[0]<='f') b=(from[0]-'a'+10)<<4;
	if(from[1]>='0'&&from[1]<='9') b+=from[1]-'0';
	else if(from[1]>='A'&&from[1]<='F') b+=from[1]-'A'+10;
	else if(from[1]>='a'&&from[1]<='f') b+=(from[1]-'a'+10);
	return b;
}

static struct neighbor *neighbor_get(uint8_t i) {
	struct neighbor *n=NULL;
	for(n = list_head(neighbors_list); n != NULL; n = list_item_next(n)) if(!i--) break;
	return n;
}

static void reply_info(uint8_t cmd,const rimeaddr_t *from,uint8_t hops) {
	replyData.command=cmd;
	rimeaddr_copy(&replyData.from,from);
	replyData.hops_count=hops;
}


/**
 * frame_replay: Invia i dati di rispondere decidendo in modo inteliggente se devono
 * essere inviati via seriale o via radio. Nel caso di invio via radio la fnzione
 * decide in modo autonomo se il pacchetto deve essere spedito in unicast o in multihop.
 * In caso di multihop il sistema utilizzera la rotta contenuta nel pacchetto sorgente
 * utilizzata nel verso opposto.
 *
 * Quando viene ricevuto un paccatto vengono salvati nella variabile globale replyData
 * le informazioni sul mittente e l'eventuale rotta. Tali informazioni vengono utilizzate
 * per rimandare la risposta al mittente.
 *
 * uint8_t cmd        : Id della risposta
 * uint8_t *payload   : Payload associato alla risposta
 * uint8_t payloadlen : Lunghezza complessiva del payload
 * int reply          : Se vale 0 la risposta verra indirizzata alla seriale anche se
 *                      proviene dal canale radio
 */
static void frame_reply(uint8_t cmd,uint8_t *payload,uint8_t payloadlen,int reply) {
	int i=0;

	uint8_t frame[MAX_FRAME_PAYLOAD];

	if(replyData.hops_count) {
		frame[i++]=replyData.hops_count|0x80;
		memcpy(frame+i,replyData.hops,sizeof(rimeaddr_t)*replyData.hops_count);
		i+=sizeof(rimeaddr_t)*replyData.hops_count;
	}

	frame[i++]=cmd;
	if(payloadlen) {
		memcpy(frame+i,payload,payloadlen);
		i+=payloadlen;
	}

	uint8_t framelen = i;
	if(reply && !rimeaddr_cmp(&replyData.from,&rimeaddr_null)) {
		//printf("add %d%d\n",replyData.from.u8[0],replyData.from.u8[1]);
		//printf("h %d\n",replyData.hops_count);
		packetbuf_copyfrom(frame,framelen);
		leds_toggle(LEDS_RED);
		if(!replyData.hops_count) unicast_send(&unicast,&replyData.from); else multihop_send(&multihop,&replyData.from);
	} else {
		if(rimeaddr_cmp(&replyData.from,&rimeaddr_null)) printf("0000");
		else printf("%02X%02X",replyData.from.u8[0],replyData.from.u8[1]);
		printf("%02X",cmd);
		if(payloadlen) for(i=0;i<payloadlen;i++) printf("%02X",payload[i]);
		putchar('\n');
		leds_toggle(LEDS_BLUE);
	}

}

static void handle_advertise_me(const rimeaddr_t *from) {
	struct neighbor *n;
	struct msg_advertise_me *m;

	m = packetbuf_dataptr();
	for(n = list_head(neighbors_list); n != NULL; n = list_item_next(n)) {
		if(rimeaddr_cmp(&n->addr, from)) {
			break;
		}
	}
	//printf("a\n");
	if(n == NULL) {
		n = memb_alloc(&neighbors_memb);
		if(n != NULL) list_add(neighbors_list, n);
	}

	if(n == NULL) return;

	rimeaddr_copy(&n->addr, from);
	n->last_rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);
	n->last_seqno = m->seqno;
}

static void handle_infomem_read(uint8_t *data,uint8_t datalen) {
	if(datalen>=2) {
		uint8_t buffer[MAX_INFOMEM_BUFFER_SIZE];
		buffer[0] = data[0];
		buffer[1] = data[1]>MAX_INFOMEM_BLOCK_SIZE ? MAX_INFOMEM_BLOCK_SIZE : data[1];
		infomem_read(buffer+2,buffer[0],buffer[1]);
		frame_reply(RPL_INFOMEM_READ,buffer,buffer[1]+2,1);
	}
}

static void handle_infomem_write(uint8_t *data,uint8_t datalen) {
	if(datalen>=2) {
		infomem_write(data+2,data[0],data[1]);
		frame_reply(RPL_INFOMEM_WRITE,data,2,1);
	}
}

static void handle_nodeid_filter(uint8_t *data,uint8_t datalen) {
	int i=0; uint8_t *np=(uint8_t *)&node_id; uint8_t ns=sizeof(node_id);
	if(datalen>ns)
		for(i=0;i<ns;i++) {
			if(np[i]!=data[i]) break;
		}

	if(i==ns)
		process_request(data+ns,datalen-ns);
}

static void delayed_commands(void) {
	struct msg_advertise_me msg;
	switch(replyData.command) {
	case RQS_DISCOVERY_START:
		// Rispondi al mittente (seriale o radio)
		frame_reply(RPL_DISCOVERY_END,NULL,0,1);
		break;
	case RQS_ADEVERTISE_YOU:
		msg.command=RPL_ADEVERTISE_ME;
		msg.rssi=delayedRssi;
		msg.seqno=seqno;
		packetbuf_copyfrom(&msg,sizeof(struct msg_advertise_me));
		broadcast_send(&broadcast);
		leds_toggle(LEDS_RED);
		seqno++;
		break;
	case RQS_BOOTLOADER:
#ifdef TMOTE_MYMOTE
		callBootLoader();
#endif
		break;
	}
}

static void process_request(uint8_t *data,uint8_t datalen) {
	uint8_t arg;
	struct msg_generic gen;
	struct neighbor *n;
	rimeaddr_t to;

	leds_toggle(LEDS_GREEN);

	switch(data[0]) {
	case RQS_DISCOVERY_START:
		//list_init(neighbors_list);
		gen.command=RQS_ADEVERTISE_YOU;
		packetbuf_copyfrom(&gen,sizeof(struct msg_generic));
		leds_toggle(LEDS_RED);
		broadcast_send(&broadcast);
		PROCESS_CONTEXT_BEGIN(&hello_world_process);
		etimer_set(&et, CLOCK_SECOND * 5 + CLOCK_SECOND/2);
		PROCESS_CONTEXT_END(&hello_world_process);
		break;
	case RQS_ADEVERTISE_YOU:
		PROCESS_CONTEXT_BEGIN(&hello_world_process);
		etimer_set(&et, CLOCK_SECOND * 1 + (random_rand() % (CLOCK_SECOND * 4)));
		PROCESS_CONTEXT_END(&hello_world_process);
		break;
	case RQS_NEIGHBORS_COUNT:
		arg=list_length(neighbors_list);
		frame_reply(RPL_NEIGHBORS_COUNT,&arg,1,1);
		break;
	case RQS_NEIGHBOR_GET:
		n=neighbor_get(data[1]);
		frame_reply(RPL_NEIGHBOR_GET,((uint8_t *)n)+sizeof(struct neighbor *),sizeof(struct neighbor)-sizeof(struct neighbor *),1);
		break;
	case RQS_REMOTE_UNIC_COMMAND:
		to.u8[0]=data[1]; to.u8[1]=data[2];
		packetbuf_copyfrom(data+3,datalen-3);
		unicast_send(&unicast,&to);
		break;
	case RQS_REMOTE_MHOP_COMMAND:
		to.u8[0]=data[1]; to.u8[1]=data[2];
		packetbuf_copyfrom(data+3,datalen-3);
		multihop_send(&multihop,&to);
		break;
	case RQS_REMOTE_BRDC_COMMAND:
		packetbuf_copyfrom(data+1,datalen-1);
		broadcast_send(&broadcast);
		break;
	case RQS_INFOMEM_READ:
		handle_infomem_read(data+1,datalen-1);
		break;
	case RQS_INFOMEM_WRITE:
		handle_infomem_write(data+1,datalen-1);
		break;
	case RQS_NODEID_FILTER:
		handle_nodeid_filter(data+1,datalen-1);
		break;
	case RQS_BOOTLOADER:
		frame_reply(RPL_BOOTLOADER,&arg,1,1);
		PROCESS_CONTEXT_BEGIN(&hello_world_process);
		etimer_set(&et, CLOCK_SECOND/10);
		PROCESS_CONTEXT_END(&hello_world_process);
		break;
	default:
		// Rimanda su seriale i comandi sconosciuti
		frame_reply(data[0],data+1,datalen-1,0);
		break;
	}
}

static void broadcast_recv(struct broadcast_conn *c, const rimeaddr_t *from) {
	uint16_t datalen=packetbuf_datalen();
	uint8_t *data=(uint8_t *)packetbuf_dataptr();
	//printf("b\n");
	if(data[0]==RPL_ADEVERTISE_ME) handle_advertise_me(from);
	else {
		reply_info(data[0],from,0);
		process_request(data,datalen);
	}
}

static const struct broadcast_callbacks broadcast_call = {broadcast_recv};

static void unicast_recv(struct unicast_conn *c, const rimeaddr_t *from) {
	uint16_t datalen=packetbuf_datalen();
	uint8_t *data=(uint8_t *)packetbuf_dataptr();

	reply_info(data[0],from,0);
	process_request(data,datalen);
}

static rimeaddr_t *multihop_forward(struct multihop_conn *c,const rimeaddr_t *originator, const rimeaddr_t *dest,const rimeaddr_t *prevhop, uint8_t hops) {
	//uint16_t datalen=packetbuf_datalen();
	uint8_t *data=(uint8_t *)packetbuf_dataptr();

	uint8_t hops_dir = data[0] & 0x80;
	uint8_t hops_max = data[0] & 0x7F;
	rimeaddr_t *hops_addr = (rimeaddr_t *)(data+1);
	//printf("multihop forward dir:%d size:%d curr:%d\n", hops_dir,hops_max,hops);

	rimeaddr_t *target = hops>=hops_max ? (rimeaddr_t *)dest : hops_dir==0 ? hops_addr+hops : hops_addr+hops_max-hops-1;
	//printf("multihop target %d %d\n", target->u8[0],target->u8[1]);

	return target;
}

static void multihop_recv(struct multihop_conn *c, const rimeaddr_t *sender, const rimeaddr_t *prevhop, uint8_t hops) {
	uint16_t datalen=packetbuf_datalen();
	uint8_t *data=(uint8_t *)packetbuf_dataptr();

	uint8_t hops_max = data[0] & 0x7F;
	uint8_t route_size = sizeof(rimeaddr_t)*hops_max+1;
	reply_info(data[route_size],sender,hops_max);
	if(hops_max) memcpy(replyData.hops,data+1,route_size-1);
	process_request(data+route_size,datalen-route_size);
}

static const struct unicast_callbacks unicast_call = {unicast_recv};
static const struct multihop_callbacks multihop_call = {multihop_recv, multihop_forward};

PROCESS_THREAD(hello_world_process, ev, data) {
	PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
	PROCESS_BEGIN();

	broadcast_open(&broadcast, 129, &broadcast_call);
	unicast_open(&unicast, 130, &unicast_call);
	multihop_open(&multihop, 131, &multihop_call);

#ifdef TMOTE_MYMOTE
	mrf49xa_setDataRate(MRF49XA_9600);
	mrf49xa_setTxPwr(MRF49XA_TXPWR_0DB);
	mrf49xa_setRxRssi(MRF49XA_RSSI_79DB,MRF49XA_LNA_0DB);
#endif

//#define DBG_MSG
#ifdef DBG_MSG
	etimer_set(&et, CLOCK_SECOND);
#endif
	while(1) {
		PROCESS_WAIT_EVENT();
		if(ev == serial_line_event_message && data != NULL) {
			uint8_t i,j=0; uint8_t len=strlen(data); uint8_t decoded[32];
			for(i=0;i<len;i+=2) decoded[j++]=decode_hex_byte(data+i);

			if(j) {
				reply_info(decoded[0],&rimeaddr_null,0);
				process_request(decoded,j);
			}
		} else if(etimer_expired(&et)) {
			if(replyData.command) delayed_commands();
#ifdef DBG_MSG
//			char buff[]="0A060006";
			char buff[]="0A06000D04\n";
			uint8_t i,j=0; uint8_t len=strlen(buff); uint8_t decoded[32];
			for(i=0;i<len;i+=2) decoded[j++]=decode_hex_byte(buff+i);

			if(j) {
				reply_info(decoded[0],&rimeaddr_null,0);
				process_request(decoded,j);
			}
			printf("sent\n");
			etimer_set(&et, CLOCK_SECOND);
#endif
		}

	}
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
