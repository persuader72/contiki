/*
 * Copyright (c) 2012, Swedish Institute of Computer Science
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
#include <stdio.h>
#include <string.h>

#include "mrf49xa.h"
#include "adc.h"
//#include "dev/ds2411.h"
#include "dev/leds.h"
#include "dev/serial-line.h"
#include "dev/slip.h"
#include "dev/uart1.h"
#include "dev/watchdog.h"
#include "dev/xmem.h"
#include "dev/infomem.c"
#include "lib/random.h"
#include "net/netstack.h"
#include "net/mac/frame802154.h"

#if WITH_UIP6
#include "net/uip-ds6.h"
#endif /* WITH_UIP6 */

#include "net/rime.h"

#include "node-id.h"
#include "sys/autostart.h"
#include "sys/profile.h"
#include "dev/pwm.h"

#ifndef WITH_UIP
#define WITH_UIP 0
#endif

void radio_setup(void){
	uint8_t baud = INFOMEM_STRUCT_A->radio.baudRate == 0xFF ? MRF49XA_57600 : INFOMEM_STRUCT_A->radio.baudRate;
	uint8_t txpwr = INFOMEM_STRUCT_A->radio.txPower == 0xFF ? MRF49XA_TXPWR_0DB : INFOMEM_STRUCT_A->radio.txPower;
	//uint8_t rssi = INFOMEM_STRUCT_A->radio.minRssi == 0xFF ? MRF49XA_RSSI_79DB : INFOMEM_STRUCT_A->radio.minRssi;
	uint8_t band = INFOMEM_STRUCT_A->radio.band == 0xFF ? MRF49XA_BAND_868 : INFOMEM_STRUCT_A->radio.band;
	uint8_t channel = INFOMEM_STRUCT_A->radio.channel > MRF49XA_MAX_CHANNEL ? MRF49XA_DEF_CHANNEL : INFOMEM_STRUCT_A->radio.channel;

	//mrf49xa_setDataRate(baud,band);
	//mrf49xa_setTxPwr(txpwr);
	//mrf49xa_setRxRssi(MRF49XA_RSSI_79DB,MRF49XA_LNA_0DB);

	mrf49xa_init(baud,txpwr,band,channel);
}

void init_platform(void);

#ifndef RF_CHANNEL
#define RF_CHANNEL              26
#endif

static void set_rime_addr(void) {
	int i; rimeaddr_t n_addr;

	memcpy(&n_addr, INFOMEM_STRUCT_A->addresses.rimeAddr, sizeof(rimeaddr_t));
	rimeaddr_set_node_addr(&n_addr);

	printf("Rime started with address ");
	for(i = 0; i < sizeof(n_addr.u8) - 1; i++) printf("%d.",n_addr.u8[i]);
	printf("%d\n", n_addr.u8[i]);
}

#if !PROCESS_CONF_NO_PROCESS_NAMES
static void
print_processes(struct process * const processes[])
{
  /*  const struct process * const * p = processes;*/
  printf("Starting");
  while(*processes != NULL) {
    printf(" '%s'", (*processes)->name);
    processes++;
  }
  putchar('\n');
}
#endif /* !PROCESS_CONF_NO_PROCESS_NAMES */

int main(int argc, char **argv) {
	msp430_cpu_init();
	clock_init();
	leds_init();
	adc_init();

	leds_on(LEDS_RED);
	clock_wait(2);
	uart1_init(115200);
	clock_wait(1);
	leds_on(LEDS_GREEN);
	clock_wait(1);
	leds_on(LEDS_BLUE);
	leds_off(LEDS_RED);
	rtimer_init();

	pwm_init();
	pwm_set(INFOMEM_STRUCT_A->pwmSets.pwmPeriod,
			INFOMEM_STRUCT_A->pwmSets.pwmDuty);


	/* Hardware initialization done! */
	//node_id_restore(); // Ripristina il node_id dalla memoria
	                   // FIXME: da memorizzare in una memoria non riscrivibile.

  /* for setting "hardcoded" IEEE 802.15.4 MAC addresses */
#ifdef IEEE_802154_MAC_ADDRESS
  {
    uint8_t ieee[] = IEEE_802154_MAC_ADDRESS;
    //memcpy(ds2411_id, ieee, sizeof(uip_lladdr.addr));
    //ds2411_id[7] = node_id & 0xff;
  }
#endif

  random_init(INFOMEM_STRUCT_A->addresses.nodeId[1]);
  leds_off(LEDS_BLUE);

  process_init(); // Initialize Contiki and our processes.
  process_start(&etimer_process, NULL);
  ctimer_init();
  init_platform();
  set_rime_addr();
  radio_setup();


  {
    uint8_t longaddr[8];
    memcpy(longaddr,INFOMEM_STRUCT_A->addresses.macAddr,sizeof(INFOMEM_STRUCT_A->addresses.macAddr));
    printf("MAC %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x ",
           longaddr[0], longaddr[1], longaddr[2], longaddr[3],
           longaddr[4], longaddr[5], longaddr[6], longaddr[7]);
  }

  printf(CONTIKI_VERSION_STRING " started. ");
  if(INFOMEM_STRUCT_A->addresses.nodeId[1] > 0 && INFOMEM_STRUCT_A->addresses.nodeId[1]<0xFFFF) {
    printf("Node id is set to %u.\n", INFOMEM_STRUCT_A->addresses.nodeId[1]);
  } else {
    printf("Node id is not set.\n");
  }


  NETSTACK_RDC.init();
  NETSTACK_MAC.init();
  NETSTACK_NETWORK.init();

  printf("%s %s, channel check rate %lu Hz, radio channel %u\n",
         NETSTACK_MAC.name, NETSTACK_RDC.name,
         CLOCK_SECOND / (NETSTACK_RDC.channel_check_interval() == 0? 1:
                         NETSTACK_RDC.channel_check_interval()),
         RF_CHANNEL);

  uart1_set_input(serial_line_input_byte);
  serial_line_init();
  leds_off(LEDS_GREEN);

#if TIMESYNCH_CONF_ENABLED
  timesynch_init();
  timesynch_set_authority_level((rimeaddr_node_addr.u8[0] << 4) + 16);
#endif /* TIMESYNCH_CONF_ENABLED */

  energest_init();
  ENERGEST_ON(ENERGEST_TYPE_CPU);
  watchdog_start();
  watchdog_stop();

  putchar('\n');
  autostart_start(autostart_processes);


  while(1) {
    int r;
    do {
      watchdog_periodic();
      r = process_run();
    } while(r > 0);

    /* Idle processing. */
    int s = splhigh();		/* Disable interrupts. */
    /* uart1_active is for avoiding LPM3 when still sending or receiving */
    if(process_nevents() != 0 || uart1_active()) {
      splx(s);                  /* Re-enable interrupts. */
    } else {
      static unsigned long irq_energest = 0;
      /* Re-enable interrupts and go to sleep atomically. */
      ENERGEST_OFF(ENERGEST_TYPE_CPU);
      ENERGEST_ON(ENERGEST_TYPE_LPM);
      /* We only want to measure the processing done in IRQs when we
	 are asleep, so we discard the processing time done when we
	 were awake. */
      energest_type_set(ENERGEST_TYPE_IRQ, irq_energest);
      watchdog_stop();
      _BIS_SR(GIE | SCG0 | SCG1 | CPUOFF); /* LPM3 sleep. This
                                              statement will block
                                              until the CPU is
                                              woken up by an
                                              interrupt that sets
                                              the wake up flag. */

      /* We get the current processing time for interrupts that was
         done during the LPM and store it for next time around.  */
      dint();
      irq_energest = energest_type_time(ENERGEST_TYPE_IRQ);
      eint();
      watchdog_start();
      ENERGEST_OFF(ENERGEST_TYPE_LPM);
      ENERGEST_ON(ENERGEST_TYPE_CPU);
    }
  }
}