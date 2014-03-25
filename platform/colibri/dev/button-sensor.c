/*
 * Copyright (c) 2005, Swedish Institute of Computer Science
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
 */

#include "contiki.h"
#include "lib/sensors.h"
#include "dev/hwconf.h"
#include "dev/button-sensor.h"
#include "isr_compat.h"
#include "sys/cc.h"

//#define DBG
#ifdef DBG
#define REMOTE_CMD 1
#define BATTERY_CHARGER 1
#endif

//posizione del bit dello switch nella variabile
//#define BC_BIT 0
//#define WU_BIT 1

static struct timer debouncetimer;
const struct sensors_sensor button_sensor;


static CC_INLINE int notIrq() {return 0;}
#define NOT_IRQ notIrq

static int status(int type);
#if REMOTE_CMD
//SW3 - wake up button
HWCONF_PIN(BUTTON_WU, 1, 0);
HWCONF_IRQ(BUTTON_WU, 1, 0);
#define WU_IRQ BUTTON_WU_CHECK_IRQ
#else
#define WU_IRQ NOT_IRQ
#endif


#if BATTERY_CHARGER
HWCONF_PIN(BUTTON_BC, 1, 3);
HWCONF_IRQ(BUTTON_BC, 1, 3);
#define BC_IRQ BUTTON_BC_CHECK_IRQ
#else
#define BC_IRQ NOT_IRQ
#endif

/*---------------------------------------------------------------------------*/
ISR(PORT1, irq_p1)
{
  P1OUT |= 0x80;
  ENERGEST_ON(ENERGEST_TYPE_IRQ);


  if(WU_IRQ() || BC_IRQ()) {
    //while(1) { BUTTON_CHECK_IRQ(); }
    if(timer_expired(&debouncetimer)) {
      timer_set(&debouncetimer, CLOCK_SECOND / 4);
      sensors_changed(&button_sensor);
      //P1OUT &= ~0x80;
      LPM4_EXIT;
    }
  }
  P1IFG = 0x00;
  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}
/*---------------------------------------------------------------------------*/
static int value(int type) {
	int status = 0;
	// NB la lettura è invertita. Il bottone premuto vale 0
#if BATTERY_CHARGER
	status |= (BUTTON_BC_READ() ? 0 : BC_BIT);
#endif
#if REMOTE_CMD
	status |= (BUTTON_WU_READ() ? 0 : WU_BIT) ;
#endif
  return status; // || !timer_expired(&debouncetimer);
}
/*---------------------------------------------------------------------------*/
static int configure(int type, int c) {
  switch (type) {
  case SENSORS_ACTIVE:
    if (c) {
      if(!status(SENSORS_ACTIVE)) {
    	  P1IFG = 0x00;
    	  timer_set(&debouncetimer, 0);
#if REMOTE_CMD
    	  BUTTON_WU_IRQ_EDGE_SELECTD();
    	  BUTTON_WU_SELECT();
    	  BUTTON_WU_MAKE_INPUT();
    	  BUTTON_WU_ENABLE_IRQ();
#endif
#if BATTERY_CHARGER
    	  BUTTON_BC_IRQ_EDGE_SELECTD();
    	  BUTTON_BC_SELECT();
    	  BUTTON_BC_MAKE_INPUT();
    	  BUTTON_BC_ENABLE_IRQ();
#endif
/*    	  P1DIR &= ~(1 << 2);
    	  P6DIR &= ~(1 << 0);

    	  P3DIR |= (1 << 1);
    	  P3OUT |= (1 << 1);*/
      }
    } else {
#if REMOTE_CMD
    	BUTTON_WU_DISABLE_IRQ();
#endif
#if BATTERY_CHARGER
    	BUTTON_BC_DISABLE_IRQ();
#endif
    }
    return 1;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static int status(int type) {
  switch (type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY: return 0; //BUTTON_IRQ_ENABLED();
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(button_sensor, BUTTON_SENSOR,value, configure, status);
