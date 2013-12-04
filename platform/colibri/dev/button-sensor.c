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

const struct sensors_sensor button_sensor;

static struct timer debouncetimer;
static int status(int type);

HWCONF_PIN(BUTTON, 1, 3);
HWCONF_IRQ(BUTTON, 1, 3);

/*---------------------------------------------------------------------------*/
ISR(PORT1, irq_p1)
{
  P1OUT |= 0x80;
  ENERGEST_ON(ENERGEST_TYPE_IRQ);


  if(BUTTON_CHECK_IRQ()) {
    //while(1) { BUTTON_CHECK_IRQ(); }
    if(timer_expired(&debouncetimer)) {
      timer_set(&debouncetimer, CLOCK_SECOND / 4);
      sensors_changed(&button_sensor);
      //P1OUT &= ~0x20;
      LPM4_EXIT;
    }
  }
  P1OUT &= ~0x80;
  P1IFG = 0x00;
  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}
/*---------------------------------------------------------------------------*/
static int value(int type) {
  return BUTTON_READ() || !timer_expired(&debouncetimer);
}
/*---------------------------------------------------------------------------*/
static int configure(int type, int c) {
  switch (type) {
  case SENSORS_ACTIVE:
    if (c) {
      if(!status(SENSORS_ACTIVE)) {
    	  timer_set(&debouncetimer, 0);
    	  BUTTON_IRQ_EDGE_SELECTD();
    	  BUTTON_SELECT();
    	  BUTTON_MAKE_INPUT();
    	  BUTTON_ENABLE_IRQ();
    	  P1DIR &= ~(1 << 2);
    	  P6DIR &= ~(1 << 0);

    	  P3DIR |= (1 << 1);
    	  P3OUT |= (1 << 1);
      }
    } else {
    	BUTTON_DISABLE_IRQ();
    }
    return 1;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static int status(int type) {
  switch (type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY: return BUTTON_IRQ_ENABLED();
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(button_sensor, BUTTON_SENSOR,value, configure, status);
