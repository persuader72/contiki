/*
 * Copyright (c) 2012, Andrea Cannavicci + Stefano Pagnottelli
 * CBL Electronics srl + Siralab srl
 * */
#include "contiki.h"
#include "pwm.h"


void pwm_init(void){

	  P1SEL |= BIT2;  // P1.2 TA0.1

	  TA0CCR0 = 128;                            // PWM Period/2

	  TA0CCTL1 = OUTMOD_6;                      // CCR1 toggle/set
	  TA0CCR1 = 32;                             // CCR1 PWM duty cycle

	  TA0CCTL2 = OUTMOD_6;                      // CCR2 toggle/set
	  TA0CCR2 = 96;                             // CCR2 PWM duty cycle
	  TA0CTL = TASSEL_2 | MC_3 | TACLR;         // SMCLK, up-down mode, clear TAR

}
