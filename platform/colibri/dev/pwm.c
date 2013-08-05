/*
 * Copyright (c) 2012, Andrea Cannavicci + Stefano Pagnottelli
 * CBL Electronics srl + Siralab srl
 * */
#include "contiki.h"
#include "pwm.h"


void pwm_init(void){

	//moved to ioInit
	  P1SEL |= BIT4; // | BIT2 | BIT3;  // P1.2 TA0.1
	  P1DIR |= BIT4;// | BIT2 | BIT3;  // P1.2 TA0.1

	  TA0CCR0 = 128;                            // PWM Period/2

	  TA0CCTL1 = OUTMOD_7;                      // CCR1 toggle/set
	  TA0CCR1 = 32;                             // CCR1 PWM duty cycle

	  TA0CCTL3 = OUTMOD_7;                      // CCR3 toggle/set
	  TA0CCR3 = 96;                             // CCR3 PWM duty cycle
	  TA0CTL = TASSEL_2 | MC_1 | TACLR;         // SMCLK, up-down mode, clear TAR

}

void pwm_set(uint16_t pwmPeriod, uint16_t pwmDuty[]){
	  TA0CCR0 = pwmPeriod;                       // PWM Period
	  TA0CCR1 = pwmDuty[0];                      // CCR1 PWM duty cycle
	  TA0CCR3 = pwmDuty[1];                      // CCR2 PWM duty cycle

}
