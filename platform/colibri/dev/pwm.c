/*
 * Copyright (c) 2012, Andrea Cannavicci + Stefano Pagnottelli
 * CBL Electronics srl + Siralab srl
 * */
#include "contiki.h"
#include "pwm.h"


void pwm_setPort(uint8_t port){


	for(uint8_t i=0;i<PWM_CHANNELS;i++){
		switch(i){
		case 0:
			if (port & PWM_CH0 ){
				PWM_CH0_PORT(DIR) |= PWM_CH0_BIT;
				PWM_CH0_PORT(SEL) |= PWM_CH0_BIT;

				//P1.1 + P6.0 as input
				P1DIR &= ~BIT1; P1SEL &= ~BIT1;
				P6DIR &= ~BIT0; P6SEL &= ~BIT0;

			}
			else{
				PWM_CH0_PORT(DIR) &= ~PWM_CH0_BIT;
				PWM_CH0_PORT(SEL) &= ~PWM_CH0_BIT;
			}

			break;
		case 1:
			if (port & PWM_CH0 ){
				PWM_CH1_PORT(DIR) |= PWM_CH1_BIT;
				PWM_CH1_PORT(SEL) |= PWM_CH1_BIT;
			}
			else{
				PWM_CH1_PORT(DIR) &= ~PWM_CH1_BIT;
				PWM_CH1_PORT(SEL) &= ~PWM_CH1_BIT;
			}
			break;
		default:
			break;
		}


	}

}

void pwm_init(void){

	//moved to ioInit
	  //P1SEL |= BIT1 | BIT2 | BIT3;  // P1.2 TA0.1
	  //P1DIR |= BIT1 | BIT2 | BIT3;  // P1.2 TA0.1

	  TA0CCR0 = 128;                            // PWM Period/2

	  TA0CCTL1 = OUTMOD_7;                      // CCR1 toggle/set
	  TA0CCR1 = 32;                             // CCR1 PWM duty cycle

	  TA0CCTL2 = OUTMOD_7;                      // CCR2 toggle/set
	  TA0CCR2 = 96;                             // CCR2 PWM duty cycle
	  TA0CTL = TASSEL_2 | MC_1 | TACLR;         // SMCLK, up-down mode, clear TAR

}

void pwm_set(uint16_t pwmPeriod, uint16_t pwmDuty[]){
	  TA0CCR0 = pwmPeriod;                       // PWM Period
	  TA0CCR1 = pwmDuty[0];                      // CCR1 PWM duty cycle
	  TA0CCR2 = pwmDuty[1];                      // CCR2 PWM duty cycle

}
