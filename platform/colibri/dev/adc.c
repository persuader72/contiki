/*
 * Copyright (c) 2012, Andrea Cannavicci + Stefano Pagnottelli
 * CBL Electronics srl + Siralab srl
 * */
#include "contiki.h"
#include "adc.h"

static uint8_t adcStatus;

uint8_t getAdcStatus(){
	return adcStatus;
}

void adc_init(void)
{
	  //WDTCTL = WDTPW | WDTHOLD;                 // Stop WDT

	  // ADCCLK è il clock interno dell'ADC pari a circa 4.8MHZ (200ns)
	  // Configure ADC10 - Pulse sample mode; ADC10SC trigger
	  ADC10CTL0 = ADC10SHT_2 | ADC10ON;         // 16 ADC10CLKs; ADC ON
	  ADC10CTL1 = ADC10SHP | ADC10CONSEQ_0;     // s/w trig, single ch/conv
	  ADC10CTL2 = ADC10RES;                     // 10-bit conversion results
	  ADC10MCTL0 = ADC10SREF_1 | ADC10INCH_10;  // AVcc/2

	  // Configure internal reference
	  while(REFCTL0 & REFGENBUSY);              // If ref generator busy, WAIT
	  REFCTL0 |= REFVSEL_2|REFON;               // Select internal ref = 2.5V
	                                            // Internal Reference ON
	  clock_delay(600);                         // Delay (~75us) for Ref to settle
	  adcStatus = 0;
}

// start  adc a bassa priorità
uint8_t start_adc(ADC_CH channel){
	if (adcStatus & HIGH_PRIORITY_LOCK)
		return 0;       //l'adc è impegnato in una conversione ad alta priorità quindi esco senza fare nulla

	if ((ADC10MCTL0 &  0xf ) != channel){
		ADC10MCTL0 &=  0xfff0;
		ADC10MCTL0 |= channel;  // imposto il canale
	    clock_delay(100);                         // Delay (~3us) for channel to settle
	}

	ADC10CTL0 |= ADC10ENC | ADC10SC;        // Sampling and conversion start
	adcStatus = LOW_PRIORITY_LOCK;          // indico che è in corso una conversione a bassa priorita
	return 1;

}
// verifica dello stato dell'ADC
// ritorna vero se l'ADC è busy
uint8_t checkAdcBusy(){
    return (ADC10CTL1 & ADC10BUSY);
}

// lettura del dato campionato
uint16_t getAdcSample(){
	adcStatus = 0;
	return ADC10MEM0;
}

//Misure singola dell'adc ad alta priorità
uint16_t get_adc(ADC_CH channel){
	adcStatus = DIRTY | HIGH_PRIORITY_LOCK;
    while (ADC10CTL1 & ADC10BUSY);          // ADC10BUSY? attendo la fine di una precedente conversione avviata

	if ((ADC10MCTL0 &  0xf ) != channel){
		ADC10MCTL0 &=  0xfff0;
		ADC10MCTL0 |= channel;  // imposto il canale
	    clock_delay(100);                         // Delay (~3us) for channel to settle
	}
    ADC10CTL0 |= ADC10ENC | ADC10SC;        // Sampling and conversion start
    while (ADC10CTL1 & ADC10BUSY);          // ADC10BUSY?
    adcStatus = DIRTY;
    return ADC10MEM0;

}

void adcOff(){
	  // ADCCLK è il clock interno dell'ADC pari a circa 4.8MHZ (200ns)
	  // Configure ADC10 - Pulse sample mode; ADC10SC trigger
	  ADC10CTL0 &= ~ADC10ON;                    // ADC OFF
	  //ADC10MCTL0 = ADC10SREF_1 | ADC10INCH_10;  // AVcc/2

	  // Configure internal reference
	  while(REFCTL0 & REFGENBUSY);              // If ref generator busy, WAIT
	  REFCTL0 &= ~REFON;                       // Select internal ref = 2.5V
												// Internal Reference ON
}

/*void adcOn(){
	  // ADCCLK è il clock interno dell'ADC pari a circa 4.8MHZ (200ns)
	  // Configure ADC10 - Pulse sample mode; ADC10SC trigger
	  ADC10CTL0 |= ADC10ON;         // 16 ADC10CLKs; ADC ON

	  // Configure internal reference
	  while(REFCTL0 & REFGENBUSY);              // If ref generator busy, WAIT
	  REFCTL0 |= REFON;                         // Internal Reference ON
	  clock_delay(600);                         // Delay (~75us) for Ref to settle
}*/
