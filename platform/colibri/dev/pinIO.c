/*
 * Copyright (c) 2012, Andrea Cannavicci + Stefano Pagnottelli
 * CBL Electronics srl + Siralab srl
 * */
#include "contiki.h"
#include "pinIO.h"
#include "infomem.h"

#define PIN0_HAS_ADC 1
#define PIN0_HAS_PWM 0
#define PIN0_HAS_DIO 1


#define PIN0_ADC_PORT(type)   P6##type
#define PIN0_ADC_PIN          BIT0
#define PIN0_PWM_PORT(type)
#define PIN0_PWM_PIN
#define PIN0_DIO_PORT(type)   P6##type
#define PIN0_DIO_PIN          BIT0



void pinIO_set(uint8_t pinConf[]){
	int i;
	pinMode mode;
	//pinConf[] contains the configuration of 4 pins because each pin can
	//have up to 4 configuration according with struct pinMode defined in
	//infomem.h
	// pwmOut     = 0,
	// analogIn   = 1,
	// digitalOut = 2,
	// digitalIn  = 3

	for (i=0;i<COLIBRI_IO_PINS;i++)
		mode = (pinMode)pinConf[i/4]>>(2*(i%4)); //
	switch(i)
	{
	case 0:
		//pin reset
		#if (PIN0_HAS_PWM)
					PIN0_PWM_PORT(OUT) ~= PIN0_PWM_PIN;
					PIN0_PWM_PORT(SEL) ~= PIN0_PWM_PIN;
		#endif
		#if (PIN0_HAS_ADC)
					PIN0_ADC_PORT(OUT) ~= PIN0_ADC_PIN;
					PIN0_ADC_PORT(SEL) ~= PIN0_ADC_PIN;
		#endif
		#if (PIN0_HAS_DIO)
					PIN0_DIO_PORT(OUT) ~= PIN0_DIO_PIN;
		#endif
		//set pin to chosed mode
		switch(mode){
		case analogIn:
		#if (PIN0_HAS_ADC)
			PIN0_ADC_PORT(OUT) |= PIN0_ADC_PIN;
		#endif
		case digitalOut:
			PIN0_DIO_PORT(OUT) |= PIN0_DIO_PIN;
		case pwmOut:
		#if (PIN0_HAS_PWM)
			PIN0_PWM_PORT(SEL) ~= PIN0_PWM_PIN;
		#endif;
		default:
			break;
		}
		break;
	case 1:
		break;
	case 2:
		break;
	case 3:
		break;
	case 4:
		break;
	case 5:
		break;
	case 6:
		break;
	default:
		break;
	}
}
