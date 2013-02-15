/*
 * Copyright (c) 2012, Andrea Cannavicci + Stefano Pagnottelli
 * CBL Electronics srl + Siralab srl
 * */
#include "contiki.h"
#include "pinIO.h"
#include "infomem.h"


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
					PIN0_PWM_PORT(OUT) &= ~PIN0_PWM_PIN;
					PIN0_PWM_PORT(SEL) &= ~PIN0_PWM_PIN;
		#endif
		#if (PIN0_HAS_ADC)
					PIN0_ADC_PORT(OUT) &= ~PIN0_ADC_PIN;
					PIN0_ADC_PORT(SEL) &= ~PIN0_ADC_PIN;
		#endif
		#if (PIN0_HAS_DIO)
					PIN0_DIO_PORT(OUT) &= ~PIN0_DIO_PIN;
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
			PIN0_PWM_PORT(SEL) &= ~ PIN0_PWM_PIN;
		#endif;
		default:
			break;
		}
		break;
	case 1:
		//pin reset
		#if (PIN1_HAS_PWM)
					PIN1_PWM_PORT(OUT) &= ~ PIN1_PWM_PIN;
					PIN1_PWM_PORT(SEL) &= ~ PIN1_PWM_PIN;
		#endif
		#if (PIN1_HAS_ADC)
					PIN1_ADC_PORT(OUT) &= ~ PIN1_ADC_PIN;
					PIN1_ADC_PORT(SEL) &= ~ PIN1_ADC_PIN;
		#endif
		#if (PIN1_HAS_DIO)
					PIN1_DIO_PORT(OUT) &= ~ PIN1_DIO_PIN;
		#endif
		//set pin to chosed mode
		switch(mode){
		case analogIn:
		#if (PIN1_HAS_ADC)
			PIN1_ADC_PORT(OUT) |= PIN1_ADC_PIN;
		#endif
		case digitalOut:
			PIN1_DIO_PORT(OUT) |= PIN1_DIO_PIN;
		case pwmOut:
		#if (PIN1_HAS_PWM)
			PIN1_PWM_PORT(SEL) &= ~ PIN1_PWM_PIN;
		#endif;
		default:
			break;
		}
		break;
	case 2:
		//pin reset
		#if (PIN1_HAS_PWM)
					PIN1_PWM_PORT(OUT) &= ~ PIN1_PWM_PIN;
					PIN1_PWM_PORT(SEL) &= ~ PIN1_PWM_PIN;
		#endif
		#if (PIN1_HAS_ADC)
					PIN1_ADC_PORT(OUT) &= ~ PIN1_ADC_PIN;
					PIN1_ADC_PORT(SEL) &= ~ PIN1_ADC_PIN;
		#endif
		#if (PIN1_HAS_DIO)
					PIN1_DIO_PORT(OUT) &= ~ PIN1_DIO_PIN;
		#endif
		//set pin to chosed mode
		switch(mode){
		case analogIn:
		#if (PIN1_HAS_ADC)
			PIN1_ADC_PORT(OUT) |= PIN1_ADC_PIN;
		#endif
		case digitalOut:
			PIN1_DIO_PORT(OUT) |= PIN1_DIO_PIN;
		case pwmOut:
		#if (PIN1_HAS_PWM)
			PIN1_PWM_PORT(SEL) &= ~ PIN1_PWM_PIN;
		#endif;
		default:
			break;
		}
		break;
	case 3:
		//pin reset
		#if (PIN3_HAS_PWM)
					PIN3_PWM_PORT(OUT) &= ~ PIN3_PWM_PIN;
					PIN3_PWM_PORT(SEL) &= ~ PIN3_PWM_PIN;
		#endif
		#if (PIN3_HAS_ADC)
					PIN3_ADC_PORT(OUT) &= ~ PIN3_ADC_PIN;
					PIN3_ADC_PORT(SEL) &= ~ PIN3_ADC_PIN;
		#endif
		#if (PIN3_HAS_DIO)
					PIN3_DIO_PORT(OUT) &= ~ PIN3_DIO_PIN;
		#endif
		//set pin to chosed mode
		switch(mode){
		case analogIn:
		#if (PIN3_HAS_ADC)
			PIN3_ADC_PORT(OUT) |= PIN3_ADC_PIN;
		#endif
		case digitalOut:
			PIN3_DIO_PORT(OUT) |= PIN3_DIO_PIN;
		case pwmOut:
		#if (PIN3_HAS_PWM)
			PIN3_PWM_PORT(SEL) &= ~ PIN3_PWM_PIN;
		#endif;
		default:
			break;
		}
		break;
	case 4:
		//pin reset
		#if (PIN4_HAS_PWM)
					PIN4_PWM_PORT(OUT) &= ~ PIN4_PWM_PIN;
					PIN4_PWM_PORT(SEL) &= ~ PIN4_PWM_PIN;
		#endif
		#if (PIN4_HAS_ADC)
					PIN4_ADC_PORT(OUT) &= ~ PIN4_ADC_PIN;
					PIN4_ADC_PORT(SEL) &= ~ PIN4_ADC_PIN;
		#endif
		#if (PIN4_HAS_DIO)
					PIN4_DIO_PORT(OUT) &= ~ PIN4_DIO_PIN;
		#endif
		//set pin to chosed mode
		switch(mode){
		case analogIn:
		#if (PIN4_HAS_ADC)
			PIN4_ADC_PORT(OUT) |= PIN4_ADC_PIN;
		#endif
		case digitalOut:
			PIN4_DIO_PORT(OUT) |= PIN4_DIO_PIN;
		case pwmOut:
		#if (PIN4_HAS_PWM)
			PIN4_PWM_PORT(SEL) &= ~ PIN4_PWM_PIN;
		#endif;
		default:
			break;
		}
		break;
	case 5:
		//pin reset
		#if (PIN5_HAS_PWM)
					PIN5_PWM_PORT(OUT) &= ~ PIN5_PWM_PIN;
					PIN5_PWM_PORT(SEL) &= ~ PIN5_PWM_PIN;
		#endif
		#if (PIN5_HAS_ADC)
					PIN5_ADC_PORT(OUT) &= ~ PIN5_ADC_PIN;
					PIN5_ADC_PORT(SEL) &= ~ PIN5_ADC_PIN;
		#endif
		#if (PIN5_HAS_DIO)
					PIN5_DIO_PORT(OUT) &= ~ PIN5_DIO_PIN;
		#endif
		//set pin to chosed mode
		switch(mode){
		case analogIn:
		#if (PIN5_HAS_ADC)
			PIN5_ADC_PORT(OUT) |= PIN5_ADC_PIN;
		#endif
		case digitalOut:
			PIN5_DIO_PORT(OUT) |= PIN5_DIO_PIN;
		case pwmOut:
		#if (PIN5_HAS_PWM)
			PIN5_PWM_PORT(SEL) &= ~ PIN5_PWM_PIN;
		#endif;
		default:
			break;
		}
		break;
	case 6:
		//pin reset
		#if (PIN6_HAS_PWM)
					PIN6_PWM_PORT(OUT) &= ~ PIN6_PWM_PIN;
					PIN6_PWM_PORT(SEL) &= ~ PIN6_PWM_PIN;
		#endif
		#if (PIN6_HAS_ADC)
					PIN6_ADC_PORT(OUT) &= ~ PIN6_ADC_PIN;
					PIN6_ADC_PORT(SEL) &= ~ PIN6_ADC_PIN;
		#endif
		#if (PIN6_HAS_DIO)
					PIN6_DIO_PORT(OUT) &= ~ PIN6_DIO_PIN;
		#endif
		//set pin to chosed mode
		switch(mode){
		case analogIn:
		#if (PIN6_HAS_ADC)
			PIN6_ADC_PORT(OUT) |= PIN6_ADC_PIN;
		#endif
		case digitalOut:
			PIN6_DIO_PORT(OUT) |= PIN6_DIO_PIN;
		case pwmOut:
		#if (PIN6_HAS_PWM)
			PIN6_PWM_PORT(SEL) &= ~ PIN6_PWM_PIN;
		#endif;
		default:
			break;
		}
		break;
	default:
		break;
	}
}
