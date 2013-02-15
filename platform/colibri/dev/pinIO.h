/*
Copyright 2013, CBL Electronics srl + Siralab srl. All rights reserved.

These sources were developed from CBL Electronics srl + Siralab srl,
Mote automation system development

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

- Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.

- Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.

- Neither the name of CBL Electronics srl + Siralab srl nor the names of its
contributors may be used to endorse or promote products derived from
this software without specific prior written permission.

This software is provided by FUB and the contributors on an "as is"
basis, without any representations or warranties of any kind, express
or implied including, but not limited to, representations or
warranties of non-infringement, merchantability or fitness for a
particular purpose. In no event shall FUB or contributors be liable
for any direct, indirect, incidental, special, exemplary, or
consequential damages (including, but not limited to, procurement of
substitute goods or services; loss of use, data, or profits; or
business interruption) however caused and on any theory of liability,
whether in contract, strict liability, or tort (including negligence
or otherwise) arising in any way out of the use of this software, even
if advised of the possibility of such damage.

This implementation was developed by the Andrea Cannavicci and Stefano
Pagnottelli at the CBL Electronics srl + Siralab srl.

For documentation and questions please use the web site
http://www.cblelectronics.com.
Todi, 2013
*/

/**
 * @file		pinIO.h
 * @addtogroup  peripherial
 * @brief		MSP430 pin set for colibri platform generation
 *
 * @author		Andrea Cannavicci <andrea.cannavicci@cblelectronics.com>
 */
#ifndef PINIO_H
#define PINIO_H


// ********************** PIN 0 ********************
#define PIN0_HAS_ADC 1
#define PIN0_HAS_PWM 0
#define PIN0_HAS_DIO 1

#define PIN0_ADC_PORT(type)   P6##type
#define PIN0_ADC_PIN          BIT0
#define PIN0_PWM_PORT(type)
#define PIN0_PWM_PIN
#define PIN0_DIO_PORT(type)   P6##type
#define PIN0_DIO_PIN          BIT0

// ********************** PIN 1 ********************
#define PIN1_HAS_ADC 0
#define PIN1_HAS_PWM 0
#define PIN1_HAS_DIO 1

#define PIN1_ADC_PORT(type)
#define PIN1_ADC_PIN
#define PIN1_PWM_PORT(type)
#define PIN1_PWM_PIN
#define PIN1_DIO_PORT(type)   P6##type
#define PIN1_DIO_PIN          BIT1


// ********************** PIN 2 ********************
#define PIN2_HAS_ADC 0
#define PIN2_HAS_PWM 0
#define PIN2_HAS_DIO 1

#define PIN2_ADC_PORT(type)
#define PIN2_ADC_PIN
#define PIN2_PWM_PORT(type)
#define PIN2_PWM_PIN
#define PIN2_DIO_PORT(type)   P6##type
#define PIN2_DIO_PIN          BIT0


// ********************** PIN 3 ********************
#define PIN3_HAS_ADC 1
#define PIN3_HAS_PWM 1
#define PIN3_HAS_DIO 1

#define PIN3_ADC_PORT(type)   P6##type
#define PIN3_ADC_PIN          BIT1
#define PIN3_PWM_PORT(type)   P1##type
#define PIN3_PWM_PIN          BIT2
#define PIN3_DIO_PORT(type)   P6##type
#define PIN3_DIO_PIN          BIT1


// ********************** PIN 4 ********************
#define PIN4_HAS_ADC 1
#define PIN4_HAS_PWM 1
#define PIN4_HAS_DIO 1

#define PIN4_ADC_PORT(type)   P6##type
#define PIN4_ADC_PIN          BIT2
#define PIN4_PWM_PORT(type)   P1##type
#define PIN4_PWM_PIN          BIT3
#define PIN4_DIO_PORT(type)   P6##type
#define PIN4_DIO_PIN          BIT3

// ********************** PIN 5 ********************
#define PIN5_HAS_ADC 1
#define PIN5_HAS_PWM 0
#define PIN5_HAS_DIO 1

#define PIN5_ADC_PORT(type)   P6##type
#define PIN5_ADC_PIN          BIT3
#define PIN5_PWM_PORT(type)
#define PIN5_PWM_PIN
#define PIN5_DIO_PORT(type)   P6##type
#define PIN5_DIO_PIN          BIT3

// ********************** PIN 6 ********************
#define PIN6_HAS_ADC 1
#define PIN6_HAS_PWM 0
#define PIN6_HAS_DIO 1

#define PIN6_ADC_PORT(type)   P6##type
#define PIN6_ADC_PIN          BIT4
#define PIN6_PWM_PORT(type)
#define PIN6_PWM_PIN
#define PIN6_DIO_PORT(type)   P6##type
#define PIN6_DIO_PIN          BIT4




#define COLIBRI_IO_PINS 7

void pinIO_init(void);
void pinIO_set(uint8_t pinConf[]);

#endif // !PWM_H
