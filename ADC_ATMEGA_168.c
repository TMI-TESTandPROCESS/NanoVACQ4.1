/*
 * ADC_ATMEGA_168.c
 *
 * Created: 19/01/2018 10:17:29
 *  Author: jbs
 */ 

#include "ADC_ATMEGA_168.h"
#include "hardware.h"
#include "ALIM.h"
#include <avr/interrupt.h>
#include <util/delay.h>

/************************************************************************/
/*                       ADC_ATMEGA_168_INIT                            */
/************************************************************************/

void ADC_ATMEGA_168_INIT (unsigned char PRESCALE)	// Initialisation de l'ADC de l'ATMEGA168
{
	ADC_INT_PWR_ON;					// Sortie du mode Veille
		
	/****** ADC Multiplexer Selection Register ******/
	ADMUX	|=	  1 << REFS0;		// Référence selection: 
	ADMUX	&=	~(1 << REFS1);		// 00 = AREF	// 01 = AVCC	// 11 = Internal 1.1V
	ADMUX	&=	~(1 << ADLAR);		// Conversion resutlt: 0 = Right Adjust	// 1 = Left Adjust
	ADMUX	&=	~(1 << MUX3);		// 0000 : ADC0		0100 : ADC4		1000 : ADC8/Temp 
	ADMUX	&=	~(1 << MUX2);		// 0001 : ADC1		0101 : ADC5		1110 : 1.1V
	ADMUX	&=	~(1 << MUX1);		// 0010 : ADC2		0110 : ADC6		1111 : GND
	ADMUX	&=	~(1 << MUX0);		// 0011 : ADC3		0111 : ADC7
	
	/****** ADC Control and Status Register A ******/
	ADCSRA   =	0;
	ADCSRA	|=	  1 << ADEN;		// 1 = Enables the ADC
	ADCSRA	&=	~(1 << ADSC);		// 1 = Start a single conversion
	ADCSRA	&=	~(1 << ADATE);		// 1 = Enables the Auto Triggering Mode
	ADCSRA	&=	~(1 << ADIF);		// ADC Interupt Flag
	ADCSRA	|=	  1 << ADIE;		// 1 = Activate the ADC Convertion Complete Interrupt
	ADCSRA  |= PRESCALE & 0x07;		// 7-BIT ADC Clock PRESCALER 000 = 2 | 001 = 2 | 010 = 4 | 011 = 8 | 100 = 16 | 101 = 32 | 110 = 64 | 111 = 128
	
	/****** ADC Control and Status Register B ******/
	ADCSRB	&=	~(1 << ACME);		// Analog Comparator Multiplexer Enable
	ADCSRB	&=	~(1 << ADTS2);		// 000 = Free Running Mode					001 = Analog comparator
	ADCSRB	&=	~(1 << ADTS1);		// 010 = external Interrupt Request 0		011 = TC0 Compare Match A		100 = TC0 Overflow
	ADCSRB	&=	~(1 << ADTS0);		// 101 = TC1 Compare Match B				110 = TC1 Overflow				111 = TC1 Capture Event
}

/************************************************************************/
/*                      ADC_ATMEGA_168_Get_ACQ                          */
/************************************************************************/

unsigned int ADC_ATMEGA_168_Get_ACQ (unsigned char Channel)
{
	ADC_ATMEGA168_ACQ_DONE = 0;				// Flag de fin d'acquisition

	ADMUX	|= Channel & 0x0F;				// Définition de la voie à mesurer
		
	ADCSRA	|=	1 << ADSC;					// 1 = Start a single conversion
	
	sei();
	
	while (ADC_ATMEGA168_ACQ_DONE == 0){}	// Boucle tant que l'ADC n'a pas fini l'acquisition et n'a pas déclanché l'interruption
	ADC_ATMEGA168_ACQ_DONE = 0;				// Reset du Flag d'acquisition terminée
											// TODO : Mode basse conso pendant l'acquisition : utile ?
	ADC_ATMEGA168_Temp	=	ADCL;			// ADCL doit être lu en premier.
	ADC_ATMEGA168_Temp	+=	ADCH << 8;
	
	return (ADC_ATMEGA168_Temp);
}

/************************************************************************/
/*                     ADC_ATMEGA_168_CONV_VOLTAGE                      */
/************************************************************************/

unsigned int ADC_ATMEGA_168_CONV_VOLTAGE (unsigned int ADC_BAT_VOLTAGE)
{
	//return (( ADC_BAT_VOLTAGE * 0.9422812249 ) - 24.9060672095);	// Conversion en 10aine de millivolts pour 4M7
	return (( ADC_BAT_VOLTAGE * 1.174736877 ) - 7.9509761551);		// Conversion en 10aine de millivolts pour 3M3
	//return ADC_BAT_VOLTAGE;										// Renvoie la valeure brute
}

/************************************************************************/
/*                      ADC_ATMEGA_168_SWITCH_OFF                       */
/************************************************************************/

void ADC_ATMEGA_168_SWITCH_OFF()
{
	ADCSRA	&=	~(1 << ADEN);		// 0 = Disables the ADC
	PRR		|=	  1 << PRADC;			// Désactive ADC
}
