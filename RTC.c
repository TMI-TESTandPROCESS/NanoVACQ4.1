/*
 * RTC.c
 *
 * Created: 11/05/2017 17:12:22
 *  Author: jbs
 */ 

#include "RTC.h"

#include "Hardware.h"
#include "ALIM.h"

#include <avr/io.h>
#include <avr/interrupt.h>

/*                       Mise � jour de l'heure                         */
/************************************************************************/
						else	{Tab_Mois[1] = 29;}
/*                     Mise � l'heure du LOGGER                         */
/************************************************************************/
	
	RTC_ACQ_secondes = 0;
	
	RTC_T3.annees		=    TIME[7];
	RTC_T3.mois			= (((TIME[6] & 0x10) >> 4) * 10) + (TIME[6] & 0x0F);
	RTC_T3.jours		= (((TIME[5] & 0x30) >> 4) * 10) + (TIME[5] & 0x0F);
	RTC_T3.heures		= (((TIME[4] & 0xF0) >> 4) * 10) + (TIME[4] & 0x0F);
	RTC_T3.minutes		= (((TIME[3] & 0xF0) >> 4) * 10) + (TIME[3] & 0x0F);
	RTC_T3.secondes		= (((TIME[2] & 0xF0) >> 4) * 10) + (TIME[2] & 0x0F);
	RTC_T3.dixiemes		=   (TIME[1] & 0x0F);
	
	if(not_a_leap_year())			// Gestion ann�e bissextile
	
	RTC_SET_TIMER();											// Lance le Timer RTC
{
	/* Returns: - TRUE if year is NOT a leap year
	            - FALSE if year IS a leap year       */
	unsigned char t_annees = RTC_T3.annees + 1900;		// Update to the Real Date
	
	if (!(t_annees % 100)) {				/* Years divisible by 100 not leap years... */
		return (t_annees % 400);			/* ...unless also divisible by 400 */
	} else {
		return (t_annees % 4);				/* Otherwise, every 4th year is a leap year */
	}
}
/*							RTC ADJUST                                  */
/************************************************************************/
void RTC_Adjust(unsigned int Temperature)
{
	unsigned int t_OCR2A;
	t_OCR2A	= 0x7F;								// D�finition de l'overflow du compteur
		
	CumulDeriv += ((float) Temperature * (float) Temperature * indice_1) +  ((float) Temperature * indice_2) + indice_3;
	
	if (CumulDeriv >= RTC_DERIVE_THRESHOLD)
	{
		CumulDeriv = CumulDeriv - RTC_DERIVE_THRESHOLD;
		t_OCR2A	= 0x80;	;
	}
	else if(CumulDeriv <= -RTC_DERIVE_THRESHOLD)
	{
		CumulDeriv = CumulDeriv + RTC_DERIVE_THRESHOLD;
		t_OCR2A	= 0x7E;
	}
		
	OCR2A	= t_OCR2A;
	

	
	
	/*unsigned int t_OCR2A;
	t_OCR2A	= OCR2A;								// D�finition de l'overflow du compteur
	
	
	CumulDeriv += ((float) Temperature * (float) Temperature * indice_1) +  ((float) Temperature * indice_2) + indice_3;

	if (CumulDeriv >= RTC_DERIVE_THRESHOLD)
	{
		CumulDeriv = CumulDeriv - RTC_DERIVE_THRESHOLD;
		TCNT2 = 0x01;
	}
	else if(CumulDeriv <= -RTC_DERIVE_THRESHOLD)
	{
		CumulDeriv = CumulDeriv + RTC_DERIVE_THRESHOLD;
		t_OCR2A	--;
	}
	
	OCR2A	= t_OCR2A;
	*/
}
/*              TIMER0 - 8 bits -                                       */
/************************************************************************/

/************************************************************************/
/*              TIMER1 - 16 bits - Pour calibration d'OSCCAL            */
/************************************************************************/
{
	cli();											// disable global interrupts
	TIMER1_PWR_ON;									// Allume le Timer 1
		
	TCCR1A	 = 0x00;								// Initialisation des registres
	TCCR1B	 = 0x00;
	TCNT1H	 = 0x00;
	TCNT1L	 = 0x00;
	
	TCCR1B	 = (0<<CS12) | (1<<CS11) | (1<<CS10);	// Set Prescalling 011 = /64
		
	TIMSK1	&= ~(1<<OCIE1A);						// Output Compare A Match Interrupt
	TIMSK1	&= ~(1<<OCIE1B);						// Output Compare B Match Interrupt
	TIMSK1	&= ~(1<<TOIE1);							// Timer1 overflow interrupt
	sei();
}

/*       TIMER2 - 8 bits - Asynchrone - Pour trig de l'acquisition      */
/************************************************************************/