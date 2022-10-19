

#include "Hardware.h"

#include <avr/io.h>
#include <avr/interrupt.h>

#include "Driver_QLEVER_T3.h"
#include "Fonctions_Logger.h"




/************************************************************************/
/*								  UART							        */
/************************************************************************/
ISR(USART_RX_vect)
{
	cli();
	tempo = 0;	
	Flag_UART_in_use = 1;
	Time_to_ADC = 0;
	USART_RECEIVE_T3(UDR0);		// Lit le Buffer de reception et Lance la machine d'état d'analyse de la trame
	
	sei();						// Enable Interrupts.
}


/************************************************************************/
/*							Internal ADC 						        */
/************************************************************************/
ISR(ADC_vect)
{
	cli();
	ADC_ATMEGA168_ACQ_DONE = 1;
	sei();
}

/************************************************************************/
/*				INT 0 : End of External Measure of ADC1 		        */
/************************************************************************/
ISR(INT0_vect)					// Interruption sur la fin de mesure de l'ADC externe
{
	cli();
	EIMSK &=	~(1 << INT0);									// Selection de l'intéruption 0
	
	ADC_1_ACQ_DONE = 1;
	sei();
}

/************************************************************************/
/*				INT 1 : End of External Measure of ADC2 		        */
/************************************************************************/
ISR(INT1_vect)					// Interruption sur la fin de mesure de l'ADC externe 2
{
	cli();
	EIMSK &=	~(1 << INT1);	
	
	ADC_2_ACQ_DONE = 1;
	sei();
}

/************************************************************************/
/*		PCINT2 : Wake up the Micro when Communication on Rx Pin 	    */
/************************************************************************/
ISR(PCINT2_vect)					// Pour reveiller le microcontrolleur lors de Communication
{
	cli();
	PCIFR	|= (1 << PCIF2);		// Reset le flag sur le changement de niveau de la pin RX
	PCICR	&= ~(1 << PCIE2);		// Désactive l'interruption sur le changement de niveau de la pin RX
	
	tempo				= 0;	
	Flag_UART_in_use	= 1;
	
	UCSR0B	|= (1<<RXEN0);			// Enable receiver
	UCSR0B	|= (1<<RXCIE0);			// Enable RX Complete Interrupt
	
	sei();
}

/************************************************************************/
/*				TIMER 2 : Used with the 32.768kHz Quartz		 	    */
/************************************************************************/
/*ISR(TIMER2_OVF_vect)				// OverFlow
{
	cli();
	Clock_Update();					//  Mise à jour de la RTC interne
	RTC_ACQ_secondes++;
	Logger_Flag_ACQ = 1;			// Lève un Flag pour
	RTC_Adjust(AMB_Temp_ADC);
	sei();
}*/

ISR(TIMER2_COMPA_vect)				// COMP A
{
	cli();
		
	if(ACQ_Rate == F_256Hz)		{ T_milliSec += 1;}
	else if (ACQ_Rate != F_1Hz)	{ T_milliSec += (OCR2A +1);}
	
	if ((T_milliSec == 0) || (ACQ_Rate == F_1Hz))
	{
		Clock_Update();					// Mise à jour de la RTC interne
		Stats_Log(AMB_Temp_ADC);		// Log des Stats d'utilisation
		Check_Pile();
		if(ACQ_Rate == F_1Hz)
		{	
			//RTC_Adjust(AMB_Temp_ADC);
		}
	}
	
	RTC_ACQ_secondes++;
	Logger_Flag_ACQ = 1;			// Lève un Flag pour
	sei();
}

ISR(TIMER2_COMPB_vect)				// // COMP B : Tempo pour préchauffage de l'ADC
{
	cli();
	Delay_Done = 1;				// Lève un Flag pour
	//TIFR2	|= (1<<OCF2B);							// TIMER 2	Clear Interrupt flag
	//while (ASSR & ((1 << TCN2UB)|(1 << TCR2BUB)));	// wait for osc not busy
	TIMSK2	&= ~(1<<OCIE2B);
	sei();
}


/*{
	cli();
	TCNT2 = 0;
	
	Clock_Update();					//  Mise à jour de la RTC interne
	RTC_ACQ_secondes++;
	Logger_Flag_ACQ = 1;			// Lève un Flag pour
	sei();
	//RTC_Adjust(AMB_Temp_ADC);
	
	
	cli();
	unsigned char t_OCR2A = OCR2A;
	t_OCR2A	 += CURRENT_CADENCE.Period;
	
	if (t_OCR2A == 0xFF) 	{OCR2A = CURRENT_CADENCE.Period - 1;}
	else					{OCR2A = t_OCR2A;}

	RTC_ACQ_secondes++;
	Logger_Flag_ACQ = 1;			// Lève un Flag pour 
	sei();
	*/
	
	
	
	/*
	cli();
	unsigned char t_OCR2A = OCR2A;
	if (t_OCR2A == 0x7F) 					// Every Second
	{
		Clock_Update();					//  Mise à jour de la RTC interne
		TCNT2 = 0;
	}
	
	
	t_OCR2A	 += CURRENT_CADENCE.Period;
	
	if (t_OCR2A == 0x7F) 					// Every Second
	{ 
		t_OCR2A = 0x7F;
		//RTC_Adjust(AMB_Temp_ADC);		
	}
	else	{OCR2A = t_OCR2A;}

	RTC_ACQ_secondes++;
	Logger_Flag_ACQ = 1;			// Lève un Flag pour 
	sei();
	*/
	
	
	
	
	
	/*
	unsigned char t_OCR2A = OCR2A;
	if (t_OCR2A == 0x7F) 	
	{
		Clock_Update();					//  Mise à jour de la RTC interne
		OCR2A = CURRENT_CADENCE.Period - 1;
		//RTC_Adjust(AMB_Temp_ADC);
	}
	else	{OCR2A = t_OCR2A + CURRENT_CADENCE.Period;}
	
	RTC_ACQ_secondes++;
	Logger_Flag_ACQ = 1;			// Lève un Flag pour
	sei();
	*/
	
//}



