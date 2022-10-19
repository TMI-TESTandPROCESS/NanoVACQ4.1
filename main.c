/************************************************************************/
/*																		*/
/* File Name : main.c													*/
/* Revision : 1.0														*/
/*																		*/
/* Company : TMI - ORION                                                */
/*                                                                      */
/* Summary : This file contains the "main" function                     */
/*																		*/
/* Sauf indication contraire, MSB en premier, LSB en dernier            */
/*																		*/
/************************************************************************/
/*                          History Revisions                           */
/************************************************************************/
/*																		*/
/* 1.0 - Creation by J.BARBARAS 29/01/2017								*/
/* A0  - Code for the PicoVACQ 2CH Board								*/
/* A1  - Added FLASH support (not finished)								*/
/*																		*/
/************************************************************************/


#include "Hardware.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>		// For memset function

#include "ALIM.h"
#include "RTC.h"
#include "Driver_QLEVER_T3.h"
#include "Fonctions_Logger.h"

#include "Infos.h"

#include "DRIVER_UART.h"
#include "DRIVER_SPI.h"


#ifdef OPTION_BIGMEMORY
unsigned char const Firmware[]  PROGMEM = {"PV2CT3A1c764"};
#else
unsigned char const Firmware[]  PROGMEM = {"PV2CT3A1m604"}; 
#endif

unsigned char Tab_Mois[]	= {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};


int main(void)
{	
	/************************************************************************/
	/*                          INITIALISE LOGGER                           */
	/************************************************************************/

	BOARD_INIT();			// Initialisation Générale de la carte
	sei();					// reenable interrupts
	
	while (1)
	{	
		if (UART_Ready_to_Send == 1)
		{
			USART_SEND_T3(UART_CMD);
			UART_EXIT();
			sei();
		}
			
			

		if (DevBoard.statut != 0x00)			// En Start
		{
			if (ERROR != 0b00000000)
			{
				Logger_Stop();
			}
			
			if (Logger_Flag_ACQ == 1)
			{
				Logger_Flag_ACQ = 0;
				
				Start_Log_Procedure();					
			}
			
			if (Flag_UART_in_use == 1)			// Si une interruption sur RX
			{
				tempo++;
												
				if (tempo >= 2000)				// Si TimeOut
				{
					UART_EXIT();
				}
			}
			
			else if (ACQ_Rate != F_256Hz)
			{
				tempo = 0;
											// Cannot Disable BOD on ATMega Automotive. 20/30µA residual consumption
				set_sleep_mode(SLEEP_MODE_PWR_SAVE);
				
				PCIFR	|= (1 << PCIF2);	// PIN RX	Reset le flag sur le changement de niveau de la pin RX
				PCICR	|= (1 << PCIE2);	//			Active l'interruption sur le changement de niveau de la pin RX
								
				UCSR0B	|= (1<<RXEN0);		// UART		Active le RX de l'UART
				UCSR0B	|= (1<<RXCIE0);		//			Active l'interruption sur RX
				
				TIFR2	|= (1<<OCF2A);		// TIMER 2	Clear Interrupt flag
				//TIMSK2	|= (1<<OCIE2A);		//			Activation de l'interruption
																				
				ADCSRA	&=	~(1 << ADEN);	// 0 = Disables the ADC
				PRR		|=	1 << PRADC;		// Désactive ADC
				
				if (Write_In_Progress != 1)
				{
					ALIM_MEM_OFF();
					//_delay_us(5);
				}
				
				sei();
				sleep_mode();
				sei();
				

			}
		}
		sei();			
	}
}