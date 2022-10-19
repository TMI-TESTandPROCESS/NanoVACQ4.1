/************************************************************************/
/*************************** DESCRIPTION ********************************/
/************************************************************************/
/*																		*/
/*	Name: DRIVER_UART.c													*/
/*	Version: 1.0														*/
/*																		*/
/*	Les fonctions de ce fichier permettent d'initailiser L'UART de		*/
/*	l'ATMEGA 88 ainsi que d'envoyer et recevoir 1 octet					*/
/*                                                                      */
/* Mode          : Asynchronous USART                                   */
/* Frame Format	 : 8 Bits - 2 Stop Bits									*/
/* Parity Format : No Parity											*/
/* Baud Rate     : 115200 bits/s                                        */
/*                                                                      */
/************************************************************************/
/************************** History Revisions ***************************/
/*																		*/
/* 1.0 - 29/01/2017 - Creation by J.BARBARAS 							*/
/*																		*/
/*																		*/
/************************************************************************/


#include "Hardware.h"			// Configuration du Hardware
#include <avr/io.h>				// AVR device-specific IO definitions
#include <avr/interrupt.h>		// Interruptions
#include <util/delay.h>

#include "DRIVER_UART.h"		// Header 	
#include "Driver_QLEVER_T3.h"	// 


/************************************************************************/
/*                       USART Initialization                           */
/************************************************************************/

void USART_Init(void)
{
	//cli();					// Désactivation des intéruptions pendant l'initialisation
	
	/***** USART Control and Status Register 0 A *****/
	UCSR0A |= 0<<TXC0;		// USART Transmit Complete - This flag bit is set when the entire frame in the transmit shift register has been shifted out and there are no new data currently present in the transmit buffer (UDRn).
	UCSR0A |= 0<<U2X0;		// Double the USART Transmission Speed -  Write this bit to zero when using synchronous operation.
	UCSR0A |= 0<<MPCM0;		// Multi-processor Communication Mode - This bit enables the multi-processor communication mode.

	/***** USART Control and Status Register 0 B *****/
	UCSR0B |= 1<<RXCIE0;	// RX Complete Interrupt Enable - Writing this bit to one enables interrupt on the RXCn flag.
	UCSR0B |= 0<<TXCIE0;	// TX Complete Interrupt Enable - Writing this bit to one enables interrupt on the TXCn flag.
	UCSR0B |= 0<<UDRIE0;	// USART Data Register Empty Interrupt Enable - Writing this bit to one enables interrupt on the UDREn flag.
	UCSR0B |= 1<<RXEN0;		// Receiver Enable - Writing this bit to one enables the USART receiver.
	UCSR0B |= 1<<TXEN0;		// Transmitter Enable - Writing this bit to one enables the USART transmitter. 
	UCSR0B |= 0<<UCSZ02;	// Character Size - The UCSZn2 bits combined with the UCSZn1:0 bit in UCSRnC sets the number of data bits in a frame the 	receiver and transmitter use.
	UCSR0B |= 0<<TXB80;		// Transmit Data Bit 8 - TXB8n is the ninth data bit in the character to be transmitted when operating with serial frames with nine data bits.
		
	/***** USART Control and Status Register 0 C *****/
	UCSR0C |= 0<<UMSEL01;	// USART Mode Select - These bits select the mode of operation of the USART
	UCSR0C |= 0<<UMSEL00;	// USART Mode Select - 0 0 Asynchronous USART - 0 1 Synchronous USART - 1 1 Master SPI (MSPIM)
	UCSR0C |= 0<<UPM01;		// Parity Mode - These bits enable and set type of parity generation and check.
	UCSR0C |= 0<<UPM00;		// Parity Mode - 0 0 Disabled - 1 0 Enabled, even parity -  1 1 Enabled, odd parity
	UCSR0C |= 0<<USBS0;		// Stop Bit Select - This bit selects the number of stop bits to be inserted by the transmitter. - 0 1-bit - 1 2-bit
	UCSR0C |= 1<<UCSZ01;	// Character Size - The UCSZn1:0 bits combined with the UCSZn2 bit in UCSRnB sets the number of data bits in a frame the receiver and transmitter use.
	UCSR0C |= 1<<UCSZ00;	// Character Size - 0 0 0 5-bit - 0 0 1 6-bit - 0 1 0 7-bit - 0 1 1 8-bit - 1 1 1 9-bit
	UCSR0C |= 0<<UCPOL0;	//  UCPOLn Bit Settings - This bit is used for synchronous mode only*/
	
		
	/******* USART Baud Rate Register *******/
	//UBRR0H = (unsigned char)(BAUD_PRESCALE>>8);	// This is a 12-bit register which contains the USART baud rate. The UBRRnH contains the four most significant bits
	//UBRR0L = (unsigned char) BAUD_PRESCALE;		// the UBRRnL contains the eight least significant bits of the USART baud rate
	UBRR0H = 0;
	UBRR0L = 0;
	
	
	/******* USART Variables *******/
	RXState = RX_IDLE;	// Initialisation de la machine d'état de la fonction de reception d'une trame.
}


/************************************************************************/
/*                        Data Transmission                             */
/************************************************************************/ 

/*void USART_Transmit_T3( unsigned char data) //Sending Frames with 5 to 8 Data Bit
{

}
*/

/************************************************************************/
/*               Data Transmission with CheckSum Update                 */
/************************************************************************/

void USART_Transmit( unsigned char data) 
{
	while (!( UCSR0A & (1<<UDRE0))) {};	// Wait for empty transmit buffer

	UDR0 = data;						// Put data into buffer, sends the data
	
	USART_cks += data;					// Update Checksum
	//_delay_us(UART_TEMPO);				// Delay pour ne pas effondrer la ligne si trop de '0' envoyés.
}

