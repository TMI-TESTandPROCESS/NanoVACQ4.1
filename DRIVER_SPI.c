/*
 * COM_SPI.c
 *
 * Created: 14/05/2017 21:46:55
 *  Author: jbs
 */ 

#include "DRIVER_SPI.h"
#include "hardware.h"
#include "Fonctions_Logger.h"
#include "ALIM.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/************************************************************************/
/*                          SPI_MASTER_INIT                             */
/************************************************************************/

// Initialise la communication SPI en Master avec la vitesse et le mode de 
// com paramétrables

void SPI_MASTER_INIT(unsigned char Mode, unsigned char Speed)
{
		
	SPCR |=   1<<MSTR;			// Master/Slave Select (1 = Master)
	SPCR &=	~(1<<SPIE);			// SPI0 Interrupt Enable = 0
	SPCR |=   1<<SPE;			// SPI0 Enable. This bit must be set to enable any SPI operations
	SPCR &=	~(1<<DORD);			// Data Order (1 = the LSB of the data word is transmitted first)
	SPSR &= ~(1<<SPI2X);		// Double SPI Speed Bit (1 = active)
	
	_delay_us(1);
	
	
	/***** SPI Control Register *****/
/*	SPCR = (0<<SPIE) | (0<<DORD) | (1<<MSTR);
	_delay_us(1);
	SPSR &= ~(1<<SPI2X);		// Double SPI Speed Bit (1 = active)
	_delay_us(1);
	*/	
	switch (Speed)				// Prescale Clock Rate Select F_CPU/(4,16,64,128) if SPI2X=0
	{
		case 4:					// F_CPU / 4
		{
			SPCR &=	~(1<<SPR0);
			SPCR &=	~(1<<SPR1);
			break;
		}
		
		case 16:				// F_CPU / 16
		{
			SPCR |=	 (1<<SPR0);		
			SPCR &=	~(1<<SPR1);		
			break;
		}
		
		case 64:				// F_CPU / 64	
		{
			SPCR &=	~(1<<SPR0);
			SPCR |=	 (1<<SPR1);
			break;
		}
		
		default:				// F_CPU / 128		(default permet de gagner 12 Octets Flash)
		{
			SPCR |=	 (1<<SPR0);
			SPCR |=	 (1<<SPR1);
			break;
		}
	}
	_delay_us(1);
		
	switch (Mode)				// Mode Select
	{
		case 0:
		{
			SPCR &= ~(1<<CPOL);	// Clock Polarity	| SPI Mode 0 : 0/0 | Mode 1 : 0/1
			SPCR &= ~(1<<CPHA);	// Clock Phase		| SPI Mode 2 : 1/0 | Mode 4 : 1/1	
			break;
		}
		case 1:
		{
			SPCR &= ~(1<<CPOL);	// Clock Polarity	| SPI Mode 0 : 0/0 | Mode 1 : 0/1
			SPCR |= 1<<CPHA;	// Clock Phase		| SPI Mode 2 : 1/0 | Mode 4 : 1/1	
			break;
		}
		default:
		{
			SPCR |= 1<<CPOL;	// Clock Polarity	| SPI Mode 0 : 0/0 | Mode 1 : 0/1
			SPCR |= 1<<CPHA;	// Clock Phase		| SPI Mode 2 : 1/0 | Mode 4 : 1/1
			break;
		}
	}	
	_delay_us(1);
	
	SPI_ENABLE;
	
	_delay_us(1);
}
/************************************************************************/
/*                       SPI SEND & RECEIVE                             */
/************************************************************************/

// Fonction pour envoyer un tableau d'octets sur le bus SPI

void SPI_SEND (unsigned char *DATA, unsigned char DATA_SIZE)
{
	for (int i = 0; i < DATA_SIZE; i++)
	{
		SPI_TRANCEIVER_CHAR(DATA[i]);
	}
}

// Fonction pour lire "DATA_SIZE" Octets sur le bus SPI

void SPI_RECEIVE (unsigned char *DATA, unsigned char DATA_SIZE)
{
	for (int i = 0; i < DATA_SIZE; i++)
	{
		DATA[i] = SPI_TRANCEIVER_CHAR(0x00);
	}
}

/************************************************************************/
/*                          SPI_TRANCEIVER                              */
/************************************************************************/
// Function to send and receive One Byte

unsigned char SPI_TRANCEIVER_CHAR (unsigned char data)
{
	SPDR = data;					// Load data into the buffer
	
	//while(!(SPSR & (1<<SPIF) ));	//Wait until transmission complete
		
		
	unsigned int Time_to_SPI = 0;
		
	do {										// Attente fin écriture
								// Update toutes les 200µs
		Time_to_SPI++;						// Pour savoir le temps de l'écriture
	} while ((!(SPSR & (1<<SPIF) )) && (Time_to_SPI < 200));	// du registre Status
	
	
	if(Time_to_SPI >= 200)
	{
		ERROR |= 0b00000010;
		Logger_Stop();									// Arrete l'enregistrement
	}	
		
		
	return(SPDR);					// Return received data
}


/************************************************************************/
/*                          CHIP SELECT		                            */
/************************************************************************/


#ifdef OPTION_BIGMEMORY
	void ChipSelect(unsigned char Number)
	{
		switch(Number)
		{
			case Memory_1:					// Select the first Memory
			{
				PORTC	&=  ~(1<<PC4);	// Mémoire	1
				PORTB	|=   (1<<PB2);		
				PORTC	|=   (1<<PC5);	
				PORTD	|=   (1<<PD5);	
				PORTC	|=   (1<<PC1);	
				PORTC	|=   (1<<PC2);
				PORTD	|=   (1<<PD6);				
				break;
			}
			case Memory_2:					// Select the Second Memory - FLASH
			{
				PORTC	|=   (1<<PC4);	
				PORTB	|=   (1<<PB2);
				PORTC	|=   (1<<PC5);
				PORTD	|=   (1<<PD5);
				PORTC	&=  ~(1<<PC1);	// Mémoire	5
				PORTC	|=   (1<<PC2);	
				PORTD	|=   (1<<PD6);			
				break;
			}
			case ADC_1:						// Select the Main ADC
			{
				PORTC	|=   (1<<PC4);	
				PORTB	|=   (1<<PB2);
				PORTC	|=   (1<<PC5);
				PORTD	|=   (1<<PD5);
				PORTC	|=   (1<<PC1);	
				PORTC	&=  ~(1<<PC2);	// ADC	
				PORTD	|=   (1<<PD6);			
				break;
			}
		
			case ADC_2:						// Select the ADC 2
			{
				PORTC	|=   (1<<PC4);
				PORTB	|=   (1<<PB2);
				PORTC	|=   (1<<PC5);
				PORTD	|=   (1<<PD5);
				PORTC	|=   (1<<PC1);	
				PORTC	|=   (1<<PC2);	
				PORTD	&=  ~(1<<PD6);	// ADC 2
				break;
			}
		
			case Clear:						// Clear all ChipSelect. ALL CS =0
			{
				PORTC	&=  ~(1<<PC4);	// Mémoire	1
				PORTB	&=  ~(1<<PB2);	// Mémoire	2
				PORTC	&=  ~(1<<PC5);	// Mémoire	3
				PORTD	&=  ~(1<<PD5);	// Mémoire	4
				PORTC	&=  ~(1<<PC1);	// Mémoire	5
				PORTC	&=  ~(1<<PC2);	// ADC
				PORTD	&=  ~(1<<PD6);	// ADC 2
				break;
			}
		
			case None:						// No Chip Selected. ALL CS = 1
			{
				PORTC	|=   (1<<PC4);	// Mémoire	1
				PORTB	|=   (1<<PB2);	// Mémoire	2
				PORTC	|=   (1<<PC5);	// Mémoire	3
				PORTD	|=   (1<<PD5);	// Mémoire	4
				PORTC	|=   (1<<PC1);	// Mémoire  5
				PORTC	|=   (1<<PC2);	// ADC
				PORTD	|=   (1<<PD6);	// ADC 2
				break;
			}
		}
		_delay_us(5);
	}
#else
	void ChipSelect(unsigned char Number)
	{
		switch(Number)
		{
			case Memory_1:					// Select the first Memory
			{
				PORTC	&=  ~(1<<PC4);	// Mémoire	1
				PORTB	|=   (1<<PB2);
				PORTC	|=   (1<<PC5);
				PORTD	|=   (1<<PD5);
				PORTC	|=   (1<<PC2);
				PORTD	|=   (1<<PD6);
				break;
			}
			case Memory_2:					// Select the Second Memory
			{
				PORTC	|=   (1<<PC4);
				PORTB	&=  ~(1<<PB2);	// Mémoire	2
				PORTC	|=   (1<<PC5);
				PORTD	|=   (1<<PD5);
				PORTC	|=   (1<<PC2);
				PORTD	|=   (1<<PD6);
				break;
			}
			case Memory_3:					// Select the Second Memory
			{
				PORTC	|=   (1<<PC4);
				PORTB	|=   (1<<PB2);
				PORTC	&=  ~(1<<PC5);	// Mémoire	3
				PORTD	|=   (1<<PD5);
				PORTC	|=   (1<<PC2);
				PORTD	|=   (1<<PD6);
				break;
			}
			case Memory_4:					// Select the Second Memory
			{
				PORTC	|=   (1<<PC4);
				PORTB	|=   (1<<PB2);
				PORTC	|=   (1<<PC5);
				PORTD	&=  ~(1<<PD5);	// Mémoire	4
				PORTC	|=   (1<<PC2);
				PORTD	|=   (1<<PD6);
				break;
			}
			case ADC_1:						// Select the ADC
			{
				PORTC	|=   (1<<PC4);
				PORTB	|=   (1<<PB2);
				PORTC	|=   (1<<PC5);
				PORTD	|=   (1<<PD5);
				PORTC	&=  ~(1<<PC2);	// ADC
				PORTD	|=   (1<<PD6);
				break;
			}
		
			case ADC_2:						// Select the ADC
			{
				PORTC	|=   (1<<PC4);
				PORTB	|=   (1<<PB2);
				PORTC	|=   (1<<PC5);
				PORTD	|=   (1<<PD5);
				PORTC	|=   (1<<PC2);
				PORTD	&=  ~(1<<PD6);	// ADC 2
				break;
			}
		
			case Clear:						// Clear all ChipSelect. ALL CS =0
			{
				PORTC	&=  ~(1<<PC4);	// Mémoire	1
				PORTB	&=  ~(1<<PB2);	// Mémoire	2
				PORTC	&=  ~(1<<PC5);	// Mémoire	3
				PORTD	&=  ~(1<<PD5);	// Mémoire	4
				PORTC	&=  ~(1<<PC2);	// ADC
				PORTD	&=  ~(1<<PD6);	// ADC 2
				break;
			}
		
			case None:						// No Chip Selected. ALL CS = 1
			{
				PORTC	|=   (1<<PC4);	// Mémoire	1
				PORTB	|=   (1<<PB2);	// Mémoire	2
				PORTC	|=   (1<<PC5);	// Mémoire	3
				PORTD	|=   (1<<PD5);	// Mémoire	4
				PORTC	|=   (1<<PC2);	// ADC
				PORTD	|=   (1<<PD6);	// ADC 2
				break;
			}
		}
		_delay_us(5);
	}
#endif


