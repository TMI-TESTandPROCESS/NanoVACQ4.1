/************************************************************************/
/*************************** DESCRIPTION ********************************/
/************************************************************************/
/*																		*/
/*	Name: EEPROM_ST_M95.c										*/
/*	Version: 1.0														*/
/*																		*/
/*	Fonctions d'�criture et de lecture sur la m�moire 24LC1025			*/
/*  de Microchip.														*/
/*	La fonction �criture comprends une s�curit� dans le cas d'une		*/
/*  �criture de plusieurs octets � chevel sur 2 pages; l'�criture se	*/
/*  fait alors en 2 temps												*/
/*                                                                      */
/*                                                                      */
/************************************************************************/
/************************** History Revisions ***************************/
/*																		*/
/* 1.0 - 05/12/2017 - Creation by J.BARBARAS 							*/
/*																		*/
/*																		*/
/************************************************************************/

#include "Hardware.h"

#include "EEPROM_ST_M95.h"
#include "Fonctions_Logger.h"
#include "DRIVER_SPI.h"
#include "ALIM.h"

#include <util/delay.h>
#include <avr/interrupt.h>



/************************************************************************/
/*                                INIT                                  */
/************************************************************************/
void EEPROM_ST_M95_INIT()
{
	SPI_MASTER_INIT(Mode_0,SPI_CLK_Prescale);	// Configure le SPI
	_delay_us(1);
}


/************************************************************************/
/*                              STATUS                                  */
/************************************************************************/
unsigned char EEPROM_ST_M95_Get_Status()
{
	unsigned char SPI_BUFFER;				// ChipSelect must be selected before
	
	SPI_TRANCEIVER_CHAR(EEPROM_CMD_RDSR);	// Read Status Register
	
	SPI_BUFFER = SPI_TRANCEIVER_CHAR(0x00);	// R�cup�re la valeure du registre Status
		
	return SPI_BUFFER;
}


/************************************************************************/
/*                               TEST                                   */
/************************************************************************/
unsigned char EEPROM_ST_M95_TEST()				// TODO : A tester !
{
	unsigned char Reponse;
	
	ChipSelect(Memory_1);						// Selectionne la m�moire 1
	SPI_TRANCEIVER_CHAR(EEPROM_ST_M95_ID_Page);	// Read Status Register
	SPI_TRANCEIVER_CHAR(0x00);
	SPI_TRANCEIVER_CHAR(0x00);					// La r�ponse doit �tre 0x10h
	Reponse = (SPI_TRANCEIVER_CHAR(0x00)) >> 4;	// Le premier bit = 1 si on a bien une m�moire de 512 kbit
	
	
	ChipSelect(Memory_2);						// Selectionne la m�moire 1
	SPI_TRANCEIVER_CHAR(EEPROM_ST_M95_ID_Page);	// Read Status Register
	SPI_TRANCEIVER_CHAR(0x00);
	SPI_TRANCEIVER_CHAR(0x00);
	Reponse |= (SPI_TRANCEIVER_CHAR(0x00)) >> 3;// Le second bit = 1 si on a bien une m�moire de 512 kbit	
				
	ChipSelect(None);							// Tous les CS � 1
		
	return Reponse;
}


/************************************************************************/
/*                              ECRITURE                                */
/************************************************************************/

void EEPROM_ST_M95_WRITE_SECURED(const unsigned long EEPROM_EXT_ADDR, unsigned char *DATA, const unsigned char BIT_COUNT)
{
	unsigned char BIT_COUNT_Temp = 0;
	unsigned char Byte_Number_To_Send = BIT_COUNT;
		
	unsigned long Buffer_Long;						// Vairaible temporaire de type Long
	
	Buffer_Long = EEPROM_EXT_ADDR;
	
	if(((Buffer_Long % EEPROM_ST_M95512_PAGE_SIZE) + BIT_COUNT)  > EEPROM_ST_M95512_PAGE_SIZE)			// Est-ce que les octets � �crire sont � cheval sur 2 pages ?
	{
		/************* Ecriture en fin de page *************/
			
		BIT_COUNT_Temp = EEPROM_ST_M95512_PAGE_SIZE - (Buffer_Long % EEPROM_ST_M95512_PAGE_SIZE);		// Calcul de la place qui reste sur la page en cours ?
		
		EEPROM_ST_M95_WRITE(Buffer_Long, DATA, BIT_COUNT_Temp);
				
		Byte_Number_To_Send		-= BIT_COUNT_Temp;	// Mise � jour du nombre d'octets a �crire sur la page suivante
		Buffer_Long				+= BIT_COUNT_Temp;	// Mise � jour de la prochaine adresse d'�criture
	}
	
	/************* Ecriture des donn�es ou de la fin des donn�es *************/	
		
	EEPROM_ST_M95_WRITE(Buffer_Long, (DATA + BIT_COUNT_Temp), Byte_Number_To_Send);
}



void EEPROM_ST_M95_WRITE(const unsigned long EEPROM_EXT_ADDR, unsigned char *DATA, const unsigned char BIT_COUNT)
{	
	unsigned char EEPROM_ADDR[3];					// Variable d'adresse temporaire
	unsigned char EEPROM_BUFFER[64];				// Buffer de reception
	
	unsigned char Memory_chip;			// Par default, c'est la m�moire 1 qui est selectionn�e.
			
	unsigned long Buffer_Long;						// Vairaible temporaire de type Long
		
	Buffer_Long = EEPROM_EXT_ADDR;
	
	EEPROM_ADDR[0] = Buffer_Long	>> 8;			// Poids Fort de l'adresse
	EEPROM_ADDR[1] = Buffer_Long;					// Poids faible de l'adresse
		
	Memory_chip = Memory_Select(Buffer_Long);
		
	for (int i = 0; i < BIT_COUNT; i++)
	{
		EEPROM_BUFFER[i] = DATA[i];				// Bufferisation des donn�es � transmettre
	}
	
	ChipSelect(Memory_chip);					// Selectionne la m�moire
	_delay_us(10);
	SPI_TRANCEIVER_CHAR(EEPROM_CMD_WREN);		// Commande d'�criture
	ChipSelect(None);							// Tous les CS � 1
		
	ChipSelect(Memory_chip);					// Selectionne la m�moire
	SPI_TRANCEIVER_CHAR(EEPROM_CMD_WRITE);		// Instruction d'�criture
	#ifdef OPTION_BIGMEMORY
			if (Buffer_Long > 0x00FFFF)	{ SPI_TRANCEIVER_CHAR((EEPROM_EXT_ADDR >> 16)); }
	#endif
	SPI_SEND(EEPROM_ADDR,EEPROM_ADDR_LENGTH);	// Adresse M�moire
	
	SPI_SEND(EEPROM_BUFFER,BIT_COUNT);			// Ecriture en m�moire
	ChipSelect(None);							// Tous les CS � 1
	
	if(ACQ_Rate > F_64Hz) { Write_In_Progress = 1;}						// If 128 or 256Hz do not wait the EEPROM to be finish
	else
	{
		ChipSelect(Memory_chip);					// Selectionne la m�moire
		Time_to_Write = 0;
		do {										// Attente fin �criture
			_delay_us(300);							// Update toutes les 200�s
			Time_to_Write++;						// Pour savoir le temps de l'�criture
		} while (((EEPROM_ST_M95_Get_Status() & 0b00000001) != 0) && (Time_to_Write < 50));	// du registre Status
	
		ChipSelect(None);							// Tous les CS � 1
	
		if(Time_to_Write >= 50)
		{
			ERROR |= 0b00000001;
		}
	}
}




/************************************************************************/
/*                              LECTURE                                 */
/************************************************************************/ 

void EEPROM_ST_M95_READ(const unsigned long EEPROM_EXT_ADDR, unsigned char *DATA, const unsigned char BIT_COUNT)
{
	unsigned char Memory_chip;
	unsigned char EEPROM_ADDR[3];

	unsigned long Buffer_Long;						// Vairaible temporaire de type Long

	Buffer_Long = EEPROM_EXT_ADDR;

	EEPROM_ADDR[0] = EEPROM_EXT_ADDR	>> 8;		// Poids milieu de l'adresse
	EEPROM_ADDR[1] = EEPROM_EXT_ADDR;				// Poids faible de l'adresse

	Memory_chip = Memory_Select(Buffer_Long);
		
	
	ChipSelect(Memory_chip);	
	//_delay_us(10);		
	
/*	SPI_TRANCEIVER_CHAR(0x66);
	SPI_TRANCEIVER_CHAR(0x99);	
	
	ChipSelect(None);								// Tous les CS � 1	
	
	_delay_us(300);
	
	ChipSelect(Memory_chip);*/

	SPI_TRANCEIVER_CHAR(EEPROM_CMD_READ);			// Instruction de Lecture
	#ifdef OPTION_BIGMEMORY
		if (Buffer_Long > 0x00FFFF)	{ 
			EEPROM_ADDR[0] = EEPROM_EXT_ADDR	>> 16;		// Poids milieu de l'adresse
			EEPROM_ADDR[1] = EEPROM_EXT_ADDR	>> 8;;				// Poids faible de l'adresse
			EEPROM_ADDR[2] = EEPROM_EXT_ADDR;
						
			SPI_SEND(EEPROM_ADDR,3);		// Adresse M�moire	
		}
		else
		{
			SPI_SEND(EEPROM_ADDR,EEPROM_ADDR_LENGTH);		// Adresse M�moire	
		}
	#else
		SPI_SEND(EEPROM_ADDR,EEPROM_ADDR_LENGTH);		// Adresse M�moire				
	#endif
	
	SPI_RECEIVE(DATA, BIT_COUNT);					// Lecture des donn�es
	ChipSelect(None);								// Tous les CS � 1
}
