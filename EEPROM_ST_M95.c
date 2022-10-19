/************************************************************************/
/*************************** DESCRIPTION ********************************/
/************************************************************************/
/*																		*/
/*	Name: EEPROM_ST_M95.c										*/
/*	Version: 1.0														*/
/*																		*/
/*	Fonctions d'écriture et de lecture sur la mémoire 24LC1025			*/
/*  de Microchip.														*/
/*	La fonction écriture comprends une sécurité dans le cas d'une		*/
/*  écriture de plusieurs octets à chevel sur 2 pages; l'écriture se	*/
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
	
	SPI_BUFFER = SPI_TRANCEIVER_CHAR(0x00);	// Récupère la valeure du registre Status
		
	return SPI_BUFFER;
}


/************************************************************************/
/*                               TEST                                   */
/************************************************************************/
unsigned char EEPROM_ST_M95_TEST()				// TODO : A tester !
{
	unsigned char Reponse;
	
	ChipSelect(Memory_1);						// Selectionne la mémoire 1
	SPI_TRANCEIVER_CHAR(EEPROM_ST_M95_ID_Page);	// Read Status Register
	SPI_TRANCEIVER_CHAR(0x00);
	SPI_TRANCEIVER_CHAR(0x00);					// La réponse doit être 0x10h
	Reponse = (SPI_TRANCEIVER_CHAR(0x00)) >> 4;	// Le premier bit = 1 si on a bien une mémoire de 512 kbit
	
	
	ChipSelect(Memory_2);						// Selectionne la mémoire 1
	SPI_TRANCEIVER_CHAR(EEPROM_ST_M95_ID_Page);	// Read Status Register
	SPI_TRANCEIVER_CHAR(0x00);
	SPI_TRANCEIVER_CHAR(0x00);
	Reponse |= (SPI_TRANCEIVER_CHAR(0x00)) >> 3;// Le second bit = 1 si on a bien une mémoire de 512 kbit	
				
	ChipSelect(None);							// Tous les CS à 1
		
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
	
	if(((Buffer_Long % EEPROM_ST_M95512_PAGE_SIZE) + BIT_COUNT)  > EEPROM_ST_M95512_PAGE_SIZE)			// Est-ce que les octets à écrire sont à cheval sur 2 pages ?
	{
		/************* Ecriture en fin de page *************/
			
		BIT_COUNT_Temp = EEPROM_ST_M95512_PAGE_SIZE - (Buffer_Long % EEPROM_ST_M95512_PAGE_SIZE);		// Calcul de la place qui reste sur la page en cours ?
		
		EEPROM_ST_M95_WRITE(Buffer_Long, DATA, BIT_COUNT_Temp);
				
		Byte_Number_To_Send		-= BIT_COUNT_Temp;	// Mise à jour du nombre d'octets a écrire sur la page suivante
		Buffer_Long				+= BIT_COUNT_Temp;	// Mise à jour de la prochaine adresse d'écriture
	}
	
	/************* Ecriture des données ou de la fin des données *************/	
		
	EEPROM_ST_M95_WRITE(Buffer_Long, (DATA + BIT_COUNT_Temp), Byte_Number_To_Send);
}



void EEPROM_ST_M95_WRITE(const unsigned long EEPROM_EXT_ADDR, unsigned char *DATA, const unsigned char BIT_COUNT)
{	
	unsigned char EEPROM_ADDR[3];					// Variable d'adresse temporaire
	unsigned char EEPROM_BUFFER[64];				// Buffer de reception
	
	unsigned char Memory_chip;			// Par default, c'est la mémoire 1 qui est selectionnée.
			
	unsigned long Buffer_Long;						// Vairaible temporaire de type Long
		
	Buffer_Long = EEPROM_EXT_ADDR;
	
	EEPROM_ADDR[0] = Buffer_Long	>> 8;			// Poids Fort de l'adresse
	EEPROM_ADDR[1] = Buffer_Long;					// Poids faible de l'adresse
		
	Memory_chip = Memory_Select(Buffer_Long);
		
	for (int i = 0; i < BIT_COUNT; i++)
	{
		EEPROM_BUFFER[i] = DATA[i];				// Bufferisation des données à transmettre
	}
	
	ChipSelect(Memory_chip);					// Selectionne la mémoire
	_delay_us(10);
	SPI_TRANCEIVER_CHAR(EEPROM_CMD_WREN);		// Commande d'écriture
	ChipSelect(None);							// Tous les CS à 1
		
	ChipSelect(Memory_chip);					// Selectionne la mémoire
	SPI_TRANCEIVER_CHAR(EEPROM_CMD_WRITE);		// Instruction d'écriture
	#ifdef OPTION_BIGMEMORY
			if (Buffer_Long > 0x00FFFF)	{ SPI_TRANCEIVER_CHAR((EEPROM_EXT_ADDR >> 16)); }
	#endif
	SPI_SEND(EEPROM_ADDR,EEPROM_ADDR_LENGTH);	// Adresse Mémoire
	
	SPI_SEND(EEPROM_BUFFER,BIT_COUNT);			// Ecriture en mémoire
	ChipSelect(None);							// Tous les CS à 1
	
	if(ACQ_Rate > F_64Hz) { Write_In_Progress = 1;}						// If 128 or 256Hz do not wait the EEPROM to be finish
	else
	{
		ChipSelect(Memory_chip);					// Selectionne la mémoire
		Time_to_Write = 0;
		do {										// Attente fin écriture
			_delay_us(300);							// Update toutes les 200µs
			Time_to_Write++;						// Pour savoir le temps de l'écriture
		} while (((EEPROM_ST_M95_Get_Status() & 0b00000001) != 0) && (Time_to_Write < 50));	// du registre Status
	
		ChipSelect(None);							// Tous les CS à 1
	
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
	
	ChipSelect(None);								// Tous les CS à 1	
	
	_delay_us(300);
	
	ChipSelect(Memory_chip);*/

	SPI_TRANCEIVER_CHAR(EEPROM_CMD_READ);			// Instruction de Lecture
	#ifdef OPTION_BIGMEMORY
		if (Buffer_Long > 0x00FFFF)	{ 
			EEPROM_ADDR[0] = EEPROM_EXT_ADDR	>> 16;		// Poids milieu de l'adresse
			EEPROM_ADDR[1] = EEPROM_EXT_ADDR	>> 8;;				// Poids faible de l'adresse
			EEPROM_ADDR[2] = EEPROM_EXT_ADDR;
						
			SPI_SEND(EEPROM_ADDR,3);		// Adresse Mémoire	
		}
		else
		{
			SPI_SEND(EEPROM_ADDR,EEPROM_ADDR_LENGTH);		// Adresse Mémoire	
		}
	#else
		SPI_SEND(EEPROM_ADDR,EEPROM_ADDR_LENGTH);		// Adresse Mémoire				
	#endif
	
	SPI_RECEIVE(DATA, BIT_COUNT);					// Lecture des données
	ChipSelect(None);								// Tous les CS à 1
}
