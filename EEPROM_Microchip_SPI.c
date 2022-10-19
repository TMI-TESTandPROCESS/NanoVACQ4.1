/************************************************************************/
/*************************** DESCRIPTION ********************************/
/************************************************************************/
/*																		*/
/*	Name: EEPROM_Microchip_SPI.c										*/
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


#include "EEPROM_Microchip_SPI.h"

#include "Hardware.h"

#include "Fonctions_Logger.h"
#include "DRIVER_SPI.h"
#include "ALIM.h"

#include <util/delay.h>


//TODO : S�curiser les �critures en fin de m�moire.



/************************************************************************/
/*                                INIT                                  */
/************************************************************************/
void EEPROM_25LC1024_INIT()
{
	SPI_PWR_ON;							// Power SPI ON
	_delay_us(10);
		
	ChipSelect(None);					// Tous les CS � 1
	SPI_MASTER_INIT(0,16);				// Configure le SPI

	ChipSelect(None);					// Tous les CS � 1
}


/************************************************************************/
/*                              STATUS                                  */
/************************************************************************/
unsigned char EEPROM_25LC1024_Get_Status()
{
	unsigned char SPI_BUFFER;
	ChipSelect(1);					// Selectionne la m�moire 1

	_delay_ms(1);	
							
	SPI_TRANCEIVER_CHAR(MICROCHIP_SPI_RDSR);// Read Status Register

	SPI_BUFFER = SPI_TRANCEIVER_CHAR(0x00);
	_delay_ms(1);
	
	ChipSelect(0);						// Tous les CS � 1

	return SPI_BUFFER;
}


/************************************************************************/
/*                              ECRITURE                                */
/************************************************************************/

void EEPROM_SECURE_WRITE(const unsigned long EEPROM_EXT_ADDR, unsigned char *DATA, const unsigned char BIT_COUNT)
{
	unsigned char EEPROM_ADDR[3];
	unsigned char EEPROM_BUFFER[50];
		
	unsigned char BIT_COUNT_Temp = 0;
	unsigned char Byte_Number_To_Send = BIT_COUNT;
	TempLong = EEPROM_EXT_ADDR;
	
	
		if(((TempLong % MICROCHIP_25LC1024_PAGE_SIZE) + BIT_COUNT)  > MICROCHIP_25LC1024_PAGE_SIZE)			// Est-ce que les octets � �crire sont � cheval sur 2 pages ?
		{
			
			/************* Ecriture en fin de page *************/
			
			BIT_COUNT_Temp = MICROCHIP_25LC1024_PAGE_SIZE - (TempLong % MICROCHIP_25LC1024_PAGE_SIZE);		// Calcul de la place qui reste sur la page en cours ?
		
			EEPROM_ADDR[0] = TempLong	>> 8;			// Poids Fort de l'adresse
			EEPROM_ADDR[1] = TempLong;					// Poids faible de l'adresse
					
			for (int i = 0; i < BIT_COUNT_Temp; i++)
			{
				EEPROM_BUFFER[i] = DATA[i];				// Bufferisation des donn�es � transmettre
			}
			
			ChipSelect(Memory_1);						// Selectionne la m�moire 1
			_delay_us(10);									
			SPI_TRANCEIVER_CHAR(MICROCHIP_SPI_WREN);
			_delay_us(10);
			ChipSelect(None);							// Tous les CS � 1
			_delay_us(10);
			
			ChipSelect(Memory_1);						// Selectionne la m�moire 1
			SPI_TRANCEIVER_CHAR(MICROCHIP_SPI_WRITE);	// Instruction d'�criture
			SPI_SEND(EEPROM_ADDR,EEPROM_ADDR_LENGTH);	// Adresse M�moire
			SPI_SEND(EEPROM_BUFFER,BIT_COUNT_Temp);		// Ecriture en m�moire
													
			ChipSelect(None);							// Tous les CS � 1
			
			do {											// Attente fin �criture
				_delay_ms(1);								// Update toutes les 1ms
			} while ((EEPROM_25LC1024_Get_Status() & 0b00000011) != 0);	// du registre Status
			
			Byte_Number_To_Send		-= BIT_COUNT_Temp;		// Mise � jour du nombre d'octets a �crire sur la page suivante
			TempLong				+= BIT_COUNT_Temp;		// Mise � jour de la prochaine adresse d'�criture
		}
			
		/************* Ecriture des donn�es ou de la fin des donn�es *************/	
			
		EEPROM_ADDR[0] = TempLong	>> 8;			// Poids milieu de l'adresse
		EEPROM_ADDR[1] = TempLong;					// Poids faible de l'adresse

		for (int i = 0; i < Byte_Number_To_Send; i++)					
		{													// Bufferisation des donn�es � transmettre
			EEPROM_BUFFER[i] = DATA[i+BIT_COUNT_Temp];		// Si une partie des donn�es � d�j� �tait envoy�,
		}													// on d�cale le pointeur sur Data
	
		ChipSelect(Memory_1);							// Selectionne la m�moire 1	
		_delay_ms(1);							
		SPI_TRANCEIVER_CHAR(MICROCHIP_SPI_WREN);
		_delay_ms(1);
		ChipSelect(None);								// Tous les CS � 1
		_delay_ms(1);
		
		ChipSelect(Memory_1);							// Selectionne la m�moire 1	
		_delay_ms(1);
		SPI_TRANCEIVER_CHAR(MICROCHIP_SPI_WRITE);		// Instruction
		SPI_SEND(EEPROM_ADDR,EEPROM_ADDR_LENGTH);		// Adresse M�moire
		SPI_SEND(EEPROM_BUFFER,Byte_Number_To_Send);	// Ecriture en m�moire
		//_delay_us(10);
		ChipSelect(None);								// Tous les CS � 1
					
		do {												// Attente fin d'�criture
			_delay_ms(1);
		} while ((EEPROM_25LC1024_Get_Status() & 0b00000011) != 0);
		
		//_delay_ms(10);
		ChipSelect(None);	
		_delay_ms(10);							// Tous les CS � 1
		ChipSelect(Clear);					// Tous les CS � 0

}


/************************************************************************/
/*                              LECTURE                                 */
/************************************************************************/ 

void EEPROM_25LC1024_READ(const unsigned long EEPROM_EXT_ADDR, unsigned char *DATA, const unsigned char BIT_COUNT)
{
	ChipSelect(Memory_1);									// Selectionne la m�moire 1	
	
	unsigned char EEPROM_ADDR[3];

	EEPROM_ADDR[0] = EEPROM_EXT_ADDR	>> 8;		// Poids milieu de l'adresse
	EEPROM_ADDR[1] = EEPROM_EXT_ADDR;				// Poids faible de l'adresse

	
	
	_delay_us(10);
	SPI_TRANCEIVER_CHAR(MICROCHIP_SPI_READ);			// Instruction de Lecture
	SPI_SEND(EEPROM_ADDR,EEPROM_ADDR_LENGTH);			// Adresse M�moire
	SPI_RECEIVE(DATA, BIT_COUNT);						// Lecture des donn�es
	_delay_us(10);
	ChipSelect(Clear);							// Tous les CS � 0
}




/*if(EEPROM_ADDR_LENGTH > 2){
	EEPROM_ADDR[0] = TempLong	>> 16;			// Poids fort de l'adresse
	EEPROM_ADDR[1] = TempLong	>> 8;			// Poids milieu de l'adresse
	EEPROM_ADDR[2] = TempLong;					// Poids faible de l'adresse
}*/