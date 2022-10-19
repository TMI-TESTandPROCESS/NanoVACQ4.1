/************************************************************************/
/*************************** DESCRIPTION ********************************/
/************************************************************************/
/*																		*/
/*	Name: EEPROM_Microchip_I2C.c										*/
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
/* 1.0 - 10/05/2017 - Creation by J.BARBARAS 							*/
/*																		*/
/*																		*/
/************************************************************************/



#include "EEPROM_ST_M95.h"
#include "DRIVER_SPI.h"
#include "Hardware.h"

#include "Fonctions_Logger.h"

/************************************************************************/
/*                        Lecture m�moire                               */
/************************************************************************/
	
// Cette fonction permet d'effectuer une �criture m�moire
// Param�tres : D�but de l'adresse, tableau pour ranger les donn�es et nombre de valeurs � �crire
// Cette fonction permet �galement de g�rer l'action � prendre lors du m�moire pleine.

void MEMORY_EXT_WRITE(const unsigned long MEMORY_ADDRESS, unsigned char *DATA, const unsigned char BIT_COUNT)
{
	EEPROM_ST_M95_INIT();														// Initialisation de la m�moire
	if((MEMORY_ADDRESS + BIT_COUNT - 1)  < (unsigned long)(TOTAL_EEPROM_SIZE + 0x008000))	// Est-ce que les octets � �crire d�passe la taille de la m�moire ?
	{			
		EEPROM_ST_M95_WRITE_SECURED(MEMORY_ADDRESS, DATA, BIT_COUNT);		// si non, ecrire normalement
	}
	
	else																	// Si on est au max de la capacit�e m�moire
	{
		if (DevBoard.Mem_Tournante == 0x01)									// Si on est en m�moire tournante
		{
			CURRENT_RECORD.ADDR_DATA = ADDR_DEBUT_DATA;						// TODO: Probleme si plusieurs JOBs. Va supprimer le Job 1. A priori normal
			//CURRENT_RECORD.ADDR_DATA = CURRENT_RECORD.DEBUT_DATA;			// TODO: A v�rifier et a adapter � QLEVER

			EEPROM_ST_M95_WRITE_SECURED(CURRENT_RECORD.ADDR_DATA, DATA, BIT_COUNT);	// On ecrit les donn�e au d�but de l'espace m�moire dispo
			CURRENT_RECORD.ADDR_DATA	+=	i_ACQ_Bytes_buffered;
		}
		else
		{
			MEM_FULL = 1;
			CURRENT_CADENCE.Nombre_ACQ_DONE -= (i_ACQ_Bytes_buffered / (ADC1.Nb_Voies_Mem*2));
			Logger_Stop();													// Si pas m�moire tournante, on arrete le Logger
		}
	}
}
/************************************************************************/
/*                        Data Transmission                             */
/************************************************************************/

void MEMORY_EXT_READ(const unsigned long MEMORY_ADDRESS, unsigned char *DATA, const unsigned char BIT_COUNT)
{
	EEPROM_ST_M95_INIT();
	EEPROM_ST_M95_READ(MEMORY_ADDRESS, DATA, BIT_COUNT);
}
