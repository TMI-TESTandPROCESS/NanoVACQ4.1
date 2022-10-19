/************************************************************************/
/*************************** DESCRIPTION ********************************/
/************************************************************************/
/*																		*/
/*	Name: EEPROM_Microchip_I2C.c										*/
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
/* 1.0 - 10/05/2017 - Creation by J.BARBARAS 							*/
/*																		*/
/*																		*/
/************************************************************************/



#include "EEPROM_ST_M95.h"
#include "DRIVER_SPI.h"
#include "Hardware.h"

#include "Fonctions_Logger.h"

/************************************************************************/
/*                        Lecture mémoire                               */
/************************************************************************/
	
// Cette fonction permet d'effectuer une écriture mémoire
// Paramètres : Début de l'adresse, tableau pour ranger les données et nombre de valeurs à écrire
// Cette fonction permet également de gérer l'action à prendre lors du mémoire pleine.

void MEMORY_EXT_WRITE(const unsigned long MEMORY_ADDRESS, unsigned char *DATA, const unsigned char BIT_COUNT)
{
	EEPROM_ST_M95_INIT();														// Initialisation de la mémoire
	if((MEMORY_ADDRESS + BIT_COUNT - 1)  < (unsigned long)(TOTAL_EEPROM_SIZE + 0x008000))	// Est-ce que les octets à écrire dépasse la taille de la mémoire ?
	{			
		EEPROM_ST_M95_WRITE_SECURED(MEMORY_ADDRESS, DATA, BIT_COUNT);		// si non, ecrire normalement
	}
	
	else																	// Si on est au max de la capacitée mémoire
	{
		if (DevBoard.Mem_Tournante == 0x01)									// Si on est en mémoire tournante
		{
			CURRENT_RECORD.ADDR_DATA = ADDR_DEBUT_DATA;						// TODO: Probleme si plusieurs JOBs. Va supprimer le Job 1. A priori normal
			//CURRENT_RECORD.ADDR_DATA = CURRENT_RECORD.DEBUT_DATA;			// TODO: A vérifier et a adapter à QLEVER

			EEPROM_ST_M95_WRITE_SECURED(CURRENT_RECORD.ADDR_DATA, DATA, BIT_COUNT);	// On ecrit les donnée au début de l'espace mémoire dispo
			CURRENT_RECORD.ADDR_DATA	+=	i_ACQ_Bytes_buffered;
		}
		else
		{
			MEM_FULL = 1;
			CURRENT_CADENCE.Nombre_ACQ_DONE -= (i_ACQ_Bytes_buffered / (ADC1.Nb_Voies_Mem*2));
			Logger_Stop();													// Si pas mémoire tournante, on arrete le Logger
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
