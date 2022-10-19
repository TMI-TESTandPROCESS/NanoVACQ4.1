/************************************************************************/
/*************************** DESCRIPTION ********************************/
/************************************************************************/
/*																		*/
/*	Name: EEPROM_Microchip_SPI.h										*/
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

#ifndef EEPROM_Microchip_SPI_H_
	#define EEPROM_Microchip_SPI_H_

	void EEPROM_ST_M95_INIT();
	void EEPROM_ST_M95_WRITE_SECURED(const unsigned long EEPROM_EXT_ADDR, unsigned char *DATA, const unsigned char BIT_COUNT);
	void EEPROM_ST_M95_READ(const unsigned long EEPROM_EXT_ADDR, unsigned char *DATA, const unsigned char BIT_COUNT);
	unsigned char EEPROM_ST_M95_Get_Status();
	unsigned char EEPROM_ST_M95_TEST();
	
	void EEPROM_ST_M95_WRITE(const unsigned long EEPROM_EXT_ADDR, unsigned char *DATA, const unsigned char BIT_COUNT);

	#define EEPROM_ADDR_LENGTH				2
	#define EEPROM_ST_M95512_PAGE_SIZE		128			
	//#define EEPROM_ST_M95512_SIZE			0x27FFF		
		
	/* ST M95512 Register Map */
	#define EEPROM_CMD_READ			0b00000011
	#define EEPROM_CMD_WRITE		0b00000010
	#define EEPROM_CMD_WREN			0b00000110
	#define EEPROM_WRDI				0b00000100
	#define EEPROM_CMD_RDSR			0b00000101
	#define EEPROM_WRSR				0b00000001
	#define EEPROM_PE				0b01000010
	#define EEPROM_SE				0b11011000
	#define EEPROM_CE				0b11000111
	#define EEPROM_RDID				0b10101011
	#define EEPROM_DPD				0b10111001
	
	#define EEPROM_ST_M95_ID_Page	0b10000011

#endif 