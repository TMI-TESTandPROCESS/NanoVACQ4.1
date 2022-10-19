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

	void EEPROM_25LC1024_INIT();
	void EEPROM_SECURE_WRITE(const unsigned long EEPROM_EXT_ADDR, unsigned char *DATA, const unsigned char BIT_COUNT);
	void EEPROM_25LC1024_READ(const unsigned long EEPROM_EXT_ADDR, unsigned char *DATA, const unsigned char BIT_COUNT);
	unsigned char EEPROM_25LC1024_Get_Status();
	
	//#define MICROCHIP_25LC1024_PAGE_SIZE	256			// 1024
	//#define MICROCHIP_25LC1024_SIZE			0x27FFF		// 1024
	
	#define EEPROM_ADDR_LENGTH				2
	#define MICROCHIP_25LC1024_PAGE_SIZE	128			// 1024
	#define MICROCHIP_25LC1024_SIZE			0xFFFF		// 1024
	
	/* 25LC1024 Register Map */
	#define MICROCHIP_SPI_READ	0b00000011
	#define MICROCHIP_SPI_WRITE	0b00000010
	#define MICROCHIP_SPI_WREN  0b00000110
	#define MICROCHIP_SPI_WRDI  0b00000100
	#define MICROCHIP_SPI_RDSR  0b00000101
	#define MICROCHIP_SPI_WRSR  0b00000001
	#define MICROCHIP_SPI_PE	0b01000010
	#define MICROCHIP_SPI_SE	0b11011000
	#define MICROCHIP_SPI_CE	0b11000111
	#define MICROCHIP_SPI_RDID	0b10101011
	#define MICROCHIP_SPI_DPD	0b10111001
	
	#define ST_ID_Page			0b10000011

#endif 