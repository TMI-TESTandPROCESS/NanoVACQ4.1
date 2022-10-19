/*
 * COM_SPI.h
 *
 * Created: 14/05/2017 21:47:15
 *  Author: jbs
 */ 


#ifndef DRIVER_SPI_H_
#define DRIVER_SPI_H_

#define Mode_0		0
#define Mode_1		1
#define Mode_2		2
#define Mode_3		3



#define None		0
#define Memory_1	1
#define Memory_2	2
#define Memory_3	3
#define Memory_4	4
#define ADC_1		5
#define ADC_2		6
#define Clear		255


void SPI_MASTER_INIT (unsigned char Mode, unsigned char Speed);
void SPI_SEND (unsigned char *DATA, unsigned char DATA_SIZE);
void SPI_RECEIVE (unsigned char *DATA, unsigned char DATA_SIZE);
unsigned char SPI_TRANCEIVER_CHAR (unsigned char data);
void ChipSelect(unsigned char Number);

#endif 