/*
 * ALIM.c
 *
 * Created: 16/05/2017 14:22:41
 *  Author: jbs
 */ 

#include "Hardware.h"
#include "ALIM.h"
#include "Fonctions_Logger.h"
#include "DRIVER_SPI.h"
#include <util/delay.h>		


void ALIM_MEM_ON()			
{
	VDD_NUM_ON;
	
	//ChipSelect(Clear);	// Met les CS à 0
	
	#ifdef OPTION_BIGMEMORY		// If flash memory, need 300µs to start
		VDD_AUX_ON;
		ChipSelect(None);			// Déselectionne tous les chips SPI
		 _delay_us(300);	
	#else
		_delay_us(5);	
		ChipSelect(None);			// Déselectionne tous les chips SPI
	#endif
	
	
	
}


void ALIM_MEM_OFF()			
{
	SPI_DISABLE;		// Désactive le SPI
	_delay_us(10);	
		
	VDD_NUM_OFF;
	#ifdef OPTION_BIGMEMORY
		VDD_ANA_OFF;
		VDD_AUX_OFF;
	#endif
	ChipSelect(Clear);	// Met les CS à 0
		
	SPI_MOSI_0;			// MOSI = 0
	SPI_MISO_0;			// MISO = 0
	SPI_SCK_0;			// SCK  = 0		
}

void ALIM_ADC_ON()
{	
	ALIM_MEM_ON();		// Les 2 sont sur la même alim, même bus...
	
	VDD_ANA_ON;			// Allume l'alimentation Analogique
	_delay_us(5);
}

void ALIM_ADC_OFF()
{
	ALIM_MEM_OFF();		// Les 2 sont sur la même alim, même bus...
	
	VDD_ANA_OFF;		// Eteint l'alimentation Analogique
}

