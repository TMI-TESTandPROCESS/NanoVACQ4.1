/**************************** ADS1120.c *********************************/
/*																		*/
/*                                                                      */
/*																		*/
/*                                                                      */
/*																		*/
/*                                                                      */
/************************************************************************/
/************************** History Revisions ***************************/
/*																		*/
/* 1.0 - 20/11/2017 - Creation by J.BARBARAS 							*/
/*																		*/
/*																		*/
/************************************************************************/

#include "ADS1120.h"

#include "Hardware.h"

#include "DRIVER_SPI.h"
#include "ALIM.h"
#include "Fonctions_Logger.h"

#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>


/************************************************************************/
/*                      Initialisation de l'ADC                         */
/************************************************************************/

void ADC_INIT(void)
{	
	SPI_MASTER_INIT(Mode_1,SPI_CLK_Prescale);	// Initialise le bus SPI en Mode 1
}


/*************************************************************************/
/*                          Get_Internal_Temp	 					     */
/*************************************************************************/
// Return the Internal Temperature of the ADC 

unsigned int Get_Internal_Temp ()	// Acquisition de la température interne à l'ADC
{
	unsigned char tempReg[2];
	unsigned char ADS_1120_Conf_Reg_1;
	
	//AMB_Temp_AGE = 0;
	
	ADS_1120_Conf_Reg_1	=	(6 << 5) + (ADS_1120_NORMAL_MODE << 3)	+ (ADS_1120_SINGLE_SHOT_MODE << 2) + (ADS_1120_INTERNAL_T_ON << 1) + ADS_1120_BURN_OUT_OFF;
	
	ChipSelect(ADC_1);
	
	SPI_TRANCEIVER_CHAR(ADS_1120_WRITE_REG | 4);
	SPI_TRANCEIVER_CHAR(ADS_1120_Conf_Reg_1);				// Configuration of the 1st Register
	_delay_us(10);
	
	SPI_TRANCEIVER_CHAR(ADS_1120_START);					// Start The convertion
	
	ChipSelect(None);
	ADC_1_ACQ_DONE = 0;										// Reset du Flag d'acquisition terminée	ADC_SET_MICRO_TO_SLEEP(DRDY_2_UNUSED);
	
	while (ADC_1_ACQ_DONE == 0){};							// Attente fin Acquisition
	
	ChipSelect(ADC_1);
	
	tempReg[0] = SPI_TRANCEIVER_CHAR(0);					// Reception de la valeur stockée
	tempReg[1] = SPI_TRANCEIVER_CHAR(0);
	
	ChipSelect(None);
		
	unsigned int T_Value = ((tempReg[0] << 6) + (tempReg[1] >>2));
	if ((T_Value & 0x2000) == 0x2000)							// Est-ce que le MSB (bit 14) est à 1
	{
		T_Value -= 1;											//   If so, the temperature is negative
		T_Value ^= 0x3FFF;										// On soustrait 1 est on fait le complément à 2
		
		return (Internal_Temp_Offset -(T_Value * 3.125));		// De manière à avoir une valeur toujours positive, on ajoute un Offset
	}
	
	else
	{
		return ((T_Value * 3.125) + Internal_Temp_Offset);		// Calcul pour les températures positives
	}
}

/*************************************************************************/
/*                       ADC_Get_Channel_Value		 				     */
/*************************************************************************/
// Standard Acquisition

unsigned int ADC_Get_Channel_Value(unsigned int Offset)
{
	unsigned char tempReg[2];
	
	ChipSelect(ADC_1);
	SPI_TRANCEIVER_CHAR(ADS_1120_START);					// Start Measure
	
	ChipSelect(None);
	ADC_1_ACQ_DONE = 0;										// Reset du Flag d'acquisition terminée	ADC_SET_MICRO_TO_SLEEP(DRDY_2_UNUSED);
	
	while (ADC_1_ACQ_DONE == 0){};							// Attente fin Acquisition
	
	ChipSelect(ADC_1);
	
	tempReg[0] = SPI_TRANCEIVER_CHAR(0);					// Reception de la valeur stockée
	tempReg[1] = SPI_TRANCEIVER_CHAR(0);
	
	ChipSelect(None);
	
	unsigned int Value = ((tempReg[0] << 8) | tempReg[1]);	// Mise en forme pour gérer les valeures négatives
	Value	= ADC_Add_Offset(Value, Offset);

	return (Value);											// Mise en forme de la valeure lue
}

/*************************************************************************/
/*                     ADC_Get_Double_ADC_Channel_Value                  */
/*************************************************************************/
// In case of 2 ADC, This function is used to read simultaneously the 2 ADC

unsigned long ADC_Get_Double_ADC_Channel_Value(unsigned int Offset)
{
	unsigned char tempReg[4];
	
	ChipSelect(ADC_1);
	SPI_TRANCEIVER_CHAR(ADS_1120_START);
	
	ChipSelect(ADC_2);
	SPI_TRANCEIVER_CHAR(ADS_1120_START);
	
	ChipSelect(None);
	ADC_1_ACQ_DONE = 0;									// Reset du Flag d'acquisition terminée	ADC_2_ACQ_DONE = 0;									// Reset du Flag d'acquisition terminée	ADC_SET_MICRO_TO_SLEEP(DRDY_2_USED);
	
	while (ADC_1_ACQ_DONE == 0){};						// Attente fin Acquisition
	
	ChipSelect(ADC_1);
	
	tempReg[0] = SPI_TRANCEIVER_CHAR(0);				// Reception de la valeur stockée
	tempReg[1] = SPI_TRANCEIVER_CHAR(0);
	
	while (ADC_2_ACQ_DONE == 0){};						// Attente fin Acquisition
		
	ChipSelect(ADC_2);
	tempReg[2] = SPI_TRANCEIVER_CHAR(0);				// Reception de la valeur stockée
	tempReg[3] = SPI_TRANCEIVER_CHAR(0);
		
	ChipSelect(None);
		
	unsigned int Value = ((tempReg[0] << 8) | tempReg[1]);	// Mise en forme pour gérer les valeures négatives
	unsigned long Value2 = ((tempReg[2] << 8) | tempReg[3]);	// Mise en forme pour gérer les valeures négatives
	unsigned long Value3;

	Value	= ADC_Add_Offset(Value,  Offset);
	Value2	= ADC_Add_Offset(Value2, Offset);

	Value3 = Value + ((Value2 << 16) & 0xFFFF0000);
	
	return (Value3);											// Mise en forme de la valeure lue
}


/*************************************************************************/
/*                       ADC_Get_FAST_Channel_Value	 				     */
/*************************************************************************/
// Function used when Hi Speed Acquisition is required

unsigned int ADC_Get_FAST_Channel_Value(unsigned int Offset)
{
	unsigned char tempReg[2];
	
	ChipSelect(ADC_1);
	
	SPI_TRANCEIVER_CHAR(ADS_1120_START);
	ADC_1_ACQ_DONE = 0;									// Reset du Flag d'acquisition terminée	ADC_SET_MICRO_TO_SLEEP(DRDY_2_UNUSED);
	
	while (ADC_1_ACQ_DONE == 0){};						// Attente fin Acquisition
	
	ChipSelect(ADC_1);
	
	tempReg[0] = SPI_TRANCEIVER_CHAR(0);				// Reception de la valeur stockée
	tempReg[1] = SPI_TRANCEIVER_CHAR(0);
	
	ChipSelect(None);
	
	unsigned int Value = ((tempReg[0] << 8) | tempReg[1]);	// Mise en forme pour gérer les valeures négatives
	Value	= ADC_Add_Offset(Value, Offset);

	return (Value);											// Mise en forme de la valeure lue
	
}

/*************************************************************************/
/*                             ADC_CONFIG			 				     */
/*************************************************************************/
// Function used to configure the 4 register of the ADS1120

void ADC_CONFIG(unsigned char ADC_Id, unsigned char Channel, unsigned char SPS, unsigned char PGA, unsigned char REF)
{
	unsigned char ADS_1120_Conf_Reg_0;
	unsigned char ADS_1120_Conf_Reg_1;
	unsigned char ADS_1120_Conf_Reg_2;
	unsigned char ADS_1120_Conf_Reg_3;

	ADS_1120_Conf_Reg_0	=	(Channel << 4) + (PGA << 1) + ADS_1120_PGA_OFF;		// Configuration des registres
	ADS_1120_Conf_Reg_1	=	(SPS << 5) + (ADS_1120_NORMAL_MODE << 3)	+ (ADS_1120_SINGLE_SHOT_MODE << 2) + (ADS_1120_INTERNAL_T_OFF << 1) + ADS_1120_BURN_OUT_OFF;
	ADS_1120_Conf_Reg_2	=	(REF << 6) + (ADS_1120_FILTER_OFF << 4) + (ADS_1120_PSW_OPEN << 3) + ADS_1120_IDAC_OFF;
	ADS_1120_Conf_Reg_3	=	(ADS_1120_IDAC1_OFF << 5) + (ADS_1120_IDAC2_OFF << 2) + (ADS_1120_DRDY_ON_DOUT << 1) + 0;
	
	ChipSelect(ADC_Id);				
	_delay_us(10);					//10us
	
	SPI_TRANCEIVER_CHAR(ADS_1120_WRITE_REG | 3);
	
	SPI_TRANCEIVER_CHAR(ADS_1120_Conf_Reg_0);			// Configuration du registre 0
	SPI_TRANCEIVER_CHAR(ADS_1120_Conf_Reg_1);			// Configuration du registre 1
	SPI_TRANCEIVER_CHAR(ADS_1120_Conf_Reg_2);			// Configuration du registre 2
	SPI_TRANCEIVER_CHAR(ADS_1120_Conf_Reg_3);			// Configuration du registre 3
	_delay_us(10);
}


/*************************************************************************/
/*                         ADC_Change_Settings		 				     */
/*************************************************************************/
// Function used to configure only the Channel and the PGA

void ADC_Change_Settings(unsigned char Channel, unsigned char PGA)
{
	unsigned char ADS_1120_Conf_Reg_0;
	ADS_1120_Conf_Reg_0 = (Channel << 4) + (PGA << 1) + ADS_1120_PGA_OFF;		// Configuration des registres
	
	ChipSelect(ADC_1);									//	CS_ADS1118 = 0	Chip Enable
	_delay_us(10);					//10us
	
	SPI_TRANCEIVER_CHAR(ADS_1120_WRITE_REG | 0);
	
	SPI_TRANCEIVER_CHAR(ADS_1120_Conf_Reg_0);			// Configuration du registre 0
	_delay_us(10);
}

/*************************************************************************/
/*                       ADC_SET_MICRO_TO_SLEEP			 			     */
/*************************************************************************/
// Function used to put the Microcontroller in sleep when acquiring

void ADC_SET_MICRO_TO_SLEEP(unsigned char DRDY_2)
{
	set_sleep_mode(SLEEP_MODE_PWR_SAVE);				// Config de la mise en veille du micro pendant l'acquisition
	EICRA &=   ~(1 << ISC01);							// Config de l'interruption de fin de mesure
	EICRA &=   ~(1 << ISC00);							// INT0 déclanchée sur LOW LEVEL
	
	EIFR  |=	(1 << INTF0);							// Reset du Flag de l'interuption 0
	EIMSK |=	(1 << INT0);							// Selection de l'intéruption 0
	
	if(DRDY_2 == USED)									// Use if 2 ADC
	{
		EICRA &=   ~(1 << ISC11);						// Config de l'interruption de fin de mesure
		EICRA &=   ~(1 << ISC10);						// INT1 déclanchée sur LOW LEVEL
		
		EIFR  |=	(1 << INTF1);						// Reset du Flag de l'interuption 1
		EIMSK |=	(1 << INT1);						// Selection de l'intéruption 1
	}
	
	sei();
	sleep_mode();										// Go to sleep
	sei();
}

/*************************************************************************/
/*                            ADC_Add_Offset			 			     */
/*************************************************************************/
// Function used to add an offset to the read value in order to always
// have a positive value recorded in memory

unsigned int ADC_Add_Offset (unsigned int Value, unsigned int Offset)
{
	if ((Value & 0x8000) == 0x8000)							// Si oui, la valeur est négative
	{
		Value -= 1;											// On soustrait 1 est on fait le complément à 2
		Value ^= 0xFFFF;
		Value = Offset - Value;
	}
	else Value += Offset;
	
	return Value;
}














/*
	unsigned char TEST_ADC;
	
	SPI_TRANCEIVER_CHAR(ADS_1120_READ_REG | 4);				// Test The good writing of the Register
	TEST_ADC = SPI_TRANCEIVER_CHAR(0);
	
	if (TEST_ADC != ADS_1120_Conf_Reg_1)					// Action if the Register is not
	{														// correctly written
		ChipSelect(None);
		_delay_us(100);
		ChipSelect(ADC_1);
		_delay_us(100);
		SPI_TRANCEIVER_CHAR(ADS_1120_RESET);
		_delay_ms(100);
		
		SPI_TRANCEIVER_CHAR(ADS_1120_WRITE_REG | 4);
		SPI_TRANCEIVER_CHAR(ADS_1120_Conf_Reg_1);
		_delay_us(100);
		
		SPI_TRANCEIVER_CHAR(ADS_1120_READ_REG | 4);
		TEST_ADC = SPI_TRANCEIVER_CHAR(0);
		
		if (TEST_ADC != ADS_1120_Conf_Reg_1)
		{
			Logger_Stop();
		}
	}
*/







/*
void ADC_TEST(unsigned char ADC_Id, unsigned char REG_CHECK_VALUE)
{
	unsigned char TEST_ADC;
	
	ChipSelect(ADC_Id);

	SPI_TRANCEIVER_CHAR(ADS_1120_READ_REG);
	TEST_ADC = SPI_TRANCEIVER_CHAR(0);

	if (TEST_ADC != REG_CHECK_VALUE)
	{
		Logger_Stop();
	}
	//else return 0;

}
*/

/*
		ChipSelect(None);									//	CS_ADS1118 = 0	Chip Enable
		_delay_us(100);
		ChipSelect(ADC_1);
		_delay_us(100);							//	CS_ADS1118 = 0	Chip Enable
		SPI_TRANCEIVER_CHAR(ADS_1120_RESET);
		_delay_ms(100);
		
		SPI_TRANCEIVER_CHAR(ADS_1120_WRITE_REG | 3);
		SPI_TRANCEIVER_CHAR(ADS_1120_Conf_Reg_0);			// Configuration du registre 0
		SPI_TRANCEIVER_CHAR(ADS_1120_Conf_Reg_1);			// Configuration du registre 1
		SPI_TRANCEIVER_CHAR(ADS_1120_Conf_Reg_2);			// Configuration du registre 2
		SPI_TRANCEIVER_CHAR(ADS_1120_Conf_Reg_3);			// Configuration du registre 3
		_delay_us(100);
		
		SPI_TRANCEIVER_CHAR(ADS_1120_READ_REG);
		TEST_ADC = SPI_TRANCEIVER_CHAR(0);
		
		if (TEST_ADC != ADS_1120_Conf_Reg_0)
		{
			Logger_Stop();
		}
*/


/************************************************************************/
/*                     ADS_1118_Get_Channel_Value                       */
/************************************************************************/

// Renvoie la valeur d'une voie de mesure de l'ADC.
// Paramètres : Numéro de voie, sample par Secondes, Gain, Offset

/*unsigned int ADC_Get_Channel_Value(unsigned char Channel, unsigned char SPS, unsigned char PGA, unsigned char REF, signed int OFFSET)
{
	unsigned char tempReg[2];
	
	unsigned char ADS_1120_Internal_T;						// Flag pour indiquer si mesure de température interne
	
	unsigned char ADS_1120_Conf_Reg_0;
	unsigned char ADS_1120_Conf_Reg_1;
	unsigned char ADS_1120_Conf_Reg_2;
	unsigned char ADS_1120_Conf_Reg_3;


	if (Channel == 0xFF)									// Vérification de la voie de mesure
	{
		ADS_1120_Internal_T = ADS_1120_INTERNAL_T_ON;		// Selection du mode d'acquisition de la température interne
	}
	else 
	{
		ADS_1120_Internal_T = ADS_1120_INTERNAL_T_OFF;		// Selection du mode d'acquisition Normal				
	}						
	

	ADS_1120_Conf_Reg_0	=	(Channel << 4) + (PGA << 1) + ADS_1120_PGA_OFF;		// Configuration des registres
	ADS_1120_Conf_Reg_1	=	(SPS << 5) + (ADS_1120_NORMAL_MODE << 3)	+ (ADS_1120_SINGLE_SHOT_MODE << 2) + (ADS_1120_Internal_T << 1) + ADS_1120_BURN_OUT_OFF;
	ADS_1120_Conf_Reg_2	=	(REF << 6) + (ADS_1120_FILTER_OFF << 4) + (ADS_1120_PSW_OPEN << 3) + ADS_1120_IDAC_OFF;
	ADS_1120_Conf_Reg_3	=	(ADS_1120_IDAC1_OFF << 5) + (ADS_1120_IDAC2_OFF << 2) + (ADS_1120_DRDY_ON_DOUT << 1) + 0;
	
	
	
	ChipSelect(ADC_1);									//	CS_ADS1118 = 0	Chip Enable
	_delay_us(10);					//10us
	

	SPI_TRANCEIVER_CHAR(ADS_1120_WRITE_REG | 3);
			
	SPI_TRANCEIVER_CHAR(ADS_1120_Conf_Reg_0);			// Configuration du registre 0
	SPI_TRANCEIVER_CHAR(ADS_1120_Conf_Reg_1);			// Configuration du registre 1
	SPI_TRANCEIVER_CHAR(ADS_1120_Conf_Reg_2);			// Configuration du registre 2
	SPI_TRANCEIVER_CHAR(ADS_1120_Conf_Reg_3);			// Configuration du registre 3
	_delay_us(100);
	
	unsigned char TEST_ADC;
	
	SPI_TRANCEIVER_CHAR(ADS_1120_READ_REG);
	TEST_ADC = SPI_TRANCEIVER_CHAR(0);
	
	if (TEST_ADC != ADS_1120_Conf_Reg_0)
	{
		ChipSelect(None);									//	CS_ADS1118 = 0	Chip Enable
		_delay_us(100);
		ChipSelect(ADC_1);		
		_delay_us(100);							//	CS_ADS1118 = 0	Chip Enable
		SPI_TRANCEIVER_CHAR(ADS_1120_RESET);
		_delay_ms(100);
		
		SPI_TRANCEIVER_CHAR(ADS_1120_WRITE_REG | 3);
		SPI_TRANCEIVER_CHAR(ADS_1120_Conf_Reg_0);			// Configuration du registre 0
		SPI_TRANCEIVER_CHAR(ADS_1120_Conf_Reg_1);			// Configuration du registre 1
		SPI_TRANCEIVER_CHAR(ADS_1120_Conf_Reg_2);			// Configuration du registre 2
		SPI_TRANCEIVER_CHAR(ADS_1120_Conf_Reg_3);			// Configuration du registre 3
		_delay_us(100);
		
		SPI_TRANCEIVER_CHAR(ADS_1120_READ_REG);
		TEST_ADC = SPI_TRANCEIVER_CHAR(0);
		
		if (TEST_ADC != ADS_1120_Conf_Reg_0)
		{
			Logger_Stop();
		}
			
		
		
	}
	
	SPI_TRANCEIVER_CHAR(ADS_1120_START);
	ADC_1_ACQ_DONE = 0;									// Reset du Flag d'acquisition terminée									
	set_sleep_mode(SLEEP_MODE_PWR_SAVE);				// Config de la mise en veille du micro pendant l'acquisition
		
	EICRA |= 0 << ISC01;								// Config de l'interruption de fin de mesure
	EICRA |= 0 << ISC00;								// INT0 déclanchée sur front descandant
	
	EIFR   = 0x01;										// Reset du Flag de l'interuption 0
	EIMSK |= 1 << INT0;									// Selection de l'intéruption 0
	
	sei();
	sleep_mode();										// Passe en veille
	sei();			
													
	while (ADC_1_ACQ_DONE == 0){};						// Attente fin Acquisition 
	
	Time_to_ADC = 0;
		
	do {													// Attente fin écriture
		Time_to_ADC++;										// WatchDog
	} while ((ADC_1_ACQ_DONE == 0) && (Time_to_ADC < 5000));	// 
	
	ADC_1_ACQ_DONE = 0;
	
	if(Time_to_ADC >= 5000)
	{
		ERROR |= 0b00000100;							// Log de l'erreur
		Logger_Stop();									// Arrete l'enregistrement
		ChipSelect(None);
	}
	else
	{
		ChipSelect(ADC_1);
		
		tempReg[0] = SPI_TRANCEIVER_CHAR(0);				// Reception de la valeur stockée
		tempReg[1] = SPI_TRANCEIVER_CHAR(0);
		
		
	}
		
	
	if (Channel == 0xFF)										// Si la voie selectionnée est la voie de température interne
	{
		unsigned int T_Value = ((tempReg[0] << 6) + (tempReg[1] >>2));		
		if ((T_Value & 0x2000) == 0x2000)						// Est-ce que le MSB (bit 14) est à 1
		{
			T_Value -= 1;										//   If so, the temperature is negative 
			T_Value ^= 0x3FFF;									// On soustrait 1 est on fait le complément à 2
			
			return (Internal_Temp_Offset -(T_Value * 3.125));	// De manière à avoir une valeur toujours positive, on ajoute un Offset
		}
		
		else 
		{
			return ((T_Value * 3.125) + Internal_Temp_Offset);	// Calcul pour les températures positives
		}
	}
	else 
	{
		unsigned int Value = ((tempReg[0] << 8) | tempReg[1]);	// Mise en forme pour gérer les valeures négatives

		if ((Value & 0x8000) == 0x8000)							// Si oui, la valeur est négative
		{
			Value -= 1;											// On soustrait 1 est on fait le complément à 2
			Value ^= 0xFFFF;
			Value = OFFSET - Value;
		}
		else
		{
			Value	+= OFFSET;									// De manière à avoir une valeur toujours positive, on ajoute un Offset
		}			
		return (Value);											// Mise en forme de la valeure lue			
	}	
	
}
*/