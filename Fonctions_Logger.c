/************************************************************************/
/*************************** DESCRIPTION ********************************/
/************************************************************************/
/*																		*/
/*	Name: Fonctions_Logger.c											*/
/*	Version: 1.0														*/
/*																		*/
/*                                                                      */
/************************************************************************/
/************************** History Revisions ***************************/
/*																		*/
/* 1.0 - 07/06/2017 - Creation by J.BARBARAS 							*/
/*																		*/
/*																		*/
/************************************************************************/

#include "Hardware.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "Fonctions_Logger.h"
#include "CALIB_Internal_RC.h"

#include "EEPROM_ST_M95.h"
#include "ADC_ATMEGA_168.h"

#include "DRIVER_UART.h"
#include "Driver_QLEVER_T3.h"


#include "alim.h"
#include "ADS1120.h"
#include "DRIVER_SPI.h"

#include "MEMORY_EXT.h"

#include <avr/sleep.h>




/************************************************************************/
/*                              BOARD INIT                              */
/************************************************************************/
void BOARD_INIT()
{
	/************************************************************************/
	/*              Map ISR vectors to application section                  */
	/************************************************************************/
	cli();					// disable interupts
	MCUCR = (1<<IVCE);		// enable interrupt vectors change
	MCUCR = (0<<IVSEL);		// move interrupt vectors to application
	
					/***************  Change Clock  *****************/	CLKPR	=	(1<<CLKPCE);										// Préparation à la modification du Clock Prescale	CLKPR	=	(0<<CLKPS3)|(0<<CLKPS2)|(1<<CLKPS1)|(0<<CLKPS0);	// Prescaler = 4
					/*******************  I/Os  *********************/	// PORT D 	DDRD	&= ~(1 << PD0);		// RX			INPUT
	DDRD	|=  (1 << PD1);		// TX			OUTPUT	DDRD	&= ~(1 << PD2);		// DRDY			INPUT			DDRD	&= ~(1 << PD3);		// DRDY 2		INPUT	DDRD	|=  (1 << PD4);		// AUTO ALIM 1	OUTPUT		DDRD	|=  (1 << PD5);		// CS_MEM_4		OUTPUT			DDRD	|=  (1 << PD6);		// CS_ADC_2		OUTPUT			DDRD	&= ~(1 << PD7);		// 				INPUT		UNUSED					// PORT B 	DDRB	|=	(1 << PB0);		// V_AUX		OUTPUT			DDRB	|=	(1 << PB1);		// VDD_ANA		OUTPUT			DDRB	|=	(1 << PB2);		// CS_MEM_2		OUTPUT			DDRB	|=	(1 << PB3);		// MOSI			OUTPUT
	DDRB	&= ~(1 << PB4);		// MISO			INPUT
	DDRB	|=	(1 << PB5);		// SCK			OUTPUT		// PORT C 	DDRC	&= ~(1 << PC0);		// 				INPUT		UNUSED	DDRC	|=	(1 << PC2);		// CS_ADC_1		OUTPUT			DDRC	|=	(1 << PC3);		// VDD_NUM 		OUTPUT			DDRC	|=	(1 << PC4);		// CS_MEM_1		OUTPUT		
	DDRC	|=	(1 << PC5);		// CS_MEM_3		OUTPUT			
	//DDRC	&= ~(1 << PC6);		// RESET		INPUT	
	
	#ifdef OPTION_BIGMEMORY
	DDRC	|=	(1 << PC1);		// CS_ADC_5		OUTPUT		
	#else
	DDRC	&= ~(1 << PC1);		// CS_ADC_5		INPUT		UNUSED
	#endif
	
		
	
	ChipSelect(Clear);				// Tous les CS à 0			PORTD	&= ~(1 <<PD3);		// VDD_MICRO	=	0
	PORTB	&= ~(1 <<PB2);		// VDD_ANA		=	0
	PORTC	&= ~(1 <<PC3);		// VDD_NUM		=	0
		
	PORTB	&= ~(1 <<PB3);		// MISO			=	0	PORTB	&= ~(1 <<PB5);		// SCK			=	0
	
	
	/**************  TIMERS & autres  *****************/	TIMSK0	&= ~(1<<OCIE0A);	// Output Compare A Match Interrupt
	TIMSK0	&= ~(1<<OCIE0A);	// Output Compare B Match Interrupt
	TIMSK0	&= ~(1<<TOIE0);		// Timer1 overflow interrupt	TCCR0A	=	0x00;			// Timer0 INIT	TCCR0B	=	0x00;				
	ACSR	|=	(1<<ACD);			// Switch off Analog Comparator
	
	PRR		|=  1 << PRTIM0;		// Eteint Timer 0
	PRR		|=	1 << PRTWI;			// Eteint I2C
	//PRR		|=	1 << PRSPI;			// Eteint SPI
			
	/*******************  ADC  **********************/
	ADCSRA	&=	~(1 << ADEN);		// 0 = Disables the ADC
	ADCSRB	=	0x00;				// ADC interne INIT
	DIDR0	=	0x3F;				// ADC5...0 Digital Input Buffer Disable for power reduction
	DIDR1	=	0x03;				// AIN1, AIN0 Digital Input Disable for power reduction
	PRR		|=	1 << PRADC;			// Eteint ADC
	
		
	/******************  OSCALL  *********************/
	CalibrationRoutine();			// Calibration du RC interne sur 1,8432MHz
	//OSCCAL_VALUE = OSCCAL;
	PRR		|=  1 << PRTIM1;		// Eteint Timer 1 utilisé pour la calibration

	/******************  UART  *********************/
	USART_Init();				// Initialise l'USART

	USART_Transmit(0xee);
		
	/*************  EEPROM PARAMS  *****************/
	Load_Params();				// Charge les paramètres contenus dans l'EEPROM

	
	/**************  Interuptions  *****************/
	
	EIMSK	= 0x00;					// Désactive les External Interrupts
	EIFR	= 0x00;					// Initialise les flags des External Interrupts
	
	PCMSK2	 = 0;					// Initialisation de Pin Change Mask Register 2
	PCMSK2	|= (1 << PCINT16);		// Selction du PCINT 16 : pin RX	sur ATMEGA168
	PCICR	&=	~(1 << PCIE2);		// Active les interruptions sur les PCINT					// ATTENTION : |=	(1
	
	/****************  Variables  *******************/
	
	Flag_UART_in_use			= 0;
	UART_DOUBLE_SPEED			= 0;
	tempo						= 0;
	UART_Ready_to_Send			= 0;
	AMB_Temp_ADC				= 22000;
	ACQ_Rate					= 0;
	ADC_1_ACQ_DONE				= 0;		
	ERROR						= 0;
	Logger_Flag_ACQ				= 0;
	Flag_Min_Pile				= 0;
	Write_In_Progress			= 0;
	Update_Nombre_ACQ_Done		= 0;
	
	DevBoard.RECORD_ID			= 0;
	DevBoard.CADENCE_ID			= 0;
	DevBoard.statut				= 0;
	
	CURRENT_RECORD.ADDR_DATA	= 0;
	CURRENT_RECORD.DEBUT_DATA	= 0;
	
	i_ACQ_Bytes_buffered		= 0;
	
	
	
	//	indice_1 = -0.00000379626735730883;
	//	indice_2 =  0.166590074301301;
	//	indice_3 = -1723.74951857566;
	
	indice_1 = -0.000003796267264;
	indice_2 =  0.16659007;
	indice_3 = -1723.74951;
	
}


/************************************************************************/
/*           Chargement des Paramètres EEPROM en local                  */
/************************************************************************/

void Load_Params(void)
{
	ALIM_MEM_ON();
	_delay_ms(1);
	
	unsigned char BUFFER[32];

	EEPROM_ADDR = (long)T3_ADDR_EEPROM_START;
	MEMORY_EXT_READ(EEPROM_ADDR, BUFFER, 32);
	
	for (int i = 0; i < 8; i++)
	{
		DevBoard.Serial_Number[i] = BUFFER[i];
	}
	
	DevBoard.minimum_pile_atteint	= (((int)BUFFER[10] << 8) & 0xFF00)	+ (BUFFER[9]);
	DevBoard.Seuil_Low_Bat			= (((int)BUFFER[12] << 8) & 0xFF00)  + (BUFFER[11]);
	
	ADC1.Nb_Voies_Lues	= Conv_Char_to_nb_Bits(BUFFER[13]);		// Nombre de voies lues
	ADC1.Nb_Voies_Mem	= Conv_Char_to_nb_Bits(BUFFER[14]);		// Nombre de voies enregistrées
	
	for (int i = 0; i < 5; i++)				// Lecture des paramètres de voies de l'ADC
	{
		ADC1.Voie[i].Channel			= (BUFFER[16+(i*3)]	  & 0xF0) >> 4;
		ADC1.Voie[i].Gain				=  BUFFER[16+(i*3)]   & 0x0F;
		ADC1.Voie[i].Voltage_Reference	= (BUFFER[16+(i*3+1)] & 0xF0) >> 4;
		ADC1.Voie[i].T_Sampling			=  BUFFER[16+(i*3+1)] & 0x0F;
		ADC1.Voie[i].Type				=  BUFFER[16+(i*3+2)];
	}
	
	EEPROM_ADDR = (long)T3_ADDR_Key_Adjust;
	MEMORY_EXT_READ(EEPROM_ADDR, BUFFER, 4);
	
	for (int i = 0; i < 4; i++)
	{
		DevBoard.Adjust_key[i]	= BUFFER[i];		// TODO : inutile pour le moment
	}

	ALIM_MEM_OFF();
}


void Load_Options(void)
{
	unsigned char BUFFER[9];
	EEPROM_ADDR = (long)T3_ADDR_Threshold;
	MEMORY_EXT_READ(EEPROM_ADDR, BUFFER, 5);
	
	// Lecture Paramètres de démarrage sur seuil
	DevBoard.Threshold_Start	= (((int)BUFFER[1] << 8) & 0xFF00) +  (BUFFER[0]);
	DevBoard.Threshold_Stop		= (((int)BUFFER[4] << 8) & 0xFF00) +  (BUFFER[3]);
	DevBoard.Threshold_Channel	= BUFFER[2];
	DevBoard.Threshold_Status = 0;
	
	if (DevBoard.Threshold_Start != 0)				// Si le démarrage sur seuil est activé,
	{
		DevBoard.Threshold_Status |= 0b00000001;	// on met le bit 0 à 1
	}
	if (DevBoard.Threshold_Stop != 0)				// Si l'arret sur Seuil est activé,
	{
		DevBoard.Threshold_Status |= 0b00000010;	// On met le bit 1 à 1
	}
	
	EEPROM_ADDR  = (long) T3_ADDR_Mem_Turn;
	MEMORY_EXT_READ(EEPROM_ADDR, &DevBoard.Mem_Tournante, 1);	// Mise à jour du Flag de la mémoire Tournante	
	
	EEPROM_ADDR = (long)T3_ADDR_Minimum_Pile_atteint;
	MEMORY_EXT_READ(EEPROM_ADDR, BUFFER, 4);

	DevBoard.minimum_pile_atteint	= (((int)BUFFER[1] << 8) & 0xFF00)	+ (BUFFER[0]);
	DevBoard.Seuil_Low_Bat			= (((int)BUFFER[3] << 8) & 0xFF00)  + (BUFFER[2]);	
}





/************************************************************************/
/*                                START                                 */
/************************************************************************/ 

void Logger_Start(unsigned char Record_Number)
{
	
	Logger_LOAD_RECORD(Record_Number);							// Chargement des paramètres de l'enregistrement en cours
	
	Stats_Init();
	
	CURRENT_RECORD.ADDR_DATA	= CURRENT_RECORD.DEBUT_DATA;	// Initialise l'adresse de début d'enregistrement
	
	ERROR					= 0;
	MEM_FULL				= 0;
	Start_Log_Flag			= 1;
	i_ACQ_Bytes_buffered	= 0;	
		
	DevBoard.statut				= Waiting_StartTime;			// Start en attente de la date programmée
	
}	


/************************************************************************/
/*                                 STOP                                 */
/************************************************************************/ 

void Logger_Stop()
{
	cli();
	if (DevBoard.statut != 0)
	{
		DevBoard.statut		= 0x00;
		
		TIFR2	|= (1<<OCF2A);							// TIMER 2	Clear Interrupt B flag
		while (ASSR & ((1 << TCN2UB)|(1 << TCR2BUB)));	// wait for osc not busy
		TIMSK2	&= ~(1<<OCIE2A);					// Masque l'interruption
		OCR2A	 = 0x7F;
		TCCR2B	 = (1<<CS22) | (1<<CS21) | (0<<CS20);	// Prescaler :  (100 = 64) (101 = 128) ( 110 = 256)
		
		//TIMSK2	&= ~(1<<TOIE2);						// Masque l'interruption
				
		
		
		ACQ_Rate = F_1Hz;
		ALIM_MEM_ON(); _delay_ms(1);
		
		
		if (MEM_FULL == 0) {MEMORY_EXT_WRITE((CURRENT_RECORD.ADDR_DATA), BUFFER_ACQ_EEPROM, i_ACQ_Bytes_buffered);}
		Logger_SET_NOMBRE_ACQ_DONE(CURRENT_RECORD.Id, CURRENT_CADENCE.Id,CURRENT_CADENCE.Nombre_ACQ_DONE);
		Logger_SET_ADDR_DEBUT_DATA(CURRENT_RECORD.Id + 1, CURRENT_RECORD.ADDR_DATA);
		Logger_Write_Error();
				
		Stats_Save(CURRENT_CADENCE.Nombre_ACQ_DONE);
		
		ALIM_MEM_OFF();
	}
	
	_delay_us(100);
	VDD_MICRO_OFF;			// Extinction de l'alimentation
	_delay_ms(10);
	_delay_ms(100);
	_delay_ms(500);
}	


/************************************************************************/
/*                          GESTION CADENCES                            */
/************************************************************************/ 

void Logger_LOAD_CADENCE (unsigned char Cadence_Number)
{
	
	//TCCR2B	 = (1<<CS22) | (0<<CS21) | (1<<CS20);	// Prescaler :  (100 = 64) (101 = 128) ( 110 = 256)
	//OCR2A	 = 0xFF;
	
	ACQ_Rate = F_1Hz;
	
	EEPROM_ADDR  = (long) T3_ADDR_CADENCE_Params;
	EEPROM_ADDR += Cadence_Number * 5;
	
	unsigned char BUFFER[5];
	
	ALIM_MEM_ON();
	MEMORY_EXT_READ(EEPROM_ADDR, BUFFER, 5);
	ALIM_MEM_OFF();
	
	CURRENT_CADENCE.Id					= Cadence_Number;
	CURRENT_CADENCE.Nombre_ACQ_DONE		= 0L;
	CURRENT_CADENCE.Nombre_ACQ_TO_DO	= ((unsigned long)BUFFER[2] << 16) + ((unsigned long)BUFFER[1] << 8) + ((unsigned long)BUFFER[0]);
	CURRENT_CADENCE.secondes			= (BUFFER[3] & 0b01111111);
	CURRENT_CADENCE.minutes				= (BUFFER[4] & 0b01111111);
	
	if ((BUFFER[4] >> 7) == 1)
	{
		unsigned char RATE = 1;
		unsigned char Multiple = 0;
		T_milliSec	= 0;
		TIMSK2	&= ~(1<<OCIE2A);						// Masque l'interruption
		TCCR2B	 = (1<<CS22) | (0<<CS21) | (1<<CS20);	// Prescaler :  (100 = 64) (101 = 128) ( 110 = 256)
		switch (BUFFER[4])
		{
			case 0b10000010:		// 2Hz - 500ms
			{
				ACQ_Rate	= F_4Hz;
				OCR2A		= 0x7F;	// Définition de l'overflow du compteur
				Multiple	= 1;
				break;
			}
			
			case 0b10000001:		// 3/4Hz - 750ms
			{
				RATE		= 3;	// Same As 4Hz execpt the rate
			}
			
			case 0b10000100:		// 4Hz - 250ms
			{
				ACQ_Rate	= F_4Hz;
				OCR2A		= 0x3F;	// Définition de l'overflow du compteur
				Multiple	= 3;
				break;
			}
			
			case 0b10001000:		// 8Hz - 125ms
			{
				ACQ_Rate	= F_8Hz;
				OCR2A		= 0x1F;	// Définition de l'overflow du compteur
				Multiple	= 7;
				break;
			}
			
			case 0b10010000:			// 16Hz - 62.5ms
			{
				ACQ_Rate	= F_16Hz;
				OCR2A		= 0x0F;		// Définition de l'overflow du compteur
				Multiple	= 15;
				ACQ_Sampling	= 0x03;		// Change the ACQ duration
				break;
			}
			
			case 0b10100000:		// 32Hz - 32ms
			{
				ACQ_Rate	= F_32Hz;
				OCR2A		= 0x07;	// Définition de l'overflow du compteur
				Multiple	= 31;
				ACQ_Sampling	= 0x04;
				break;
			}
			
			case 0b11000000:		// 64Hz - 16ms
			{
				ACQ_Rate	= F_64Hz;
				OCR2A		= 0x03;	// Définition de l'overflow du compteur
				Multiple	= 63;
				ACQ_Sampling	= 0x05;
				break;
			}
			
			case 0b11110000:		// 128Hz - 7.8ms
			{
				ACQ_Rate	= F_128Hz;
				OCR2A		= 0x01;	// Définition de l'overflow du compteur
				Multiple	= 127;
				ACQ_Sampling	= 0x06;
				break;
			}
			
			case 0b11111111:		// 256Hz - 3.9ms
			{
				ACQ_Rate	= F_256Hz;
				TCCR2B		= (1<<CS22) | (0<<CS21) | (0<<CS20);	// Prescaler :  (100 = 64) (101 = 128) ( 110 = 256) 
				OCR2A		= 0x01;	// Définition de l'overflow du compteur
				Multiple	= 255;
				ACQ_Sampling	= 0x06;
				break;
			}
		}
		CURRENT_CADENCE.ACQ_RATE = RATE + ((((CURRENT_CADENCE.secondes & 0xF0) >> 4) * 10) + (CURRENT_CADENCE.secondes & 0x0F)) * (int)(Multiple +1);
	}
	
	else 
	{
		CURRENT_CADENCE.ACQ_RATE		= ((((CURRENT_CADENCE.minutes & 0xF0) >> 4) * 600) + ((CURRENT_CADENCE.minutes & 0x0F) * 60)) + ((((CURRENT_CADENCE.secondes & 0xF0) >> 4) * 10) + (CURRENT_CADENCE.secondes & 0x0F));	
	}
	
	TIMSK2	|= (1<<OCIE2A);		//			Timer/Counter2 Output Compare Match A Interrupt Enable
	cli();
}


unsigned long Logger_GET_NBR_ACQ_TO_DO(unsigned char Cadence_Number)
{
	EEPROM_ADDR  = (long) T3_ADDR_CADENCE_Params;
	EEPROM_ADDR += (Cadence_Number * 5);
	
	unsigned char BUFFER[3];

	MEMORY_EXT_READ(EEPROM_ADDR, BUFFER, 3);
	
	return ((unsigned long)BUFFER[2] << 16) + ((unsigned long)BUFFER[1] << 8) + ((unsigned long)BUFFER[0]);	
}


void Logger_SET_NOMBRE_ACQ_DONE(unsigned char Record_Number, unsigned char Cadence_Number, unsigned long NB_ACQ_DONE)
{
	unsigned char DATA[3];
	
	EEPROM_ADDR  = (long) T3_ADDR_CADENCE_NOMBRE_ACQ_DONE;
	EEPROM_ADDR = EEPROM_ADDR + (Record_Number * 29) + (Cadence_Number * 3);
	
	DATA[0] =  NB_ACQ_DONE & 0x0000FF;
	DATA[1] = (NB_ACQ_DONE & 0x00FF00) >> 8;
	DATA[2] = (NB_ACQ_DONE & 0xFF0000) >> 16;
	
	MEMORY_EXT_WRITE(EEPROM_ADDR, DATA, 3);
}

void Logger_Write_Error()
{
	
	EEPROM_ADDR  = (long) T3_ADDR_Error_Byte;
		
	EEPROM_ST_M95_WRITE(EEPROM_ADDR, (unsigned char*)&ERROR, 1);
}



/************************************************************************/
/*                Vérification du Minimum pile atteint                  */
/************************************************************************/
void Logger_Check_Min_Pile(unsigned int valeur_Pile)
{
	if (valeur_Pile < DevBoard.minimum_pile_atteint)
	{
		DevBoard.minimum_pile_atteint = valeur_Pile;
		Flag_Min_Pile = 1;
	}
	
	if (valeur_Pile < DevBoard.Seuil_Low_Bat)
	{
		ALIM_MEM_ON(); //_delay_ms(1);
		SAVE_Min_Pile();
		Logger_Stop();
	}
}

void SAVE_Min_Pile()
{	
	unsigned char Min_PILE[2];
	
	Min_PILE[0] = DevBoard.minimum_pile_atteint & 0x00FF;
	Min_PILE[1] = DevBoard.minimum_pile_atteint >> 8;
	
	MEMORY_EXT_WRITE((long) T3_ADDR_Minimum_Pile_atteint, Min_PILE, 2);		
}

/****************************** Tension Pile ****************************/
unsigned int Get_Pile()
{
	ADC_ATMEGA_168_INIT(0);									// Initialisation de l'ADC de l'ATMEGA
	_delay_us(10);
	
	BAT_Voltage	= ADC_ATMEGA_168_CONV_VOLTAGE(ADC_ATMEGA_168_Get_ACQ(0x07));	// Mesure de la tension Pile
	if (BAT_Voltage > 0xFFF0)
	{
		BAT_Voltage = 0;
	}
	
	ADC_ATMEGA_168_SWITCH_OFF();							// POWER OFF ADC
	
	return BAT_Voltage;										// en 10aine de milliVolts
}

void Check_Pile()
{
	BAT_Voltage	=	Get_Pile();					// Mesure de la tension Pile
	//BAT_Voltage = 0x0268;
	Logger_Check_Min_Pile(BAT_Voltage);			// Vérification du minimum Pile
	
	
}




/************************************************************************/
/*                      STATISTIQUES D'UTILISATION                      */
/************************************************************************/

void Stats_Init ()		// Initialisation des valeurs tempon de statistique
{
	NB_SECONDES_DONE = 0;
	for (int i=0; i<9 ; i++)
	{
		Stats.Temp_Range[i] = 0;
	}
	Stats.Temperature_Min_Cycle = 0xFFFF;
	Stats.Temperature_Max_Cycle = 0x0000;
}



void Stats_Log(unsigned int Temperature)
{
	// Température en °C = (Temperature - 20000)/100
	unsigned char i;

	if		(Temperature <= 14000)		{i=0;}	// T < -60°C			0x3C40
	else if (Temperature < 17000)		{i=1;}	// -60°C < T < -30°C	0x3880
	else if (Temperature < 20000)		{i=2;}	// -30°C < T < 0°C		0x2000
	else if (Temperature < 23000)		{i=3;}	// 30°C > T >  0°C
	else if (Temperature < 26000)		{i=4;}	// 60°C > T > 30°C		0x03C0
	else if (Temperature < 29000)		{i=5;}	// 90°C > T > 60°C		0x0780
	else if (Temperature < 32000)		{i=6;}	// 120°C > T > 90°C		0x0B40
	else if (Temperature < 34000)		{i=7;}	// 140°C > T > 120°C	0x0F00
	else if (Temperature >= 34000)		{i=8;}	// T > 140°c			0x1180
	
	Stats.Temp_Range[i] ++;
	
	if (Temperature < Stats.Temperature_Min_Cycle)				// Log de la temperature Minimum atteinte pendant le cycle
	{
		Stats.Temperature_Min_Cycle = Temperature;
	}
	else if (Temperature > Stats.Temperature_Max_Cycle)			// Log de la temperature Maximum atteinte pendant le cycle
	{
		Stats.Temperature_Max_Cycle = Temperature;
	}
	NB_SECONDES_DONE++;
}


void Stats_Save(unsigned long NB_ACQ_DONE)
{
	unsigned char BUFFER[64];
	
	EEPROM_ADDR	=	(long) T3_ADDR_Logger_Stats;
	MEMORY_EXT_READ(EEPROM_ADDR, BUFFER, 64);
	
	for (int i=0; i<9 ; i++)
	{
		Add_Long_To_Tab((long)Stats.Temp_Range[i], BUFFER, i*4);	// Ajoute Temp_Range 1 à BUFFER[0123]
	}
	

	
	Add_Long_To_Tab(Start_Log_Flag,BUFFER,44);
	Add_Long_To_Tab(NB_SECONDES_DONE,BUFFER,48);
	Add_Long_To_Tab(NB_SECONDES_DONE,BUFFER,52);
	Add_Long_To_Tab(NB_ACQ_DONE,BUFFER,56);
	Add_Long_To_Tab(NB_ACQ_DONE,BUFFER,60);
	
	
	if (Stats.Temperature_Min_Cycle <  Tab_to_Int(BUFFER,36))	//	Mise à jour des température Min et Max atteinte par le logger	
	{
		Int_to_Tab(Stats.Temperature_Min_Cycle, BUFFER, 36);	// Sauvegarde de la température minimum atteinte
	}
	if (Stats.Temperature_Max_Cycle >  Tab_to_Int(BUFFER,38))	
	{
		Int_to_Tab(Stats.Temperature_Max_Cycle, BUFFER, 38);	// Sauvegarde de la température Maximale atteinte
	}
		
	Int_to_Tab(Stats.Temperature_Min_Cycle, BUFFER, 40);		// Mise à jour des température Min et max atteinte par le logger
	Int_to_Tab(Stats.Temperature_Max_Cycle, BUFFER, 42);		// pendant ce Cycle
	

	MEMORY_EXT_WRITE(EEPROM_ADDR, BUFFER, 64);					// Ecriture en mémoire de toutes ces informations
	
	Start_Log_Flag = 0;
}



/************************************************************************/
/*                           GESTION RECORDS                            */
/************************************************************************/ 


void Logger_LOAD_RECORD (unsigned char Record_Number)
{
	CURRENT_RECORD.DEBUT_DATA = Logger_GET_ADDR_DEBUT_DATA(Record_Number);
}


unsigned long Logger_GET_NOMBRE_ACQ_DONE(unsigned char Record_Number, unsigned char Cadence_Number)
{
	ALIM_MEM_ON();
	EEPROM_ADDR  = (long) T3_ADDR_CADENCE_NOMBRE_ACQ_DONE;
	EEPROM_ADDR += (Record_Number * 29) + (Cadence_Number * 3);
	
	unsigned char BUFFER[3];

	MEMORY_EXT_READ(EEPROM_ADDR, BUFFER, 3);
	ALIM_MEM_OFF();
	return ((unsigned long)0x00 << 24) + (((unsigned long)BUFFER[2] << 16) & 0x00FF0000) + (((unsigned long)BUFFER[1] << 8) & 0x0000FF00) + ((unsigned long)BUFFER[0]);
}


unsigned long Logger_GET_ADDR_DEBUT_DATA(unsigned char Record_Number)
{
	EEPROM_ADDR  = (long) T3_ADDR_RECORD_DEBUT_DATA;
	EEPROM_ADDR += (Record_Number * 29);
	
	unsigned char BUFFER[3];

	MEMORY_EXT_READ(EEPROM_ADDR, BUFFER, 3);
		
	return ((unsigned long)0x00 << 24) + (((unsigned long) BUFFER[2] << 16) & 0x00FF0000) + (((unsigned long)BUFFER[1] << 8) & 0x0000FF00) + ((unsigned long)BUFFER[0]);
}

void Logger_SET_ADDR_DEBUT_DATA(unsigned char Record_Number, unsigned long Addr_Debut_Data)
{
	unsigned char DATA[3];
	
	EEPROM_ADDR  = (long) T3_ADDR_RECORD_DEBUT_DATA;
	EEPROM_ADDR += (Record_Number * 29);
	
	DATA[0] = Addr_Debut_Data & 0x0000FF;
	DATA[1] = (Addr_Debut_Data & 0x00FF00) >> 8;
	DATA[2] = (Addr_Debut_Data & 0xFF0000) >> 16;
	
	MEMORY_EXT_WRITE(EEPROM_ADDR, DATA, 3);
}



void Logger_UPDATE_CURRENT_RECORD()
{
	unsigned char DATA[6];
	volatile unsigned long temp_DATA;
	
	CURRENT_RECORD.Id = 0;								// Vérification de l'enregistrement 0
	EEPROM_ADDR  = (long)T3_ADDR_CADENCE_NOMBRE_ACQ_DONE;
	MEMORY_EXT_READ(EEPROM_ADDR, DATA, 6);
	

	temp_DATA = DATA[0] + DATA[1] + DATA[2] + DATA[3] + DATA[4] + DATA[5];			// Transformation en long
		
	while ((temp_DATA != 0) && CURRENT_RECORD.Id <= 14) // On recherche le premier enregistrement 
	{													// dont le nombre d'acquisitions faites est nulle
		EEPROM_ADDR += 29;								// 
		CURRENT_RECORD.Id++;
		MEMORY_EXT_READ(EEPROM_ADDR, DATA, 6);
		temp_DATA = DATA[0] + DATA[1] + DATA[2] + DATA[3] + DATA[4] + DATA[5];
	}
}
			

void Logger_SAVE_START_DATE (unsigned char *START_TIME, unsigned char Record_Number)
{
	EEPROM_ADDR  = (long)T3_ADDR_RECORD_START_DATE;
	EEPROM_ADDR += (Record_Number * 29);			// Offset pour être sur le bon enregistrement
	
	MEMORY_EXT_WRITE(EEPROM_ADDR, START_TIME, 7);	// Sauvegarde de la date de Start
}


void Logger_UPDATE_START_DATE (unsigned char Record_Number)
{
	EEPROM_ADDR  = (long)T3_ADDR_RECORD_START_DATE;
	EEPROM_ADDR += (Record_Number * 29);			// Offset pour être sur le bon enregistrement
	
	unsigned char START_TIME[7];
	START_TIME[0]	= 0;
	START_TIME[1]	= (RTC_T3.secondes	/ 0xA * 0x10) + (RTC_T3.secondes % 0xA);	// Seconds
	START_TIME[2]	= (RTC_T3.minutes	/ 0xA * 0x10) + (RTC_T3.minutes  % 0xA);	// Minutes
	START_TIME[3]	= (RTC_T3.heures	/ 0xA * 0x10) + (RTC_T3.heures   % 0xA);	// Hours
	START_TIME[4]	= (RTC_T3.jours		/ 0xA * 0x10) + (RTC_T3.jours	 % 0xA);	// Day
	START_TIME[5]	= (RTC_T3.mois		/ 0xA * 0x10) + (RTC_T3.mois	 % 0xA);	// Month
	START_TIME[6]	= (RTC_T3.annees);												// Years

	
	MEMORY_EXT_WRITE(EEPROM_ADDR, START_TIME, 7);	// Sauvegarde de la date de Start
}




/************************************************************************/
/*					     	       MESURES			                    */
/************************************************************************/



/********************* Acquisitions ADC Externe *************************/

void Get_ACQUISITIONS(unsigned char *DATA, unsigned char Nb_Voies)
{
	cli();
	unsigned long Voie[nb_Voies_MAX];
	ERROR |= 0b00001000;
		
	ADC_INIT();										// Initialisation de l'ADC
	_delay_us(1);									// Délai
	
	ChipSelect(None);								//	CS_ADS1118 = 0	Chip Enable


	if(ACQ_Rate < F_32Hz)
	{
		Delay_Done = 0;
			
		unsigned char warmup_delay = 2;					// Value in ms : x3.9ms (must be > 1)
		if (AMB_Temp_ADC < 20000) { warmup_delay = 6;}
		if (AMB_Temp_ADC < 18000) { warmup_delay = 8;}
	
		set_sleep_mode(SLEEP_MODE_IDLE);				// Config de la mise en veille du micro pendant le warmup
	
		t_TCNT2 = TCNT2;
	
		//if(t_TCNT2 > 0x77)	{OCR2B = (warmup_delay - 1);}				// Définition de l'overflow du compteur 
		//else				{OCR2B	= TCNT2 + (warmup_delay - 1);}
	
		OCR2B	= TCNT2 + (warmup_delay - 1);									
		
		TIFR2	|= (1<<OCF2B);							// TIMER 2	Clear Interrupt B flag
		while (ASSR & ((1 << TCN2UB)|(1 << TCR2BUB)));	// wait for osc not busy
		TIMSK2	|= (1<<OCIE2B);							// Enable Timer/Counter2 Output Compare Match B Interrupt 
		
		sei();
		sleep_mode();										// Passe en veille
		sei();
	
		while (Delay_Done == 0){};						// Attente fin Acquisition 
		Delay_Done = 0;
		
		VDD_ANA_ON;										// Allume l'alimentation Analogique
		_delay_us(2);
		
		AMB_Temp_ADC		= Get_Internal_Temp ();		// Voie de température interne
	}
	else if (ACQ_Rate < F_256Hz) {_delay_us(100);}
	
	VDD_ANA_ON;										// Allume l'alimentation Analogique
	_delay_us(2);	
	
	for (int i = 0; i < Nb_Voies; i++)				// Pour toutes les voies
	{
		switch(ADC1.Voie[i].Type)
		{
			case Temperature_PT1000:
			{
			/*	if(ACQ_Rate == 0) {ACQ_Sampling = ADC1.Voie[i].T_Sampling;}
				Voie[i]	 =	0;
				ADC_CONFIG(ADC_1, ADC1.Voie[i].Channel,ACQ_Sampling,ADC1.Voie[i].Gain, ADC1.Voie[i].Voltage_Reference);
				Voie[i]	+=	ADC_Get_Channel_Value(1000);	if(ACQ_Rate == 2) {Voie[i] = Voie[i] << 1; break;}
				Voie[i]	+=	ADC_Get_Channel_Value(1000);
				break;*/
			}
			
		/*	case FAST_Measure:
			{
				Voie[i]	 =	0;
				ADC_CONFIG(ADC_1, ADC1.Voie[i].Channel,ADC1.Voie[i].T_Sampling,ADC1.Voie[i].Gain, ADC1.Voie[i].Voltage_Reference);
				Voie[i]	=	(ADC_Get_Channel_Value(1000) << 2);
				
				break;
			}*/
			
			case Pression_15bars:
			{
				if(ACQ_Rate < F_16Hz) {ACQ_Sampling = ADC1.Voie[i].T_Sampling;}
				Voie[i]	 =	0;
				ADC_CONFIG(ADC_1, ADC1.Voie[i].Channel,ACQ_Sampling,ADC1.Voie[i].Gain, ADC1.Voie[i].Voltage_Reference);
				Voie[i]	+=	ADC_Get_Channel_Value(1000);	if(ACQ_Rate > F_64Hz) {Voie[i] = Voie[i] << 1; break;}
				Voie[i]	+=	ADC_Get_Channel_Value(1000);
				break;
			}			
			
			case Double_Temperature:
			{
				VDD_AUX_ON;
				Voie[i] = 0;
				Voie[i+1] = 0;
				ADC_CONFIG(ADC_1, ADC1.Voie[i].Channel,ADC1.Voie[i].T_Sampling,ADC1.Voie[i].Gain, ADC1.Voie[i].Voltage_Reference);				// Config ADC Parameters
				ADC_CONFIG(ADC_2, ADC1.Voie[i].Channel,ADC1.Voie[i].T_Sampling,ADC1.Voie[i].Gain, ADC1.Voie[i].Voltage_Reference);				// Config ADC Parameters

				Voie[i]	  =	ADC_Get_Double_ADC_Channel_Value(0);
				Voie[i]	 +=	ADC_Get_Double_ADC_Channel_Value(0);
			
				Voie[i+1] = (Voie[i] >> 16) & 0x0000FFFF;
				Voie[i]	  =  Voie[i] & 0x0000FFFF;
				
				break;
			}
			
			case Double_Pressure_05bars:
			{
				VDD_AUX_ON;
				Voie[i] = 0;
				Voie[i+1] = 0;
				ADC_CONFIG(ADC_1, ADC1.Voie[i].Channel,ADC1.Voie[i].T_Sampling,ADC1.Voie[i].Gain, ADC1.Voie[i].Voltage_Reference);				// Config ADC Parameters
				ADC_CONFIG(ADC_2, ADC1.Voie[i].Channel,ADC1.Voie[i].T_Sampling,ADC1.Voie[i].Gain, ADC1.Voie[i].Voltage_Reference);				// Config ADC Parameters

				Voie[i]	  =	ADC_Get_Double_ADC_Channel_Value(2000);
				Voie[i]	 +=	ADC_Get_Double_ADC_Channel_Value(2000);
					
				Voie[i+1] = (Voie[i] >> 16) & 0x0000FFFF;
				Voie[i]	  =  Voie[i] & 0x0000FFFF;
				
				VDD_AUX_OFF;
				
				break;
			}
				
			
			case Dummy:
			{
				break;
			}
			
			/*case FAST_Temperature_Pressure:
			{
				Voie[i]	 =	0;
				//Voie[i]	+=	ADC_Get_Channel_Value(ADC1.Voie[i].Channel,ADC1.Voie[i].T_Sampling,ADC1.Voie[i].Gain, ADC1.Voie[i].Voltage_Reference, 0);
				Voie[i]	+=	ADC_Get_Channel_Value(0);
				
							i++;
							ADC_Change_Settings(ADC1.Voie[i].Channel, ADC1.Voie[i].Gain);
				Voie[i]	+=	ADC_Get_Channel_Value(0);
				Voie[i]	+=	ADC_Get_Channel_Value(0);
							
				break;
			}
			
	
			
			case Pression_30bars:
			{
				Voie[i]	 =	0;
				ADC_CONFIG(ADC_1, ADC1.Voie[i].Channel,ADC1.Voie[i].T_Sampling,ADC1.Voie[i].Gain, ADC1.Voie[i].Voltage_Reference);				// Config ADC Parameters
				Voie[i]	+=	ADC_Get_Channel_Value(2000);
				break;
			}
			
			case Thermocouple_K:
			{
				Voie[i]	 =	0;
				//Voie[i]	+=	ADC_Get_Channel_Value(ADC1.Voie[i].Channel, ADC1.Voie[i].T_Sampling, ADC1.Voie[i].Gain, ADC1.Voie[i].Voltage_Reference, 0);
				//Voie[i]	+=	ADC_Get_Channel_Value(ADC1.Voie[i].Channel, ADC1.Voie[i].T_Sampling, ADC1.Voie[i].Gain, ADC1.Voie[i].Voltage_Reference, 0);
				break;
			}	*/
			
			case Autres_Tension_Pile:
			{
				Voie[i]	= BAT_Voltage;	
				break;
			}
			
			case Autres_Temperature_Int:
			{
				Voie[i]	= AMB_Temp_ADC;
				break;
			}
			
		/*	case DEBUG_REF:
			{
				Voie[i]	 =	0;
				//Voie[i]	=	ADC_Get_Channel_Value(0x0C,ADC1.Voie[0].T_Sampling,4, 0x01, 0);
				Voie[i]	+=	TCNT2;
				break;
			}
			
			case DEBUG_ALIM:
			{
				Voie[i]	 =	0;
				//Voie[i]	=	ADC_Get_Channel_Value(0x0D,ADC1.Voie[0].T_Sampling,4, 0x00, 0);
				break;
			}*/

		}
	}
	ChipSelect(None);									//	CS_ADS1118 = 1	Chip Disable
	
	#ifdef OPTION_BIGMEMORY
	if(ACQ_Rate < F_256Hz)
	{
		//Eteindre l'alim de la mémoire.
	}
	#endif
	
	//VDD_ANA_OFF;
		
		
	

	for (int i = 0; i < Nb_Voies; i++)					// Stockage des résultats dans le tableau DATA			
	{
		DATA[i*2]	=	Voie[i]  & 0x000000FF;
		DATA[i*2+1] =	(Voie[i] >> 8) & 0x000000FF;
	}
	ERROR &= 0b11110111;
}

/********************* Température Interne de l'ADC *********************/

/*unsigned int Get_Internal_Temp ()	// Acquisition de la température interne à l'ADC
{
	AMB_Temp_AGE = 0;
	return ADC_Get_Channel_Value(255,6,0,0,0);
}
*/



/************************************************************************/
/*                            Check_Clock	                            */
/************************************************************************/

// Comparaison d'une date avec la date actuelle
// Renvoi 1 si la date actuelle a dépassé la date en entrée

unsigned char Check_Clock(unsigned char *TIME){	unsigned char Temp;
	Temp = 0x00;
	if (RTC_T3.secondes >= (((TIME[1] & 0xF0) >> 4) * 10) + (TIME[1] & 0x0F))
	{
		if (RTC_T3.minutes >= (((TIME[2] & 0xF0) >> 4) * 10) + (TIME[2] & 0x0F))
		{
			if(RTC_T3.heures >= (((TIME[3] & 0xF0) >> 4) * 10) + (TIME[3] & 0x0F))
			{
				if (RTC_T3.jours >= (((TIME[4] & 0x30) >> 4) * 10) + (TIME[4] & 0x0F))
				{
					if (RTC_T3.mois >= (((TIME[5] & 0x10) >> 4) * 10) + (TIME[5] & 0x0F))
					//if (RTC_T3.mois >= (TIME[5] & 0x0F))
					{
						if (RTC_T3.annees >= TIME[6])
						{
							Temp = 0x01;
						}
					}
				}
			}
		}
	}
	return Temp;
}



/************************************************************************/
/*                                SLEEP	                                */
/************************************************************************/

// void POWER_OFF()
// {
// 	ADCSRA	=	0 << ADEN;			// 0 = Disables the ADC
// 	SPCR	=	0 << SPE;			// SPI0 Enable. This bit must be set to enable any SPI operations
// 	PORTC	&=	~(1 << PORTC2);		// VDD_ANA	= 0
// 	PORTD	&=	~(1 << PORTD5) ;	// VDD_MEM	= 0
// 	PORTD	&=	~(1 << PORTD6);		// CS_MEM1	= 0
// 	PORTB	&=	~(1 << PORTB0);		// CS_MEM2	= 0
// 	PORTB	&=	~(1 << PORTB2);		// CS_ADC	= 0
// 	PORTB	&=	~(1 << PORTB3);		// MOSI		= 0
// 	PORTB	&=	~(1 << PORTB4);		// MISO		= 0
// 	PORTB	&=	~(1 << PORTB5);		// SCK		= 0
// 	
// 	ChipSelect(Clear);					// Tous les CS à 0
// 	
// 	//PRR		|=	1 << PRADC;
// 	//PRR		|=	1 << PRSPI;
// 	PRR		|=	1 << PRTWI;
// 	//PRR		|=  1 << PRTIM0;
// 	//PRR		|=	1 << PRTIM1;
// 	
// 	//SMCR	=	SLEEP_MODE_PWR_SAVE;
// 	//SMCR	=	1 << SE;
// 	
// 	PORTD	&=	~(1 << PORTD3);		// AUTO_ALIM= 0
// 	
// }

/************************************************************************/
/*                       SEND EEPROM TO USART	                        */
/************************************************************************/

void SEND_EEPROM_TO_USART(const unsigned long MEMORY_ADDRESS, const unsigned char BIT_COUNT)
{
	unsigned char Memory_chip;
	
	ChipSelect(None);					// Tous les CS à 1
	ALIM_MEM_ON();
		
	SPI_MASTER_INIT(Mode_0,SPI_CLK_Prescale);
	
	unsigned long Buffer_Long;						// Vairaible temporaire de type Long

	Buffer_Long = MEMORY_ADDRESS;

	
	Memory_chip = Memory_Select(Buffer_Long);
	
	
	ChipSelect(Memory_chip);					// Selectionne la Mémoire 1
	
	SPI_TRANCEIVER_CHAR(EEPROM_CMD_READ);		// Instruction de Lecture
	#ifdef OPTION_BIGMEMORY
			if (Buffer_Long > 0x00FFFF)	{ SPI_TRANCEIVER_CHAR((MEMORY_ADDRESS >> 16));}
	#endif		
	SPI_TRANCEIVER_CHAR((unsigned char)(MEMORY_ADDRESS >> 8));
	SPI_TRANCEIVER_CHAR((unsigned char)(MEMORY_ADDRESS));
	
	
	unsigned char buf;
	
	for (int i = 0; i < BIT_COUNT; i++)				
	{
		buf = SPI_TRANCEIVER_CHAR(0x00);			// Lecture des données
		USART_Transmit(buf);						// Envoie des données
	}
	ALIM_MEM_OFF();
}

/* si une interuption est tiré dans la boucle For, et que la mémoire est utilisé, à la reprise ca ne va plus fonctionner.*/


unsigned char Memory_Select(unsigned long Adress)
{
	unsigned char Memory;
	Memory = Memory_1;
	
	#ifdef OPTION_NONE
		if (Adress > 0x37FFF)					// Vérifie la mémoire sur laquelle écrire
		{
			Memory = Memory_4;
		}
		
		else if (Adress > 0x27FFF)					// Vérifie la mémoire sur laquelle écrire
		{
			Memory = Memory_3;
		}
		
		else if (Adress > 0x17FFF)					// Vérifie la mémoire sur laquelle écrire
		{
			Memory = Memory_2;
		}
			
	#else
		if (Adress > 0x00FFFF)					// Vérifie la mémoire sur laquelle écrire
		{
			Memory = Memory_2;
		}
	#endif
	
	return Memory;
}



/************************************************************************/
/*                        OSCCAL Live Update 	                        */
/************************************************************************/

void OSCCAL_Live_Update()
{	
/*	Ticks_Count = TCNT1L;
	Ticks_Count = (TCNT1H << 8) + Ticks_Count;
	TCCR1B	= (0<<CS12) | (0<<CS11) | (0<<CS10);	// Stop Counter
	TCNT1H	= 0x00;
	TCNT1L	= 0x00;
	TCCR1B	= (1<<CS12) | (0<<CS11) | (1<<CS10);	// Set Prescalling 101 = /1024
	
	if (Ticks_Count > (1816 + 10))
	{
		OSCCAL--;                                    // If count is more than count value corresponding to the given frequency:
		asm ("nop");
	}                                                // - decrease speed
	if (Ticks_Count < (1816 - 10))
	{
		OSCCAL++;
		asm ("nop");
	}*/
}


/************************************************************************/
/*                               CALCULS                                */
/************************************************************************/

/***************** Renvoie le nombre de '1' dans un char ****************/
unsigned char Conv_Char_to_nb_Bits(const unsigned char Value)
{
	unsigned char sum = 0;
	unsigned char temp = Value;

	while( temp != 0 ) {
		sum += ( temp & 1L );
		temp >>= 1;
	}
	return sum;
}
/***************** Char Tab to Int ****************/

// Permet de concaténer 2 char d'un tableau dans un Int.
// l'index du premier char est donnée en argument

unsigned int Tab_to_Int(unsigned char *Tab, unsigned char Start)
{
	return ((((unsigned int)Tab[Start] << 8) & 0xFF00) + (unsigned int)Tab[Start+1]);
}
/***************** Int to Char Tab ****************/

// Permet de diviser un int en 2 char et de les stocker dans un tableau
// l'index du premier char est donnée en argument

void Int_to_Tab(unsigned int int_val, unsigned char *Tab, unsigned char Start)
{
	Tab[Start] =	((int_val) & 0xFF00) >> 8;
	Tab[Start+1] =	((int_val) & 0x00FF);
}


/***************** Char Tab to Long ****************/

// Permet de concaténer 4 char d'un tableau dans un Long.
// l'index du premier char est donnée en argument

unsigned long Tab_to_Long(unsigned char *Tab, unsigned char Start)
{
	return ((((long)Tab[Start] << 24) & 0xFF000000) + (((long)Tab[Start+1] << 16) & 0x00FF0000) + (((long)Tab[Start+2] << 8) & 0x0000FF00) + (long)Tab[Start+3]);
}

/***************** Long to Char Tab ****************/

// Permet de diviser un long en 4 char et de les stocker dans un tableau
// l'index du premier char est donnée en argument

void Long_to_Tab(unsigned long long_val, unsigned char *Tab, unsigned char Start)
{
	Tab[Start]	 =  ((long_val)	& 0xFF000000)	>> 24;
	Tab[Start+1] =	((long_val)	& 0x00FF0000)	>> 16;
	Tab[Start+2] =	((long_val) & 0x0000FF00)	>> 8;
	Tab[Start+3] =	((long_val)	& 0x000000FF);
}



void Add_Long_To_Tab(unsigned long Long, unsigned char *Tab, unsigned char Start)
{
	Long += Tab_to_Long(Tab, Start);
	Long_to_Tab(Long, Tab, Start);
}


void Start_Log_Procedure()
{
	/************************************************************************/
	/*                    Vérification démarrage différé                    */
	/************************************************************************/
	
	if(DevBoard.statut == Waiting_StartTime)					
	{
		unsigned char Buffer;
		Buffer = Check_Clock(RX_START_TIME);	// Comparaison entre l'heure actuelle et l'heure de départ
		
		if (Buffer == 0x01)
		{
			Logger_LOAD_CADENCE(0);										// Chargement de la première cadence
			if ((DevBoard.Threshold_Status & 1) == 0)	// Si le démarrage sur seuil de température est désactivé
			{
				DevBoard.statut = Logging;			// Démarage des enregistrement
			}
			else								// Sinon
			{
				DevBoard.statut = Waiting_Threshold;			// Démarrage de la vérification de seuil
			}
			RTC_ACQ_secondes = 0;
		}
	}
	
	
	/************************************************************************/
	/*                   Vérification démarrage sur seuil                   */
	/************************************************************************/	
	
	if(DevBoard.statut == Waiting_Threshold)	
	{
		if ((RTC_ACQ_secondes % CURRENT_CADENCE.ACQ_RATE) == 0)	// Vérification de la cadence
		{
			unsigned char DATA[2];								// Définition du tableau des valeurs lues par l'ADC.
				
			RTC_ACQ_secondes = 0;

			ALIM_ADC_ON();	_delay_ms(WarmUpTime);						// Power on the ADC
			
			Get_ACQUISITIONS(DATA,1);							// Acquisition de la voie du Threshold

			unsigned int buffer;
			buffer = ((DATA[1] << 8) & 0xFF00) + DATA[0];

			if (buffer >= DevBoard.Threshold_Start)
			{
				DevBoard.statut = Logging;
				CURRENT_CADENCE.Id++;
				Logger_LOAD_CADENCE(CURRENT_CADENCE.Id);										// Update des paramètres de la cadence suivante	
				Logger_UPDATE_START_DATE(CURRENT_RECORD.Id);		
			}
			ALIM_MEM_OFF();	
		}		
	}

	/************************************************************************/
	/*                      Vérification de la cadence                      */
	/************************************************************************/	
	
	if (DevBoard.statut == Logging)
	{
		if (CURRENT_CADENCE.Nombre_ACQ_DONE < CURRENT_CADENCE.Nombre_ACQ_TO_DO)	// Enregistrement d'une cadence
		{
			if (((RTC_ACQ_secondes % CURRENT_CADENCE.ACQ_RATE) == 0))				// Vérification de la cadence // (CURRENT_CADENCE.Period !=0) || 
			{
				unsigned char DATA[nb_Voies_MAX*2];								// Définition du tableau des valeurs lues par l'ADC.
				
				RTC_ACQ_secondes = 0;
				
				ALIM_MEM_ON();													// Power on the ADC
							
				Get_ACQUISITIONS(DATA,ADC1.Nb_Voies_Mem);						// Acquisition des voies de mesures ADC
				
				CURRENT_CADENCE.Nombre_ACQ_DONE++;												// Mis à jour du nombre d'acquisition effectuées
				
				if((CURRENT_CADENCE.Nombre_ACQ_DONE % 32) == 0)
				{
					Update_Nombre_ACQ_Done = 1;
				}
								
				unsigned char Nb_Bytes_ACQ =  ADC1.Nb_Voies_Mem*2;
								
				for(int i=0; i < Nb_Bytes_ACQ; i++)							// Buffer the Acquisition data in a new TAB
				{
					BUFFER_ACQ_EEPROM[i_ACQ_Bytes_buffered]	= DATA[i];
					i_ACQ_Bytes_buffered++;
				}
								
				//CURRENT_RECORD.ADDR_DATA = CURRENT_RECORD.ADDR_DATA + (Nb_Bytes_ACQ);	// Update the current DATA pointer
				Write_In_Progress = 0;
				
				if((i_ACQ_Bytes_buffered>= Size_Buffer_ACQ_EEPROM) || ((CURRENT_RECORD.ADDR_DATA % EEPROM_ST_M95512_PAGE_SIZE) + Nb_Bytes_ACQ + i_ACQ_Bytes_buffered) > EEPROM_ST_M95512_PAGE_SIZE)
				{
					MEMORY_EXT_WRITE((CURRENT_RECORD.ADDR_DATA), BUFFER_ACQ_EEPROM, i_ACQ_Bytes_buffered);
					CURRENT_RECORD.ADDR_DATA += i_ACQ_Bytes_buffered;
					i_ACQ_Bytes_buffered = 0;
				}
				
				else if(Flag_Min_Pile == 1)	
				{
					SAVE_Min_Pile();
					Flag_Min_Pile = 0;
				}
					
				
				else if(Update_Nombre_ACQ_Done == 1)
				{
					Logger_SET_NOMBRE_ACQ_DONE(CURRENT_RECORD.Id, CURRENT_CADENCE.Id,CURRENT_CADENCE.Nombre_ACQ_DONE);
					Update_Nombre_ACQ_Done = 0;
				}
				
				VDD_ANA_OFF;
			
				if(Write_In_Progress != 1)
				{
					ALIM_MEM_OFF();	
					_delay_us(5);
				}


				/***** STOP sur Seuil *****/
				if (((DevBoard.Threshold_Status >> 1) & 1) == 1)							// vérification de l'activation de l'option Stop sur Seuil		
				{
					unsigned int buffer;
					buffer = ((DATA[1] << 8) & 0xFF00) + DATA[0];							// Mise en ordre des Octets

					if (buffer <= DevBoard.Threshold_Stop)									// Détection du franchissement du seuil
					{
						Logger_Stop();														// Arret du Logger	
					}
				}
			}
		}
		else																					// Si toutes les acquisitions ont été faites
		{
			cli();
			ACQ_Rate = F_1Hz;
			
			ALIM_MEM_ON(); 
			//_delay_ms(1);
			
			if (Logger_GET_NBR_ACQ_TO_DO(CURRENT_CADENCE.Id + 1) != 0)							// Vérification de la programmation de la cadence suivante;
			{
				Logger_SET_NOMBRE_ACQ_DONE(CURRENT_RECORD.Id, CURRENT_CADENCE.Id, CURRENT_CADENCE.Nombre_ACQ_DONE);	// Log du nombre d'acquisition effectuées
				Stats_Save(CURRENT_CADENCE.Nombre_ACQ_DONE);			// Sauvegarde des données statistiques
				CURRENT_CADENCE.Id++;
				Logger_LOAD_CADENCE(CURRENT_CADENCE.Id);										// Update des paramètres de la cadence suivante
			}
			
			else																				//Si pas d'autre cadence
			{
				Logger_Stop();																	// Arret du Logger
				_delay_ms(10);																		
				_delay_ms(100);
				_delay_ms(500);																	// Il lui faut bien tout cà !!!	
			}
			//ALIM_MEM_OFF();
		}
	}
	sei();		
}