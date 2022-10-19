/************************************************************************/
/*************************** DESCRIPTION ********************************/
/************************************************************************/
/*																		*/
/*	Name: DRIVER_QLEVER_T3												*/
/*	Version: 1.0														*/
/*																		*/
/*	Les fonctions de ce fichier permettent d'initailiser L'UART de		*/
/*	l'ATMEGA 88 ainsi que d'envoyer et recevoir 1 octet					*/
/*                                                                      */

/*                                                                      */
/************************************************************************/
/************************** History Revisions ***************************/
/*																		*/
/* 1.0 - 19/05/2017 - Creation by J.BARBARASS 							*/
/*																		*/
/*																		*/
/************************************************************************/



#include "Hardware.h"

#include <avr/io.h>
#include <avr/pgmspace.h>

#include "DRIVER_UART.h"
#include "Driver_QLEVER_T3.h"
#include "DRIVER_SPI.h"

#include "ALIM.h"
#include "ADS1120.h"
#include "Fonctions_Logger.h"
#include "MEMORY_EXT.h"

#include <avr/interrupt.h>
#include <util/delay.h>



	
/************************************************************************/
/*                           UART RECEIVE                               */
/************************************************************************/ 

// Fonction d'analyse de la trame reçu, en temps réel, octet par octet

void USART_RECEIVE_T3(const unsigned char UART_BUFFER) 
{
	PORTD	&= ~(1 << PORTD6);
	switch (RXState)											// Machine d'état de décodage de la trame
    {
		
		/********** Vérification du premier OCTET **********/
		case RX_IDLE:							
        {
			USART_cks		=		0;							// Initialisation du Cheksum
			RX_i			=		0;							// Initialisation de RX_i
			UART_CMD		=		0;							// Initialisation de la commande recu
					
			if		(UART_BUFFER == PREFIXE_ADDR)				// Vérification de l'octet de Start
			{
				PORTD	|= (1 << PORTD6);
				RXState		= RX_ADDR;								
			}
				
			else if (UART_BUFFER == PREFIXE_ADDR_WAKE_UP)		// Lors d'un reveil de Power Save, le premier bit est perdu.
			{
				RXState		= RX_COMMAND;						// Il faut alors sauter l'octet ADDR pour recaler la trame.
				USART_cks	= 0xFF;								// et mettre à jour le checksum comme si l'octet ADDR avait été recu.
			}
				
			else 
			{
				UART_EXIT();									// Sinon RAZ
			}
			break;
		}
		
		/************ Vérification de l'Adresse ************/
		case RX_ADDR:							
		{
			
			if (UART_BUFFER == 0xFF)							// Le 2ème octet (adresse) doit être 0xFF
			{	
				USART_cks  += UART_BUFFER;						// CheckSum update
				RXState		= RX_COMMAND;						
			}
			else
			{
				UART_EXIT();
			}
			break;	
		}
		
		/********** Vérification du Code Commande **********/
		case RX_COMMAND:						
		{
			USART_cks	+= UART_BUFFER;		// CheckSum update
			UART_CMD	 = UART_BUFFER;
			
			if		((UART_CMD == 0x00) || (UART_CMD == 0x01) || (UART_CMD == 0x02) || (UART_CMD == 0x06))
			{
				RXState = RX_CKS;				// Type Outils / Status / Stop / Acquisition
			}
			
			else if	((UART_CMD == 0x03) || (UART_CMD == 0x0B) || (UART_CMD == 0x0E))			
			{									// Lecture EEPROM / BaudRate Change / Flash Erase
				RXState	= RX_READ_EEPROM;	
			}
			else if (UART_CMD == 0x04)			// Écriture EEPROM
			{
				VDD_MICRO_ON;				
				RXState	= RX_WRITE_EEPROM;
			}
			else if (UART_CMD == 0x0A)			// Start
			{	
				RXState = RX_START;
			}
			else if (UART_CMD == 0x09)			// Boot
			{
				RXState = RX_BOOT;
			}
			else 
			{
				UART_EXIT();
			}	
			break;
		}
		
		/********************** BOOT **********************/
		case RX_BOOT:
		{
			USART_cks += UART_BUFFER;					// CheckSum update
			switch (RX_i)								// Vérification de la séquence de Reset
			{
				case 0:
				{
					if (UART_BUFFER == 0x66){	RX_i++; }
					else					{	RXState = RX_IDLE;}
					break;
				}
				case 1:
				{
					if (UART_BUFFER == 0x99){	RX_i++; }
					else					{	RXState = RX_IDLE;}
					break;
				}
				case 2:
				{
					if (UART_BUFFER == 0x55){	RX_i++; }
					else					{	RXState = RX_IDLE;}
					break;
				}
				case 3:
				{
					if (UART_BUFFER == 0xFF){	RX_i++; }
					else					{	RXState = RX_IDLE;}
					break;
				}
				case 4:
				{
					if ((UART_BUFFER == 0x00) )	{	RXState = RX_CKS; }		//|| (UART_BUFFER == 0x01)
					else						{	RXState = RX_IDLE;}
					break;
				}
				default: 
				{
					UART_EXIT();
				}	
			}
			break;
		}
		
		
		
		
		/********************** START **********************/
		case RX_START:
		{		
			USART_cks			+= UART_BUFFER;			// CheckSum update
			RX_START_TIME[RX_i]  = UART_BUFFER;			// Sauvegarde de l'heure de départ
			RX_i++;
					
			if (RX_i > 7)		
			{	
				RXState = RX_CKS;
			}
			break;
		}
		
		/***************** Lecture EEPROM ******************/
		case RX_READ_EEPROM:
		{
			USART_cks			+= UART_BUFFER;			// CheckSum update
			
			if (RX_i < 3)								// Bufferisation de l'adresse à lire
			{
				USART_EEPROM_ADDR  = USART_EEPROM_ADDR << 8;
				USART_EEPROM_ADDR |= UART_BUFFER;
			}
			
			else										// Bufferisation du nombre d'octets à lire
			{
				USART_EEPROM_NB_VALEURS = UART_BUFFER;
				RXState = RX_CKS;
			}
			RX_i ++;
			break;
		}
		
		/**************** Ecriture EEPROM ******************/
		case RX_WRITE_EEPROM:
		{
			USART_cks		+= UART_BUFFER;
					
			if		(RX_i < 3)								// Bufferisation de l'adresse à ecrire
			{
				USART_EEPROM_ADDR = USART_EEPROM_ADDR << 8;
				USART_EEPROM_ADDR |= UART_BUFFER;

			}
			
			else if (RX_i == 3)								// Bufferisation du nombre d'octets à ecrire
			{
				USART_EEPROM_NB_VALEURS = UART_BUFFER;
			}

			else											//if ((RX_i > 0x03) && (RX_i < USART_EEPROM_NB_VALEURS + 4))
			{	
				USART_Buffer_Tab[RX_i - 4] = UART_BUFFER;	// Bufferisation des valeurs à écrire
			}
			
			RX_i ++;
			
			if (RX_i >= (USART_EEPROM_NB_VALEURS + 4))		// Fin de la trame d'écriture
			{
 				RXState = RX_CKS;
			}
			
			break;
		}
		
		/************* Vérification du Checksum ************/
		case RX_CKS:							
		{
			USART_cks = (256 - USART_cks)%256;				// Mise en forme du Checksum
		
			if (UART_BUFFER == USART_cks)					// Vérification du Checksum
			{
				UART_Ready_to_Send = 1;						// Flag de réponse du Logger
			
			}

			else											// Si mauvais Checksum
			{
				USART_Transmit(0xee);
				UART_EXIT();								// On sort de la partie traitement UART				
			}
					
			break;
		}
		
	}


}


/************************************************************************/
/*                             UART SEND                                */
/************************************************************************/ 

// Fonction de réponse à la demande de QLEVER

void USART_SEND_T3 (const unsigned char USART_cmd)
{
	cli();

	DISABLE_UART;								// Desable the UART Receiver Module
	DISABLE_PCI2;
	
	tempo				= 0;					// Reset du timer (watchdog) sur l'UART
	USART_cks			= 0;					// Initialisation du Cheksum
	EEPROM_ADDR_TEMP	= 0x00000000;			// Initialisation de la variable temporaire d'adresse mémoire
	
	USART_Transmit(USART_cmd);					// Transmition du code commande
	
	switch (USART_cmd)
	{	
		/******************** Type Outil *******************/
		case 0x00:
		{
			for (int i = 0; i < 12; i++)		// Envoie de la version du microcode
			{
				USART_Transmit(pgm_read_byte(&Firmware[i]));
			}

			USART_Transmit(0x02);				// Nombre total de voies
			
			USART_Transmit(0xFF);				// Cryptage Voie Pile
			
			break;
		}
		
		/********************** Statut *********************/	
		case 0x01:
		{			
			if((DevBoard.statut == Waiting_StartTime ) || (DevBoard.statut == Waiting_Threshold ))
			{
				USART_Transmit(Logging);		// Le logger renvoie 0x55 quelque soit le mode de Start
			}
			else
			{
				USART_Transmit(DevBoard.statut);
			}
			
			//char1 = ((int)TCNT2*100/255);												// Mise en forme des Dixièmes de Secondes

			
			//USART_Transmit(CURRENT_RECORD.ADDR_DATA & 0x00FF);														// Error Byte
			//USART_Transmit(CURRENT_CADENCE.Period);	
			USART_Transmit((char)OCR2A);	
													
			//USART_Transmit((char1 / 0xA * 0x10) + (char1 % 0xA));						// Dixièmes de Secondes
		
			USART_Transmit(0x00);	
			
			USART_Transmit((RTC_T3.secondes / 0xA * 0x10) + (RTC_T3.secondes % 0xA));	// Secondes
			USART_Transmit((RTC_T3.minutes  / 0xA * 0x10) + (RTC_T3.minutes  % 0xA));	// Minutes
			USART_Transmit((RTC_T3.heures   / 0xA * 0x10) + (RTC_T3.heures   % 0xA));	// Heures
			
			USART_Transmit((RTC_T3.jours	/ 0xA * 0x10) + (RTC_T3.jours	% 0xA));	// Jours 
			USART_Transmit((RTC_T3.mois		/ 0xA * 0x10) + (RTC_T3.mois	% 0xA));	// Mois 
	
			USART_Transmit(CURRENT_RECORD.Id);											// N° de Job
			USART_Transmit(CURRENT_CADENCE.Id);											// N° de Cadence
				
			USART_Transmit(CURRENT_RECORD.ADDR_DATA);									// Pointeur Mémoire
			USART_Transmit(CURRENT_RECORD.ADDR_DATA >> 8);
			USART_Transmit(CURRENT_RECORD.ADDR_DATA >> 16);
	
			USART_Transmit( CURRENT_CADENCE.Nombre_ACQ_TO_DO - CURRENT_CADENCE.Nombre_ACQ_DONE);		// Nombre d'acquisitions réstante
			USART_Transmit((CURRENT_CADENCE.Nombre_ACQ_TO_DO - CURRENT_CADENCE.Nombre_ACQ_DONE) >> 8);
			USART_Transmit((CURRENT_CADENCE.Nombre_ACQ_TO_DO - CURRENT_CADENCE.Nombre_ACQ_DONE) >> 16);

			USART_Transmit(CURRENT_CADENCE.secondes);									// Cadence : Secondes
			USART_Transmit(CURRENT_CADENCE.minutes);									// Cadence : Minutes
	
			//USART_Transmit(DevBoard.minimum_pile_atteint);								// Valeur min de la pile atteinte
			//USART_Transmit(DevBoard.minimum_pile_atteint >> 8);		
			
			USART_Transmit(AMB_Temp_ADC);								// Valeur min de la pile atteinte
			USART_Transmit(AMB_Temp_ADC >> 8);
			
			
			USART_Transmit((CURRENT_CADENCE.Nombre_ACQ_TO_DO) & 0x000000FF);			// Nombre total d'acquisition de la cadence
			USART_Transmit((CURRENT_CADENCE.Nombre_ACQ_TO_DO >> 8) & 0x000000FF);
			USART_Transmit((CURRENT_CADENCE.Nombre_ACQ_TO_DO >> 16) & 0x000000FF);
		
			break;
		}
		
		
		/***************** Lecture Mémoire *****************/
		case 0x03:		 
		{
			EEPROM_ADDR_TEMP = (((USART_EEPROM_ADDR & 0x000000FF) << 16) + (USART_EEPROM_ADDR & 0x0000FF00) + ((USART_EEPROM_ADDR & 0x00FF0000) >> 16));
			
			VDD_MICRO_ON;
			#ifdef OPTION_BIGMEMORY
				_delay_ms(1);
			#endif
								
			USART_Transmit(USART_EEPROM_ADDR >> 16);
			USART_Transmit(USART_EEPROM_ADDR >> 8);
			USART_Transmit(USART_EEPROM_ADDR);
			USART_Transmit(USART_EEPROM_NB_VALEURS);
			
			switch (EEPROM_ADDR_TEMP)
			{
			
				case  0:								// Sur le T3 une écriture à l'adresse 00
				{														// Correspond à une mise à l'heure
					USART_Transmit(0x04);					
				
					char1 = ((int)TCNT2*100/(OCR2A+1));												// Mise en forme des Dixièmes de Secondes
					USART_Transmit((char1 / 0xA * 0x10) + (char1 % 0xA));						// Dixièmes de Secondes
					//USART_Transmit(0x00);	
				
					USART_Transmit((RTC_T3.secondes / 0xA * 0x10) + (RTC_T3.secondes % 0xA));	// Secondes
					USART_Transmit((RTC_T3.minutes  / 0xA * 0x10) + (RTC_T3.minutes  % 0xA));	// Minutes
					USART_Transmit((RTC_T3.heures   / 0xA * 0x10) + (RTC_T3.heures   % 0xA));	// Heures
				
					USART_Transmit((RTC_T3.jours	/ 0xA * 0x10) + (RTC_T3.jours	% 0xA));	// Jours
					USART_Transmit((RTC_T3.mois		/ 0xA * 0x10) + (RTC_T3.mois	% 0xA));	// Mois
				
					USART_Transmit(RTC_T3.annees);
				
					break;
				
				}
			
				case T3_ADDR_EEPROM_START:
				{
					if (USART_EEPROM_NB_VALEURS == 8)
					{
						for (int i = 0; i < 8; i++)
						{
							USART_Transmit(DevBoard.Serial_Number[i]);
						}
						break;
					}
				}
			
				case T3_ADDR_Key_Adjust:// && (USART_EEPROM_NB_VALEURS == 4))
				{
					if ((EEPROM_ADDR_TEMP == T3_ADDR_Key_Adjust) && (USART_EEPROM_NB_VALEURS == 4))
					{
						for (int i = 0; i < 4; i++)
						{
							USART_Transmit(DevBoard.Adjust_key[i]);
						}
						break;
					}					
				}
			
				default: 
				{
					
					//if (DevBoard.statut == 0x00)
					
					//SEND_EEPROM_TO_USART(EEPROM_ADDR_TEMP,USART_EEPROM_NB_VALEURS);	// Lit la mémoire et envoie direct sur l'UART
					
				
					//else
					{
						ALIM_MEM_ON();
											
						MEMORY_EXT_READ(EEPROM_ADDR_TEMP, USART_Buffer_Tab, USART_EEPROM_NB_VALEURS);
					
						ALIM_MEM_OFF();
					
				
					
						for (int i = 0; i < USART_EEPROM_NB_VALEURS; i++)
						{
							USART_Transmit(USART_Buffer_Tab[i]);
						}
					}
					break;
				}
			}
		
			break;
		}
		
		/***************** Ecriture Mémoire ****************/		
		case 0x04:
		{	
			VDD_MICRO_ON;
			
			
			USART_Transmit(USART_EEPROM_ADDR >> 16);
			USART_Transmit(USART_EEPROM_ADDR >> 8);
			USART_Transmit(USART_EEPROM_ADDR);
			
			ALIM_MEM_ON();
						
			if (USART_EEPROM_ADDR == 0)								// Sur le T3 une écriture à l'adresse 00
			{														// Correspond à une mise à l'heure
				USART_Transmit(0x08);								// Nombre d'octets renvoyés
			
				Clock_Mise_a_l_heure(USART_Buffer_Tab);				// corresponds à une mise à l'heure
				
				for (int i = 0; i < USART_EEPROM_NB_VALEURS; i++)
				{
					USART_Transmit(USART_Buffer_Tab[i]);			// Acquitement bête
				}
			}
			
			else
			{
				EEPROM_ADDR_TEMP = (((USART_EEPROM_ADDR & 0x000000FF) << 16) + (USART_EEPROM_ADDR & 0x0000FF00) + ((USART_EEPROM_ADDR & 0x00FF0000) >> 16));
							
				MEMORY_EXT_WRITE(EEPROM_ADDR_TEMP, USART_Buffer_Tab, USART_EEPROM_NB_VALEURS);	// Ecrit en EEPROM les valeurs

				USART_Transmit(USART_EEPROM_NB_VALEURS);										// Envoie le nombre d'octet relus			
				
				//SEND_EEPROM_TO_USART(EEPROM_ADDR_TEMP,USART_EEPROM_NB_VALEURS);					// Relit la mémoire et envoie direct sur l'UART		
				
				MEMORY_EXT_READ(EEPROM_ADDR_TEMP, USART_Buffer_Tab, USART_EEPROM_NB_VALEURS);
				
				ALIM_MEM_OFF();
				
				for (int i = 0; i < USART_EEPROM_NB_VALEURS; i++)
				{
					USART_Transmit(USART_Buffer_Tab[i]);
				}
				
				
				
				if (EEPROM_ADDR_TEMP < 0x8100)
				{
					Load_Params();
				}
			}
			
			ALIM_MEM_OFF();
			break;
		}
		
		/************** Acquisition Temps Réel *************/
		
		case 0x06:
		{
			unsigned char DATA[nb_Voies_MAX*2];	
			
			VDD_MICRO_ON;												// Alimente le MicroControlleur par la pile
			_delay_ms(1);

								
			if ((DevBoard.statut & 1) != 1) 
			{
				_delay_ms(10);											// Tempo si pas en start
				
				unsigned int Bat = Get_Pile();							// Tension Pile
				
				ALIM_ADC_ON();											// Power on the ADC
				_delay_ms(10);
				
				Get_ACQUISITIONS(DATA,(ADC1.Nb_Voies_Lues-1));			// -1 pour la voie pile qui est traitée à part
				
				ALIM_ADC_OFF();
				
				for (int i = 0; i < ADC1.Nb_Voies_Lues-1 ; i++)			// Envoie les données lues
				{	
					USART_Transmit(DATA[i*2]);
					USART_Transmit(DATA[i*2+1]);
				}
				
				USART_Transmit(Bat & 0x00FF);							// Envoie la voie Pile
				USART_Transmit(Bat >> 8);
			}
			
			else														// Si en START
			{
				USART_Transmit(AMB_Temp_ADC & 0x00FF);					// Envoie la voie de Temperature Interne
				USART_Transmit(AMB_Temp_ADC >> 8);						// TODO, ne marche pas si plus d'une voie,...voir même avec une voie !!
			}
			
			break;
		}
		
		/********************** START **********************/
		case 0x0A:
		{
			VDD_MICRO_ON;								// Alimente le MicroControlleur par la pile
			_delay_ms(1);
			ACQ_Rate = F_1Hz;
		
			ALIM_MEM_ON();
			//_delay_ms(5);
		
			Load_Options();
				
			Logger_UPDATE_CURRENT_RECORD();
			if (CURRENT_RECORD.Id > 14)					// Vérifie si le nombre max de Job n'est pas dépassé
			{
				USART_Transmit(0xF0);					// Send Error Code for Full Job
				break;									// Sort du Switch case
			}
			Logger_SAVE_START_DATE(RX_START_TIME, CURRENT_RECORD.Id);	// Sauvegarde l'heure de départ programmée
			if (CURRENT_RECORD.Id == 0)
			{
				CURRENT_RECORD.DEBUT_DATA = ADDR_DEBUT_DATA;
			}		
			else
			{
				unsigned char DATA[21];
				EEPROM_ADDR  = ((long)T3_ADDR_RECORD_DEBUT_DATA) + (29 * (CURRENT_RECORD.Id - 1));
				MEMORY_EXT_READ(EEPROM_ADDR, DATA, 21);
				CURRENT_RECORD.DEBUT_DATA = ((((long)DATA[0]) & 0x000000FF) + (((long)DATA[1] << 8) & 0x0000FF00) + (((long)DATA[2] << 16) & 0x00FF0000));
				for (int i=3; i<20; i=i+3)
				{
					CURRENT_RECORD.DEBUT_DATA += ((((((long)DATA[i]) & 0x000000FF) + (((long)DATA[i+1] << 8) & 0x0000FF00) + (((long)DATA[i+2] << 16) & 0x00FF0000))) * 2 * ADC1.Nb_Voies_Mem);
				}
			}
			Logger_SET_ADDR_DEBUT_DATA(CURRENT_RECORD.Id, CURRENT_RECORD.DEBUT_DATA);
			
			
			Logger_Start(CURRENT_RECORD.Id);			// Start the logger
			
			ALIM_MEM_OFF();	
							
			break;
		}
		
		/*********************** STOP **********************/
		case 0x02:
		{
			Logger_Stop();								// Arrete l'enregistrement
			break;
		}
		
		/******************* Chip Erase *******************/
		case 0x0E:
		{
			VDD_MICRO_ON;
			_delay_us(100);
			
			ALIM_MEM_ON();
						
			EEPROM_ST_M95_INIT();	
			//_delay_us(500);
			
			ChipSelect(Memory_2);
			SPI_TRANCEIVER_CHAR(EEPROM_CMD_WREN);			// Write/erase Enable
			ChipSelect(None);		
		
			ChipSelect(Memory_2);							// Flash Select	
			SPI_TRANCEIVER_CHAR(USART_EEPROM_NB_VALEURS);	// Flash Erase Command
			
			if (USART_EEPROM_NB_VALEURS != 0x60)			// If not all Chip Erase
			{
				SPI_TRANCEIVER_CHAR((unsigned char)(USART_EEPROM_ADDR));
				SPI_TRANCEIVER_CHAR((unsigned char)(USART_EEPROM_ADDR >> 8));
				SPI_TRANCEIVER_CHAR((unsigned char)(USART_EEPROM_ADDR >> 16));
			}
			ChipSelect(None);	
			
			ChipSelect(Memory_2);			
			do {															// Check End of Erase
				_delay_ms(5);												// Update every 5ms
				} while (((EEPROM_ST_M95_Get_Status() & 0b00000001) != 0));	// du registre Status
		
			ALIM_MEM_OFF();
			_delay_us(10);
		
			break;
		}
		
		/******************* CHANGE MAIN CLOCK PRESCALE *********************/
		case 0x0B:
		{
			UART_DOUBLE_SPEED = USART_EEPROM_NB_VALEURS;	
			CLKPR	=	(1<<CLKPCE);											// Préparation à la modification du Clock Prescale
			
			if (UART_DOUBLE_SPEED == 0x01)
			{
				CLKPR	=	(0<<CLKPS3)|(0<<CLKPS2)|(0<<CLKPS1)|(1<<CLKPS0);	// Main clock Prescaler = 2
			}		
			else
			{
				CLKPR	=	(0<<CLKPS3)|(0<<CLKPS2)|(1<<CLKPS1)|(0<<CLKPS0);	// Main clock Prescaler = 4
			}	
			break;
		}
				
		
		/******************* BOOTLOADER *******************/
		case 0x09:
		{
			GOTO_BOOT;								// Jump to Boot
			break;
		}		
	}	
	
	USART_Transmit(256 - USART_cks);				// Envoie le Cheksum

	_delay_us(150);									// Tempo indispensable // Ne marche pas à 100µs, laisser à 150µs !!!

}


/************************************************************************/
/*                             UART EXIT                                */
/************************************************************************/

// Fonction permettant de sortir du traitement de l'UART à la suite d’une erreur ou à la fin du traitement
// Réinitialise les variables et réactive les interruptions sur la pin RX


void UART_EXIT()
{	
	RXState					= RX_IDLE;			// Reset UART_Receive function
	tempo					= 0;				// Clear variables
	USART_cks				= 0;
	UART_Ready_to_Send		= 0;
	Flag_UART_in_use		= 0;				// Fin d'utilisation de l'UART
			
	ENABLE_UART;								// Enable the UART Receiver Module
	
	CLEAR_PCI2;									// Clear the Pin Change Interrupt Flag 2
	ENABLE_PCI2;								// Enable the Pin Change Interrupt 2

	if (DevBoard.statut == 0x00)	{ VDD_MICRO_OFF; 	VDD_NUM_OFF;}		// Eteint l'alimentation par la pile si pas en Start
}












	/*MEMORY_EXT_READ(EEPROM_ADDR_TEMP, USART_Buffer_Tab, USART_EEPROM_NB_VALEURS);
				
	ALIM_MEM_OFF();
	sei();
				
	for (int i = 0; i < USART_EEPROM_NB_VALEURS; i++)
	{
		USART_Transmit(USART_Buffer_Tab[i]);
	}
	*/
	
				
				
	//USART_Transmit(AMB_Temp_ADC);			// DEBUG
	//USART_Transmit(AMB_Temp_ADC >> 8);

	//USART_Transmit(CURRENT_CADENCE.ACQ_RATE & 0x000000FF);			// DEBUG
	//USART_Transmit((CURRENT_CADENCE.ACQ_RATE >> 8)  & 0x000000FF);

	//USART_Transmit(CumulDeriv & 0x000000FF);			// DEBUG
	//USART_Transmit((CumulDeriv >> 8)  & 0x000000FF);
	//USART_Transmit((CumulDeriv >> 16) & 0x000000FF);
	//USART_Transmit((CumulDeriv >> 24) & 0x000000FF);
				
	//USART_Transmit(RTC_ACQ_secondes & 0x000000FF);			// DEBUG
	//USART_Transmit((RTC_ACQ_secondes >> 8)  & 0x000000FF);
				