/*
 * IncFile1.h
 *
 * Created: 14/11/2017 10:27:53
 *  Author: jbs
 */ 


#include "Hardware.h"

/************************************************************************/
/*							     INFOS					                */
/************************************************************************/

/******************** BOARD *******************/
#ifdef BOARD_DEVBOARD
	#pragma message  "************ BOARD: DevBOARD ************"
#elif defined BOARD_PICOVACQ_2017_ALPHA
	#pragma message "********* BOARD: PicoVACQ Alpha **********"
#elif defined PICOVACQ_2CH_BOARD
	#pragma message "********* BOARD: PicoVACQ 2CH **********"
#else
	#error "Board non definie"
#endif

/******************** OPTIONS *******************/
#ifdef OPTION_BIGMEMORY
#pragma message  "********** OPTION: BigMemory ************"
#else
#pragma message  "************* OPTION: None **************"
#endif



/******************* Micro *********************/
#ifdef ATMEGA88
#pragma message "Micro : \t\t\tATmega 88"
#elif defined ATMEGA168
#pragma message "Micro : \t\t\tATmega 168"
#else
#error No CPU defined !
#endif

/****************** Frequency ******************/
#if (F_CPU == 1843200UL)
	#pragma message "Frequence CPU : \t1,84MHz"
#elif (F_CPU == 1000000UL)
	#pragma message "Frequence CPU : \t1 MHz"
#else
	#pragma "Frequence CPU non definie"
#endif


/***************** BAUD Rate *******************/
#if (USART_BAUDRATE == 9600)
	#pragma message "Baudrate : \t\t9600 Bd"
#elif (USART_BAUDRATE == 115200)
	#pragma message "Baudrate : \t\t115200 Bd"
#elif (USART_BAUDRATE == 62500)
	#pragma message "Baudrate : \t\t115200 Bd"
#else
	#error "Baudrate non defini"
#endif

/********* Bootloader Start Address ************/
#if (BOOTLOADER_START_ADDRESS == 0x0C00)
	#pragma message "Adresse Bootloader : \t0x1800"
#elif  (BOOTLOADER_START_ADDRESS == 0x1C00)
	#pragma message "Adresse Bootloader : \t0x3800"
#else
	#error "Adresse de Boot non definie"
#endif


/******************** ADC **********************/
#ifdef AD7124
	#pragma message "ADC : \t\t\tAD7124"
#elif defined ADS1120
	#pragma message "ADC : \t\t\tADS1120"
#elif defined ADS1118
	#pragma message "ADC : \t\t\tADS1118"
#else
	#error "ADC non defini"
#endif


/***************** Memories ********************/
#ifdef			M1_24XX1025
#pragma message "Memoire : \t\tMicrochip I2C 24XX1025"
#elif defined	M1_25XX1024
#pragma message "Memoire : \t\tMicrochip SPI 25XX1024"
#elif defined	M1_M95512
#pragma message "Memoires : \t\tST M95512"
#elif defined	M2_M95512
#pragma message "Memoires : \t\t2x ST M95512"
#elif defined	M4_M95512
#pragma message "Memoires : \t\t4x ST M95512"
#else
#error No Memory defined !
#endif


#define VALUE_TO_STRING(x) #x
#define VALUE(x) VALUE_TO_STRING(x)
#define VAR_NAME_VALUE(var) #var ":\t\t"  VALUE(var)



/*************** Début DATA *******************/

#pragma message(VAR_NAME_VALUE(TOTAL_EEPROM_SIZE))
#pragma message(VAR_NAME_VALUE(ADDR_DEBUT_DATA))




