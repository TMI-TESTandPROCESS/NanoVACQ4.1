//************************************************************************/
/*************************** DESCRIPTION ********************************/
/************************************************************************/
/*																		*/
/*	Name: Hardware.h													*/
/*	Version: 1.0														*/
/*                                                                      */
/*	Description de toutes les variables partagées (globales) ainsi que	*/
/*  les informations sur le hardware dans lequel sera utilisé le code,	*/
/*	comme par exemple, le nom de l'ADC, la taille des mémoires et leurs	*/ 
/*	nombres, la fréquence d'horloge,etc...                              */
/*                                                                      */
/************************* History Revisions ****************************/
/*																		*/
/* 1.0 - 04/02/2017 - Creation by J.BARBARAS 							*/
/*																		*/
/*																		*/
/************************************************************************/
 

#include "RTC.h"
#include "EEPROM_ST_M95.h"
#include <avr/io.h>


#ifndef HARDWARE_H_
#define HARDWARE_H_

//#define BOARD_PICOVACQ_2017_ALPHA
#define PICOVACQ_2CH_BOARD
//#define OPTION_BIGMEMORY
//#define OPTION_2PT
#define OPTION_NONE


#define GOTO_BOOT	asm("jmp 0x3800;");
#define REBOOT		asm("jmp 0x0000;");





/************************************************************************/
/*								 BOARD                                  */
/************************************************************************/


#ifdef BOARD_DEVBOARD
	#define ATMEGA88
	#define F_CPU						1843200UL
	#define USART_BAUDRATE				115200
	#define BOOTLOADER_START_ADDRESS	0xC00
	#define AD7124
	#define LCT2943_TEMP_INT
	#define LCT2943_BAT_VOLTAGE
	#define Number_of_Memories			1
	#define M1_24XX1025
#endif

#ifdef BOARD_PICOVACQ_2017_ALPHA
	#define ATMEGA168
	#define F_CPU						1843200UL
	#define USART_BAUDRATE				115200
	#define BOOTLOADER_START_ADDRESS	0x1C00
	#define ADS1120
	#define ADC_ATMEGA_168_TEMP
	#define ADC_ATMEGA_168_BAT_VOLTAGE
	#define Number_of_Memories			2
	#define M2_M95512
	#define nb_Voies_MAX				5
	#define TOTAL_EEPROM_SIZE			0x020000
	#define ADDR_DEBUT_DATA				0x82E4
#endif

#ifdef PICOVACQ_2CH_BOARD
	#define ATMEGA168
	#define F_CPU						1843200UL
	#define USART_BAUDRATE				115200
	#define BOOTLOADER_START_ADDRESS	0x1C00
	#define ADS1120
	#define ADC_ATMEGA_168_TEMP
	#define ADC_ATMEGA_168_BAT_VOLTAGE
	#define Number_of_Memories			4
	#define M4_M95512
	#define nb_Voies_MAX				5
	#ifdef OPTION_BIGMEMORY
		#define TOTAL_EEPROM_SIZE			0x7FFFFF
		#define ADDR_DEBUT_DATA				0x10000
	#else
		#define TOTAL_EEPROM_SIZE			0x040000
		#define ADDR_DEBUT_DATA				0x82E4
	#endif
	#define Timer2_Prescale				128
	
#endif

#ifdef PICOVACQ_2_2CH_BOARDS
#define ATMEGA168
#define F_CPU						1843200UL
#define USART_BAUDRATE				115200
#define BOOTLOADER_START_ADDRESS	0x1C00
#define ADS1120
#define ADC_ATMEGA_168_TEMP
#define ADC_ATMEGA_168_BAT_VOLTAGE
#define Number_of_Memories			4
#define M4_M95512
#define nb_Voies_MAX				5
#define TOTAL_EEPROM_SIZE			0x040000
#define ADDR_DEBUT_DATA				0x82E4
#define Timer2_Prescale				128

#endif




#define SPI_CLK_Prescale				4			// SPI clock = F_CPU/Prescale	(4/16/64/128)
#define WarmUpTime						10			// Délai en ms
extern unsigned char const Firmware[];


/************************************************************************/
/*                              LOGGER                                  */
/************************************************************************/
struct LOGGER
{
	unsigned char	Serial_Number[8];
	volatile unsigned char statut;			// 0x55 Start - 0x51 Wait time - 0x52 Wait threshold
	volatile unsigned int	minimum_pile_atteint;
	unsigned int	Seuil_Low_Bat;
	unsigned long	duree_fonctionnement;	// 136 ans pour une résolution à la seconde !!	
	unsigned char	Mem_Tournante;
	unsigned char	Voies_Internes;		// T mem/ T lue // Vbat mem / Vbat lue // 0 non / 1 oui
	
	unsigned char	Threshold_Channel;
	unsigned int	Threshold_Start;
	unsigned int	Threshold_Stop;	
	unsigned char	Threshold_Status;
	

	
	unsigned char	CADENCE_ID;
	unsigned char	RECORD_ID;
	
	unsigned char	Adjust_key[4];
	unsigned char	Etal_Key[4];
};

struct LOGGER DevBoard;



struct CADENCE
{
	unsigned char Id;
	unsigned long Nombre_ACQ_TO_DO;
	unsigned char secondes;
	unsigned char minutes;
	unsigned int ACQ_RATE;
	unsigned long Nombre_ACQ_DONE;
	unsigned char Period;
	
};

struct RECORD
{
	unsigned char Id;
	unsigned char Statut;
	struct DATE START_DATE;
	unsigned long DEBUT_DATA;
	unsigned long ADDR_DATA;
	
};

volatile struct RECORD CURRENT_RECORD;
volatile struct CADENCE CURRENT_CADENCE;	




#define Waiting_StartTime		0x51	// Wait time
#define Waiting_Threshold		0x53	// Wait threshold
#define Logging					0x55	// Start

#define Size_Buffer_ACQ_EEPROM	32		// 32 Bytes


/************************************************************************/
/*                        MAPPING MEMORIES                              */
/************************************************************************/

unsigned long EEPROM_ADDR;
unsigned char Time_to_Write;



#define T3_ADDR_EEPROM_START				0x8000	

#define T3_ADDR_Nb_Voie_lues				0x800D
#define T3_ADDR_Nb_Voie_mem					0x800E
#define T3_ADDR_Error_Byte					0x800F
#define T3_ADDR_Params_Voies				0x8011
#define T3_ADDR_Mem_Turn					0x8026

#define T3_ADDR_Threshold					0x80A4

#define T3_ADDR_Key_Adjust					0x80A0	

#define T3_ADDR_CADENCE_Params				0x8100
#define T3_ADDR_CADENCE_SECONDES			0x8103
#define T3_ADDR_CADENCE_MINUTES				0x8104
#define T3_ADDR_CADENCE_NOMBRE_ACQ_DONE		0x812E
#define T3_ADDR_RECORD_DEBUT_DATA			0x812B
#define T3_ADDR_RECORD_START_DATE			0x8123

#define T3_ADDR_Minimum_Pile_atteint		0x8009
#define T3_ADDR_Seuil_Low_Bat				0x800B
#define T3_ADDR_Temp_Min_et_Max_atteinte	0x8054
#define T3_ADDR_Battery_and_Logger_Life		0x805C

#define	T3_ADDR_Logger_Stats				0x8030	


/************************************************************************/
/*					   Paramètres des voie à mesurer			        */
/************************************************************************/

struct sVoie
{
	unsigned char Id;
	unsigned char Channel;
	unsigned char Type;
	unsigned char Gain;
	unsigned char T_Sampling;
	unsigned char Nb_Sampling;
	unsigned char Voltage_Reference;
	unsigned long Polynome[3];
};

#define Dummy							0x01

#define Temperature_PT1000				0x11
#define Temperature_PT100				0x12

#define Pression_15bars					0x21
#define Pression_30bars					0x22

#define Double_Temperature				0x31
#define Double_Pressure_05bars			0x32

#define Thermocouple_J					0x41
#define Thermocouple_K					0x42
#define Thermocouple_T					0x43

#define Autres_Tension_Pile				0x81
#define Autres_Temperature_Int			0x82
#define Autres_Humidite					0x85

#define DEBUG_REF						0x90
#define DEBUG_ALIM						0x91
#define DEBUG_1							0x92
#define Temperature_PT1000_2			0x93
#define FAST_Temperature_Pressure		0x94
#define FAST_Measure					0x95



/************************************************************************/
/*                               USART                                  */
/************************************************************************/

#define UART_Buffer_Size		128
unsigned char UART_DOUBLE_SPEED;


/************************************************************************/
/*                            ADC Interne                               */
/************************************************************************/

volatile unsigned char	ADC_ATMEGA168_ACQ_DONE;
volatile unsigned int	ADC_ATMEGA168_Temp;

#define Internal_Temp_Offset	20000

unsigned int BAT_Voltage;


/************************************************************************/
/*                               ADC                                   */
/************************************************************************/

struct sADC
{
	unsigned char Id;
	unsigned char Nb_Voies_Lues;
	unsigned char Nb_Voies_Mem;
	struct sVoie Voie[5];
};

volatile struct sADC ADC1;

volatile unsigned char ADC_1_ACQ_DONE;
volatile unsigned char ADC_2_ACQ_DONE;
unsigned int AMB_Temp_ADC;
unsigned int Time_to_ADC;
unsigned char BUFFER_ACQ_EEPROM[260];
unsigned char i_ACQ_Bytes_buffered;


/************************************************************************/
/*                               RTC                                   */
/************************************************************************/

volatile unsigned int RTC_ACQ_secondes;
//volatile unsigned int Ticks_Count;
extern unsigned char Tab_Mois[12];
volatile unsigned char Logger_Flag_ACQ;
volatile unsigned char Delay_Done;

#define RTC_DERIVE_THRESHOLD 7812
unsigned char T_milliSec;


/************************************************************************/
/*                        Statistiques                            */
/************************************************************************/

struct Logger_Stats
{
	unsigned long Temp_Range[9];
	
	unsigned int Temperature_Min;
	unsigned int Temperature_Max;
	unsigned int Temperature_Min_Cycle;
	unsigned int Temperature_Max_Cycle;
};
volatile struct Logger_Stats Stats;


#define ADS1118_Temperature_min		0x2000		// min°C
#define ADS1118_Temperature_m60		0x3880		// -60°C
#define ADS1118_Temperature_m30		0x3C40		// -30°C
#define ADS1118_Temperature_0		0x0000		//   0°C
#define ADS1118_Temperature_p30		0x03C0		//  30°C
#define ADS1118_Temperature_p60		0x0780		//  60°C
#define ADS1118_Temperature_p90		0x0B40		//  90°C
#define ADS1118_Temperature_p120	0x0F00		// 120°C
#define ADS1118_Temperature_p140	0x1180		// 140°C

unsigned long NB_SECONDES_DONE;
unsigned char Start_Log_Flag;					// Pour mémoriser le nombre de Start effectués par le Logger


/************************************************************************/
/*                        Variables de tests                            */
/************************************************************************/


volatile unsigned char ERROR;

volatile unsigned char Flag_Comm_Connected;

unsigned char char1;
unsigned char char2;

unsigned char Update_Nombre_ACQ_Done;

unsigned char ACQ_Sampling;

//unsigned char AMB_Temp_AGE;
volatile unsigned char Flag_Min_Pile;

unsigned char t_TCNT2;

unsigned char MEM_FULL;
unsigned char ACQ_Rate;

#define USED			1
#define UNUSED			0

#define DRDY_2_USED		1
#define DRDY_2_UNUSED	0

#define F_1Hz		0 
#define F_4Hz		1
#define F_8Hz		2
#define F_16Hz		3
#define F_32Hz		4
#define F_64Hz		5
#define F_128Hz		6
#define F_256Hz		7

unsigned char Write_In_Progress;

#endif /* HARDWARE_H_ */


