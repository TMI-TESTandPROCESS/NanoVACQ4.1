/*
 * Fonctions_Logger.h
 *
 * Created: 07/06/2017 15:25:40
 *  Author: jbs
 */ 


#ifndef FONCTIONS_LOGGER_H_
#define FONCTIONS_LOGGER_H_


void(*boot)( void );

/************************************************************************/
/*                              BOARD INIT                              */
/************************************************************************/

void BOARD_INIT();

void Load_Params(void);
void Load_Options(void);

/************************************************************************/
/*                            START / STOP                              */
/************************************************************************/

void Logger_Start(unsigned char Record_Number);
void Logger_Stop();




/************************************************************************/
/*                          GESTION CADENCES                            */
/************************************************************************/

void Logger_LOAD_CADENCE (unsigned char Cadence_Number);
unsigned long Logger_GET_NBR_ACQ_TO_DO(unsigned char Cadence_Number);
unsigned char Logger_GET_CADENCE_SECONDES(unsigned char Cadence_Number);
unsigned char Logger_GET_CADENCE_MINUTES(unsigned char Cadence_Number);
unsigned int Logger_CALC_ACQ_RATE (unsigned char Minutes, unsigned char Secondes);
unsigned long Logger_GET_NOMBRE_ACQ_DONE(unsigned char Record_Number, unsigned char Cadence_Number);
void          Logger_SET_NOMBRE_ACQ_DONE(unsigned char Record_Number, unsigned char Cadence_Number, unsigned long NB_ACQ_DONE);



/************************************************************************/
/*                           GESTION RECORDS                            */
/************************************************************************/

void Logger_LOAD_RECORD (unsigned char Record_Number);

unsigned long Logger_GET_ADDR_DEBUT_DATA(unsigned char Record_Number);
void          Logger_SET_ADDR_DEBUT_DATA(unsigned char Record_Number, unsigned long Adddr_Debut_Data);
void Logger_UPDATE_CURRENT_RECORD();
void Logger_SAVE_START_DATE (unsigned char *START_TIME, unsigned char Record_Number);


/************************************************************************/
/*                             ACQUISITIONS                             */
/************************************************************************/


unsigned int ADC_ATMEGA_168_CONV_VOLTAGE (unsigned int ADC_BAT_VOLTAGE);
void Start_Log_Procedure();

/************************************************************************/
/*                               PORT I/O                               */
/************************************************************************/

void ADC_CS(unsigned char value);
void MEM_CS(unsigned char value);

void Check_Pile();


void SAVE_Min_Pile();

void POWER_OFF();


void RTC_Adjust(unsigned int Temperature);


unsigned int Get_Pile();
void Get_ACQUISITIONS(unsigned char *DATA, unsigned char Nb_Voies);

void SEND_EEPROM_TO_USART(const unsigned long MEMORY_ADDRESS, const unsigned char BIT_COUNT);
unsigned char Memory_Select(unsigned long Adress);

unsigned char Check_Clock(unsigned char *TIME);

void Add_Long_To_Tab(unsigned long Long, unsigned char *Tab, unsigned char Start);
void Stats_Init ();
void VDD_ANA(unsigned char state);

void Update_Extrem_Temp(unsigned char *DATA);
//void Stats_LOG(unsigned int Temperature, unsigned int ACQ_Rate);

void Logger_Log_Battery_Usage(unsigned int ACQ_Rate, unsigned long NB_ACQ_DONE);
void Logger_Check_Min_Pile(unsigned int valeur_Pile);

void Stats_Log(unsigned int Temperature);
void Stats_Save(unsigned long NB_ACQ_DONE);

void Logger_Write_Error();

/********** Calculs ************/

unsigned char Conv_Char_to_nb_Bits(const unsigned char Value);

unsigned int Tab_to_Int(unsigned char *Tab, unsigned char Start);
void Int_to_Tab(unsigned int int_val, unsigned char *Tab, unsigned char Start);

unsigned long Tab_to_Long(unsigned char *Tab, unsigned char Start);
void Long_to_Tab(unsigned long long_val, unsigned char *Tab, unsigned char Start);


#endif /* FONCTIONS_LOGGER_H_ */