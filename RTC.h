/*
 * RTC.h
 *
 * Created: 11/05/2017 17:12:11
 *  Author: jbs
 */ 


#ifndef RTC_H_
#define RTC_H_

#define START_TIMER0 TCCR0B		= (1<<CS02) | (0<<CS01) | (1<<CS00);	// Set Prescalling 101 = /1024 = 1,8KHz
#define STOP_TIMER0  TCCR0B		= 0;
#define CLEAR_TIMER0 TCNT0		= 0;


struct DATE
{
	unsigned char dixiemes;
	unsigned char secondes;
	unsigned char minutes;
	unsigned char heures;
	unsigned char jours;
	unsigned char mois;
	unsigned char annees;
};

volatile struct DATE RTC_T3;


void Clock_Update();
void Clock_Mise_a_l_heure( unsigned char *TIME);void OSCCAL_COUNT();
void RTC_SET_TIMER();unsigned char not_a_leap_year();

int CumulDeriv;
int Deriv;
float indice_1;
float indice_2;
float indice_3;

/************************************************************************/
/*               Calibration de l'oscillateur interne                   */
/************************************************************************/

void CalibrationRoutine(void);
void CalibrationInit(void);
void CalibrateInternalRc(void);
unsigned int Counter(void);
void BinarySearch(unsigned int ct);
void NeighborSearch(void);
void OSCCAL_COUNT();





#endif /* RTC_H_ */