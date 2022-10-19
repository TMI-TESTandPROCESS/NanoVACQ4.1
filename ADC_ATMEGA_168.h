/*
 * ADC_ATMEGA_168.h
 *
 * Created: 19/01/2018 10:17:46
 *  Author: jbs
 */ 


#ifndef ADC_ATMEGA_168_H_
#define ADC_ATMEGA_168_H_


void ADC_ATMEGA_168_INIT (unsigned char PRESCALE);
unsigned int ADC_ATMEGA_168_Get_ACQ (unsigned char Channel);
unsigned int ADC_ATMEGA_168_CONV_VOLTAGE (unsigned int ADC_BAT_VOLTAGE);
void ADC_ATMEGA_168_SWITCH_OFF();

#endif /* ADC_ATMEGA_168_H_ */