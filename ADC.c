/*
 * ADC.c
 *
 * Created: 28/05/2017 10:22:33
 *  Author: jbs
 */ 

#include "hardware.h"
#include "AD7124.h"
#include <avr/interrupt.h>


unsigned int ADC_GET_INT_T3()
{
	
	ADC_ESB = 0;
	ADC_MSB = 0;

	#if (ADC_EXT == 7124)
		AD7124_GET_ACQ_LONG();
	#elif (ADC_EXT == 1118)
	#endif
	
	while (ADC_ACQ_DONE == 0){}			//Boucle tant que l'ADC n'a pas fini l'acquisition et n'a pas déclanché l'intéruption
	ADC_ACQ_DONE = 0;
	
	return ((ADC_ESB << 8) + (ADC_MSB));
}


unsigned long ADC_READ_DATA ()
{
	return AD7124_READ_LONG (AD7124_DATA_REG);
}

