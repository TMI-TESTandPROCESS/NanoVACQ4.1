/*
 * ALIM.h
 *
 * Created: 16/05/2017 14:22:48
 *  Author: jbs
 */ 


#ifndef ALIM_H_
#define ALIM_H_


#define SPI_PWR_ON		PRR		&=	~(1 << PRSPI);		// Power SPI ON
#define SPI_PWR_OFF		PRR		|=	(1 << PRSPI);		// Power SPI OFF 

#define ADC_INT_PWR_ON	PRR		&=	~(1 << PRADC);		// Alimente l'ADC Interne
#define ADC_INT_PWR_OFF	PRR		|=	(1 << PRADC);		// Eteint l' ADC Interne

#define TIMER0_PWR_ON	PRR		&=  ~(1 << PRTIM0);		// Allume le Timer 0
#define TIMER0_PWR_OFF	PRR		|=  (1 << PRTIM0);		// Eteint le Timer 0

#define TIMER1_PWR_ON	PRR		&=  ~(1 << PRTIM1);		// Allume le Timer 1
#define TIMER1_PWR_OFF	PRR		|=  (1 << PRTIM1);		// Eteint le Timer 1

#define TIMER2_PWR_ON	PRR		&=  ~(1 << PRTIM2);		// Allume le Timer 2
#define TIMER2_PWR_OFF	PRR		|=  (1 << PRTIM2);		// Eteint le Timer 2


#define SPI_ENABLE		SPCR	|=  (1<<SPE);			// Active le SPI
#define SPI_DISABLE		SPCR	&=  ~(1<<SPE);			// Désactive le SPI

#define SPI_MOSI_0		PORTB	&=	~(1 << PORTB3);		// MOSI = 0
#define SPI_MISO_0		PORTB	&=	~(1 << PORTB4);		// MISO = 0
#define SPI_SCK_0		PORTB	&=	~(1 << PORTB5);		// SCK  = 0


#define VDD_MICRO_ON	PORTD	|= (1 << PORTD4);		// VDD_MICRO = 1	PORT.D4
#define VDD_MICRO_OFF	PORTD	&= ~(1 << PORTD4);		// VDD_MICRO = 0

#define VDD_NUM_ON		PORTC	|= (1 << PORTC3);		// VDD_MICRO = 1	PORT.C3
#define VDD_NUM_OFF		PORTC	&= ~(1 << PORTC3);		// VDD_MICRO = 0

#define VDD_ANA_ON		PORTB	|=	 (1<<PORTB1);		// VDD_ANA = 1		PORT.B1
#define VDD_ANA_OFF		PORTB	&=  ~(1<<PORTB1);		// VDD_ANA = 0

#define VDD_AUX_ON		PORTB	|=	 (1<<PORTB0);		// VDD_AUX = 1		PORT.B0
#define VDD_AUX_OFF		PORTB	&=  ~(1<<PORTB0);		// VDD_AUX = 0

void ALIM_MEM_ON()	;		
void ALIM_MEM_OFF()	;
void ALIM_ADC_ON();
void ALIM_ADC_OFF();

#endif /* ALIM_H_ */