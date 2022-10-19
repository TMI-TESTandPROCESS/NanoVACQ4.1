/*
 * Driver_QLEVER_T3.h
 *
 * Created: 19/05/2017 20:53:33
 *  Author: jbs
 */ 


#ifndef DRIVER_QLEVER_T3_H_
#define DRIVER_QLEVER_T3_H_

#define UART_TEMPO				10				// Temporisation après la commande d'envoie de byte sur UART

#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
#define PREFIXE_ADDR			0xAA
#define PREFIXE_ADDR_WAKE_UP	0xD5

#define ENABLE_UART		UCSR0B |= 1<<RXEN0;	
#define	DISABLE_UART	UCSR0B &= ~(1<<RXEN0);					// Desable the UART Receiver 
#define CLEAR_PCI2		PCIFR |= (1 << PCIF2);					// Clear the Pin Change Interrupt Flag 2
#define ENABLE_PCI2		PCICR |= (1 << PCIE2);					// Enable the Pin Change Interrupt 2
#define DISABLE_PCI2	PCICR &= ~(1 << PCIE2);					// Enable the Pin Change Interrupt 2



unsigned char	USART_cks;
unsigned char	USART_Buffer_Tab[UART_Buffer_Size];
unsigned long	USART_EEPROM_ADDR;

unsigned char	RX_START_TIME[8];
unsigned long	EEPROM_ADDR_TEMP;
unsigned char	USART_EEPROM_NB_VALEURS;

unsigned char	RX_i;
unsigned char	UART_CMD;

volatile unsigned int tempo;					// Variable servant de WatchDog

volatile unsigned char Flag_UART_in_use;
volatile unsigned char UART_Ready_to_Send;


typedef enum
{
	RX_IDLE,
	RX_ADDR,
	RX_COMMAND,
	RX_READ_EEPROM,
	RX_WRITE_EEPROM,
	RX_START,
	RX_BOOT,
	RX_CKS

} RX_STATES;

RX_STATES RXState;


void USART_RECEIVE_T3(const unsigned char UART_BUFFER);
void USART_SEND_T3 (const unsigned char USART_cmd);
void UART_EXIT();
//void USART_SEND_CKS();

#endif /* DRIVER_QLEVER_T3_H_ */