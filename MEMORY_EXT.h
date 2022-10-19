/*
 * MEMORY.h
 *
 * Created: 11/06/2017 10:27:25
 *  Author: jbs
 */ 


#ifndef MEMORY_EXT_H_
#define MEMORY_EXT_H_

void MEMORY_EXT_WRITE(const unsigned long MEMORY_ADDRESS, unsigned char *DATA, const unsigned char BIT_COUNT);
void MEMORY_EXT_READ (const unsigned long MEMORY_ADDRESS, unsigned char *DATA, const unsigned char BIT_COUNT);


#endif /* MEMORY_EXT_H_ */