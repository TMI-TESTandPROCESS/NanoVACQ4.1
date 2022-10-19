/*
 * CALIB_Internal_RC.h
 *
 * Created: 15/01/2019 14:20:13
 *  Author: jbs
 */ 


#ifndef CALIB_INTERNAL_RC_H_
#define CALIB_INTERNAL_RC_H_

/************************************************************************/
/*                 Calibration of the internal RC                       */
/************************************************************************/

//#define CALIBRATION_METHOD_BINARY_WITHOUT_NEIGHBOR

unsigned char neighborsSearched;		// Holds the number of neighbors searched
unsigned char calStep;					// The binary search step size
unsigned char bestCountDiff;			// The lowest difference between desired and measured counter value
unsigned char bestCountDiff_first;		// Stores the lowest difference between desired and measured counter value for the first search
unsigned char bestOSCCAL;				// The OSCCAL value corresponding to the bestCountDiff
unsigned char bestOSCCAL_first;			// Stores the OSCCAL value corresponding to the bestCountDiff for the first search
unsigned int countVal;					// The desired counter value
unsigned int calibration;				// Calibration status
signed char sign;						// Stores the direction of the binary step (-1 or 1)


#if defined(__AVR_ATmega88__) | defined(__AVR_ATmega88P__) | defined(__AVR_ATmega88PA__) | defined (__AVR_ATmega88PB__) | \
defined(__AVR_ATmega168__) | defined(__AVR_ATmega168A__) | defined(__AVR_ATmega168P__) | defined (__AVR_ATmega168PA__) | defined (__AVR_ATmega168PB__)
#define ASYNC_TIMER                        AS2		// Asynchronous Timer/Counter2. '1': Counter2 is clocked from a crystal Oscillator connected to the TOSC1 pin.
#define NO_PRESCALING                      CS20		// CS20 = No prescaling
#define ASYNC_TIMER_CONTROL_REGISTER       TCCR2B	// TC2 Control Register B
#define ASYNC_TIMER_CONTROL_UPDATE_BUSY    TCR2AUB	// Timer/Counter Control Register2 Update Busy
#define OUTPUT_COMPARE_UPDATE_BUSY         OCR2AUB	// Enable External Clock Input
#define TIMER_UPDATE_BUSY                  TCN2UB	// Timer/Counter2 Update Busy
#define TIMER                              TCNT2	// TC2 8 bit Counter Value Register
#define OSCCAL_RESOLUTION                  7
#define LOOP_CYCLES                        7
//#define TWO_RANGES								// Pour faire la recherche sur la partie haute et basse de OSCCAL
//#define CALIBRATION_METHOD_SIMPLE

#else
#error "CPU non définie pour Compensation RC"
#endif


#define CALIBRATION_FREQUENCY 1843200		// Modify CALIBRATION_FREQUENCY to desired calibration frequency
//#define CALIBRATION_FREQUENCY 3686400		// Modify CALIBRATION_FREQUENCY to desired calibration frequency

#define XTAL_FREQUENCY 32768				// Frequency of the external oscillator. A 32kHz crystal is recommended
#define EXTERNAL_TICKS 100					// Ticks on XTAL. Modify to increase/decrease accuracy. Entre 40 et 255

#define FALSE		0						// Fixed calibration values and macros
#define TRUE		1						// These values are fixed and used by all calibration methods. Not to be modified.
#define RUNNING		0						//
#define FINISHED	1						//
#define DEFAULT_OSCCAL_MASK        0x00		// Lower half and
#define DEFAULT_OSCCAL_MASK_HIGH   0x80		// upper half for devices with splitted OSCCAL register

#define DEFAULT_OSCCAL_HIGH ((1 << (OSCCAL_RESOLUTION - 1)) | DEFAULT_OSCCAL_MASK_HIGH)
#define INITIAL_STEP         (1 << (OSCCAL_RESOLUTION - 2))
#define DEFAULT_OSCCAL      ((1 << (OSCCAL_RESOLUTION - 1)) | DEFAULT_OSCCAL_MASK)

// Initialisation du pas de recherche
#define PREPARE_CALIBRATION()\
calStep = INITIAL_STEP;\
calibration = RUNNING;

// Set up Count Value
#define COMPUTE_COUNT_VALUE()\
countVal = ((EXTERNAL_TICKS*CALIBRATION_FREQUENCY)/(XTAL_FREQUENCY*LOOP_CYCLES));

// Set up timer to be ASYNCHRONOUS from the CPU clock with a second EXTERNAL 32,768kHz CRYSTAL driving it.	//No prescaling on asynchronous timer.
#define SETUP_ASYNC_TIMER()\
ASSR |= (1<<ASYNC_TIMER);\
ASYNC_TIMER_CONTROL_REGISTER = (1<<NO_PRESCALING);

// Calcul la valeur absolue
#define ABS(var) (((var) < 0) ? -(var) : (var));

// NOP
#define NOP() asm ("nop");
// For ATmega64 and ATmega128, 8 nop instructions must be run

unsigned int count;
unsigned char OSCCAL_VALUE;

#endif /* CALIB_INTERNAL_RC_H_ */