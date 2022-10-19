/*
 * ADS1120.h
 *
 * Created: 20/11/2017 16:52:34
 *  Author: jbs
 */ 


#ifndef ADS1120_H_
#define ADS1120_H_



#define ADS_1120_RESET				0X06
#define ADS_1120_START				0X08
#define ADS_1120_POWERDOWN			0X02
#define ADS_1120_READ_DATA			0X10
#define ADS_1120_READ_REG			0X20
#define ADS_1120_WRITE_REG			0X40

/************** REGISTER 0 **************/
#define ADS_1120_PGA_ON				0
#define ADS_1120_PGA_OFF			1

/************** REGISTER 1 **************/
#define ADS_1120_NORMAL_MODE		0x00
#define ADS_1120_DUTY_CYCLE_MODE	0x01
#define ADS_1120_TURBO_MODE			0x10

#define ADS_1120_SINGLE_SHOT_MODE	0
#define ADS_1120_CONTINUOUS_MODE	1

#define ADS_1120_BURN_OUT_OFF		0
#define ADS_1120_BURN_OUT_ON		1

#define ADS_1120_INTERNAL_T_OFF		0
#define ADS_1120_INTERNAL_T_ON		1

/************** REGISTER 2 **************/

#define ADS_1120_REF_INTERNAL		0	// Voltage Reference Selection
#define ADS_1120_REF_0				1
#define ADS_1120_REF_1				2
#define ADS_1120_REF_ANALOG			3

#define ADS_1120_FILTER_OFF			0	// FIR Filter Configuration
#define ADS_1120_FILTER_50_60_ON	1
#define ADS_1120_FILTER_50_ON		2
#define ADS_1120_FILTER_60_ON		3

#define ADS_1120_PSW_OPEN			0
#define ADS_1120_PSW_AUTO			1

#define ADS_1120_IDAC_OFF			0
#define ADS_1120_IDAC_50u			2
#define ADS_1120_IDAC_100u			3
#define ADS_1120_IDAC_250u			4
#define ADS_1120_IDAC_500u			5
#define ADS_1120_IDAC_1000u			6
#define ADS_1120_IDAC_1500u			7

/************** REGISTER 3 **************/
#define ADS_1120_IDAC1_OFF			0
#define ADS_1120_IDAC1_TO_A0		1
#define ADS_1120_IDAC1_TO_A1		2
#define ADS_1120_IDAC1_TO_A2		3
#define ADS_1120_IDAC1_TO_A3		4
#define ADS_1120_IDAC1_TO_RP0		5
#define ADS_1120_IDAC1_TO_RN0		6

#define ADS_1120_IDAC2_OFF			0
#define ADS_1120_IDAC2_TO_A0		1
#define ADS_1120_IDAC2_TO_A1		2
#define ADS_1120_IDAC2_TO_A2		3
#define ADS_1120_IDAC2_TO_A3		4
#define ADS_1120_IDAC2_TO_RP0		5
#define ADS_1120_IDAC2_TO_RN0		6

#define ADS_1120_DRDY_ONLY			0
#define ADS_1120_DRDY_ON_DOUT		1


#define ADS_1120_PullUp_Dis	0			// Pullup resistor disabled on DOUT/DRDY pin
#define ADS_1120_PullUp_Ena	1			// Pullup resistor enabled on DOUT/DRDY pin (default)		

#define ADS_1120_NOP		1			// Valid data, update the Config register (default)

void ADC_INIT(void);		
unsigned int ADC_Get_Channel_Value(unsigned int Offset);


unsigned int Get_Internal_Temp ();

void ADC_SET_MICRO_TO_SLEEP(unsigned char DRDY_2);
unsigned int ADC_Get_FAST_Channel_Value(unsigned int Offset);
void ADC_Change_Settings(unsigned char Channel, unsigned char PGA);
unsigned long ADC_Get_Double_ADC_Channel_Value(unsigned int Offset);
void ADC_CONFIG(unsigned char ADC_Id, unsigned char Channel, unsigned char SPS, unsigned char PGA, unsigned char REF);
void ADC_TEST(unsigned char ADC_Id, unsigned char REG_CHECK_VALUE);
unsigned int ADC_Add_Offset (unsigned int Value, unsigned int Offset);


#endif /* ADS1120_H_ */