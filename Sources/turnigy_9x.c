/*
 * turnigy_9x.c
 *
 *  Created on: Jan 29, 2016
 *      Author: Norbert
 */
#include "init.h"

int32_t ch1=0,ch2=0,ch3=0,ch4=0; 
int32_t ch1_temp,ch2_temp,ch3_temp,ch4_temp;
int32_t ch1_offset=0,ch2_offset=0,ch3_offset=0,ch4_offset=0;

/*----PPM----------------------------
 * 
 * 
 * I need to identify the where is the PPM signal(see PPM wiki,etc..)
 * The timer period must be same as the PPM period is
 * 1. synchronization(needed only one time): The timer synchronizes with the PPM rising edge, this is the default value of ppm (zero value)
 * 2. measure where the ppm is, how distant is the falling edge from the synchronization value.
 * 
 *      |	    :	      |  			:	|
 *		|       : 		  |				:	|
 *______|_______:_________|_____________:___|_________
 *    Timer   ~40%      Timer		  80%	Timer
 * 	 period	   PPM		period		  PPM	period
 * 
 * 
 * 
 * 
*/

void turnigy_9x()							
{												//The timer peri
	if((ch1_offset != 1) || (ch2_offset != 1) || (ch3_offset != 1) || (ch4_offset != 1))
	{
		basepower=ch3/20;
		setpoint_x = ch1/1000;
		setpoint_y = -ch2/1000;
	}
}

void init_turnigy_timer(){
	SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
	PIT_MCR = 0;
	PIT_MCR = PIT_MCR_FRZ_MASK;		// Enable PIT module clock with debug freeze	
	PIT_LDVAL1 = PERIPHERAL_BUS_CLOCK / PIT1_OVERFLOW_FREQUENCY;	// Calculate and Load timer reset value
//	PIT_TCTRL1 = PIT_TCTRL_TIE_MASK;	// Enable timer interrupt
	PIT_TCTRL1 |= PIT_TCTRL_TEN_MASK;	// Enable timer
}

void calibrate_offset()
{
	int i;
	int32_t ch1_tmp=0,ch2_tmp=0,ch3_tmp=0,ch4_tmp=0;
	Delay_mS(1000);
	for(i=0;i<TURNIGY_CALIBRATE_NUMBER;i++)
	{
		Delay_mS(50);
		ch1_tmp+=ch1;
		ch2_tmp+=ch2;
		ch3_tmp+=ch3;
		ch4_tmp+=ch4;
	}
	if((ch1_tmp!=0) && (ch2_tmp!=0) && (ch3_tmp!=0) && (ch4_tmp!=0))
	{
		ch1_offset=ch1_tmp/TURNIGY_CALIBRATE_NUMBER;
		ch2_offset=ch2_tmp/TURNIGY_CALIBRATE_NUMBER;
		ch3_offset=ch3_tmp/TURNIGY_CALIBRATE_NUMBER;
		ch4_offset=ch4_tmp/TURNIGY_CALIBRATE_NUMBER;
	}
	else{
		ch1_offset=1;
		ch2_offset=1;
		ch3_offset=1;
		ch4_offset=1;				
	}	
}

void init_turnigy9x()
{
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
	ch1_pin = PORT_PCR_MUX(1) | PORT_PCR_IRQC(0b1010);
	ch2_pin	= PORT_PCR_MUX(1) | PORT_PCR_IRQC(0b1010);
	ch3_pin	= PORT_PCR_MUX(1) | PORT_PCR_IRQC(0b1010);
	ch4_pin	= PORT_PCR_MUX(1) | PORT_PCR_IRQC(0b1010);
	
	calibrate_offset();
/*	
	GPIOA_PDDR |= 1<<14;
	GPIOA_PDDR |= 1<<15;
	GPIOA_PDDR |= 1<<16;
	GPIOA_PDDR |= 1<<17;*/
}
