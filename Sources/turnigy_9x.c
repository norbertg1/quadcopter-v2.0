/*
 * turnigy_9x.c
 *
 *  Created on: Jan 29, 2016
 *      Author: Norbert
 */
#include "init.h"

uint8_t ch0,ch1,ch2,ch3; 

void synchronize_9x()
{
	while(!ch1_pulse)	{}
	PIT_TCTRL1 &= (~PIT_TCTRL_TIE_MASK & ~PIT_TCTRL_TEN_MASK);	//Disable timer, interrupt
	PIT_CVAL1=0;
	PIT_TCTRL1 |= (PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK);	//Enable timer, interrupt
}

void init_turnigy_timer(){
	SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;
	//Enable the Clock to the FTM0 Module
	FTM0_MODE |= FTM_MODE_WPDIS_MASK;
	// FTMEN is bit 0, need to set to zero so DECAPEN can be set to 0
	FTM0_MODE &= ~FTM_MODE_FTMEN_MASK; 
	// Set Edge Aligned PWM
	FTM0_CNT = 0;
	// FTM Counter Value - reset counter to zero
	FTM0_MOD = FTM_CLOCK/(1<<FTM_CLK_PRESCALE)/TURNIGY_FREQUENCY;
	// Count value of full duty cycle
	//TMPM0_MOD erteket ha eleri a szamlalo(TPM0_CNT) akkor tortenik a megszakitas(PWM egy periodusa) a szamlalo csak 16bites
	FTM0_CNTIN = 0;
	// Set the Counter Initial Value to 0 
}

void init_turnigy9x()
{
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
	ch0_pin = PORT_PCR_MUX(1) | PORT_PCR_IRQC(6);
	ch1_pin	= PORT_PCR_MUX(1) | PORT_PCR_IRQC(6);
	ch2_pin	= PORT_PCR_MUX(1) | PORT_PCR_IRQC(6);
	ch3_pin	= PORT_PCR_MUX(1) | PORT_PCR_IRQC(6);
	
	GPIOA_PDDR = 1<<14;
	GPIOA_PDDR = 1<<15;
	GPIOA_PDDR = 1<<16;
	GPIOA_PDDR = 1<<17;
}

void capture_ppm(){
	if(ch0_pulse){
		ch0 = (FTM0_MOD/FTM0_CNT)*100;
		ch0_pin |= PORT_PCR_ISF_MASK;
	}
	if(ch1_pulse)	{ch1 = (FTM0_MOD/FTM0_CNT)*100; ch1_pin |= PORT_PCR_ISF_MASK;}
	if(ch2_pulse)	{ch2 = (FTM0_MOD/FTM0_CNT)*100;	ch2_pin |= PORT_PCR_ISF_MASK;}
	if(ch3_pulse)	{ch3 = (FTM0_MOD/FTM0_CNT)*100;	ch3_pin |= PORT_PCR_ISF_MASK;}
}
