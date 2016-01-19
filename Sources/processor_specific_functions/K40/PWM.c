/*
 * PWM.c
 *
 *  Created on: Dec 19, 2014
 *      Author: Norbi
 */
#include "init.h"

void init_PWM()
{
	SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;	//Enable the Clock to the FTM0 Module
	FTM0_MODE |= FTM_MODE_WPDIS_MASK;
	// FTMEN is bit 0, need to set to zero so DECAPEN can be set to 0
	FTM0_MODE &= ~FTM_MODE_FTMEN_MASK; 
	// Set Edge Aligned PWM
	FTM0_QDCTRL &= ~FTM_QDCTRL_QUADEN_MASK;  
	// QUADEN is Bit 1, Set Quadrature Decoder Mode (QUADEN) Enable to 0,   (disabled)
	// Also need to setup the FTM0C1SC channel control register 
	FTM0_CNT = 0;	// FTM Counter Value - reset counter to zero
    FTM0_MOD = FTM0_CLOCK/(1<<FTM0_CLK_PRESCALE)/PWM_FREQUNECY;  // Count value of full duty cycle
    //TMPM0_MOD erteket ha eleri a szamlalo(TPM0_CNT) akkor tortenik a megszakitas(PWM egy periodusa) a szamlalo csak 16bites
	FTM0_CNTIN = 0;// Set the Counter Initial Value to 0 
}
void init_Motor_PWM()
{

	init_PWM();
	//Setup Channels
	FTM0_C1SC |= FTM_CnSC_ELSB_MASK | FTM_CnSC_MSB_MASK; //Edge or level select
	FTM0_C1SC &= ~FTM_CnSC_ELSA_MASK; //Edge or level Select
	
	FTM0_C6SC |= FTM_CnSC_ELSB_MASK | FTM_CnSC_MSB_MASK; //Edge or level select
	FTM0_C6SC &= ~FTM_CnSC_ELSA_MASK; //Edge or level Select
	
	FTM0_C3SC |= FTM_CnSC_ELSB_MASK | FTM_CnSC_MSB_MASK; //Edge or level select
	FTM0_C3SC &= ~FTM_CnSC_ELSA_MASK; //Edge or level Select
	
	FTM0_C7SC |= FTM_CnSC_ELSB_MASK | FTM_CnSC_MSB_MASK; //Edge or level select
	FTM0_C7SC &= ~FTM_CnSC_ELSA_MASK; //Edge or level Select

	// Edit registers when no clock is fed to timer so the MOD value, gets pushed in immediately
	FTM0_SC = 0; // Make sure its Off!

	// FTMx_CnV contains the captured FTM counter value, this value determines the pulse width
	FTM0_C1V = 0;
	FTM0_C3V = 0;
	FTM0_C6V = 0;
	FTM0_C7V = 0;

	//Status and Control bits 
	FTM0_SC =  FTM_SC_PS(4) | FTM_SC_CLKS(1); // Selects Clock source to be "system clock" or (01)
	
    PORTD_PCR0 = PORT_PCR_MUX(4);		//M1 - A	TPM0_CH0
    PORTA_PCR5  = PORT_PCR_MUX(3);     	//M2 - B	TPM0_CH2	
    PORTC_PCR9 = PORT_PCR_MUX(3);		//M3 - C	TPM0_CH5
    PORTD_PCR4 = PORT_PCR_MUX(4);		//M4 - D	TPM0_CH4
        
}
