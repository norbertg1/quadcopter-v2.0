/*
 * PWM.c
 *
 *  Created on: Dec 19, 2014
 *      Author: Norbi
 */
#include "init.h"

void init_PWM()
{
	SIM_SCGC6 |= SIM_SCGC6_FTM1_MASK;
	SIM_SCGC3 |= SIM_SCGC3_FTM2_MASK;
	//Enable the Clock to the FTM0 Module
	FTM1_MODE |= FTM_MODE_WPDIS_MASK;
	FTM2_MODE |= FTM_MODE_WPDIS_MASK;
	// FTMEN is bit 0, need to set to zero so DECAPEN can be set to 0
	FTM1_MODE &= ~FTM_MODE_FTMEN_MASK; 
	FTM2_MODE &= ~FTM_MODE_FTMEN_MASK; 
	// Set Edge Aligned PWM
	FTM1_QDCTRL &= ~FTM_QDCTRL_QUADEN_MASK;  
	FTM2_QDCTRL &= ~FTM_QDCTRL_QUADEN_MASK;  
	// QUADEN is Bit 1, Set Quadrature Decoder Mode (QUADEN) Enable to 0,   (disabled)
	// Also need to setup the FTM0C1SC channel control register 
	FTM1_CNT = 0;
	FTM2_CNT = 0;	
	// FTM Counter Value - reset counter to zero
	FTM1_MOD = FTM12_CLOCK/(1<<FTM12_CLK_PRESCALE)/PWM_FREQUNECY;
	FTM2_MOD = FTM12_CLOCK/(1<<FTM12_CLK_PRESCALE)/PWM_FREQUNECY;  
	// Count value of full duty cycle
	//TMPM0_MOD erteket ha eleri a szamlalo(TPM0_CNT) akkor tortenik a megszakitas(PWM egy periodusa) a szamlalo csak 16bites
	FTM1_CNTIN = 0;
	FTM2_CNTIN = 0;
	// Set the Counter Initial Value to 0 
}
void init_Motor_PWM()
{

	init_PWM();
	//Setup Channels
	FTM1_C0SC |= FTM_CnSC_ELSB_MASK | FTM_CnSC_MSB_MASK; //Edge or level select
	FTM1_C0SC &= ~FTM_CnSC_ELSA_MASK; //Edge or level Select
	
	FTM1_C1SC |= FTM_CnSC_ELSB_MASK | FTM_CnSC_MSB_MASK; //Edge or level select
	FTM1_C1SC &= ~FTM_CnSC_ELSA_MASK; //Edge or level Select
	
	FTM2_C0SC |= FTM_CnSC_ELSB_MASK | FTM_CnSC_MSB_MASK; //Edge or level select
	FTM2_C0SC &= ~FTM_CnSC_ELSA_MASK; //Edge or level Select
	
	FTM2_C1SC |= FTM_CnSC_ELSB_MASK | FTM_CnSC_MSB_MASK; //Edge or level select
	FTM2_C1SC &= ~FTM_CnSC_ELSA_MASK; //Edge or level Select

	// Edit registers when no clock is fed to timer so the MOD value, gets pushed in immediately
	FTM1_SC = 0;
	FTM2_SC = 0;
	// Make sure its Off!

	// FTMx_CnV contains the captured FTM counter value, this value determines the pulse width
	FTM1_C0V = 0;
	FTM1_C1V = 0;
	FTM2_C0V = 0;
	FTM2_C1V = 0;

	//Status and Control bits 
	FTM1_SC =  FTM_SC_PS(4) | FTM_SC_CLKS(1);
	FTM2_SC =  FTM_SC_PS(4) | FTM_SC_CLKS(1);
	// Selects Clock source to be "system clock" or (01)
		
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
	
//  PORTA_PCR8	= PORT_PCR_MUX(3);		//M1 - A	FTM1_CH0
    PORTA_PCR12	= PORT_PCR_MUX(3);		//M1 - A	FTM1_CH0
    PORTA_PCR9  = PORT_PCR_MUX(3);     	//M2 - B	FTM1_CH1	
    PORTA_PCR10 = PORT_PCR_MUX(3);		//M3 - C	FTM2_CH0
    PORTA_PCR11 = PORT_PCR_MUX(3);		//M4 - D	FTM2_CH1*/
        
}
