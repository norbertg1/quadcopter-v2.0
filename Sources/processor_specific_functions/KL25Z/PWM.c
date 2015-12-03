/*
 * PWM.c
 *
 *  Created on: Dec 19, 2014
 *      Author: Norbi
 */
#include "init.h"

void init_PWM()
{
	SIM_SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK;// We Want MCGPLLCLK/2 (See Page 196 of the KL25 Sub-Family Reference Manual, Rev. 3, September 2012)
    SIM_SOPT2 &= ~(SIM_SOPT2_TPMSRC_MASK);
    SIM_SOPT2 |= SIM_SOPT2_TPMSRC(1); //We want the MCGPLLCLK/2 (See Page 196 of the KL25 Sub-Family Reference Manual, Rev. 3, September 2012)

	SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK;							//Clock enable to PWM
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK;	//Enable clock to PTA, PTC, PTD
	TPM0_SC = 0;							//Zero registers
	TPM0_CONF = 0;
    TPM0_SC = TPM_SC_PS(TPM0_CLK_PRESCALE);	//Set the system clock prescaler prescler
    TPM0_MOD = TPM0_CLOCK/(1<<TPM0_CLK_PRESCALE)/PWM_FREQUNECY;	//Set the frequency of Timer/PWM
}
void init_Motor_PWM()
{

	init_PWM();
	//Setup Channels 0,1,2,3
	TPM0_C0SC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;	//M1	Channel Mode select, Edge or level select
	TPM0_C2SC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;	//M2	Channel Mode select, Edge or level select
	TPM0_C3SC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;	//M3	Channel Mode select, Edge or level select
	TPM0_C4SC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;	//M4	Channel Mode select, Edge or level select
	
	TPM0_C0V = 0; 	//M1	Set duty cycle to 0%
	TPM0_C2V = 0;	//M2	Set duty cycle to 0%
	TPM0_C3V = 0;	//M3	Set duty cycle to 0%
	TPM0_C4V = 0;	//M4	Set duty cycle to 0%
	
    PORTD_PCR0 = PORT_PCR_MUX(4);		//M1 - A	TPM0_CH0
    PORTA_PCR5  = PORT_PCR_MUX(3);     	//M2 - B	TPM0_CH2	
    PORTC_PCR4 = PORT_PCR_MUX(4);		//M3 - C	TPM0_CH3
    PORTD_PCR4 = PORT_PCR_MUX(4);		//M4 - D	TPM0_CH4
        
    TPM0_SC |= TPM_SC_CMOD(1);	//Enable the counter
}

/*-------------------------------LED PWM----------------------------------------------------*/
//------------------------------------------------------------------------------------------//
/*------------------------------------------------------------------------------------------*/
void PWM_LED_RED()		//LED on PTB18
{
	TPM2_C0V = TPM2_MOD;
    TPM2_C0SC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;	//RED	Channel Mode select,//Edge or level Select
    PORTB_PCR18 = PORT_PCR_MUX(3);
}
void PWM_LED_GREEN()	//LED on PTB19
{
	TPM2_C1V = TPM2_MOD;
    TPM2_C1SC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;	//GREEN	Channel Mode select,//Edge or level Select
    PORTB_PCR19 = PORT_PCR_MUX(3);     
}
void PWM_LED_BLUE()		//LED on PTD1
{

	TPM0_C1V = TPM0_MOD;		//CONFLICT!!!!
    TPM0_C1SC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;	//BLUE	Channel Mode select,//Edge or level Select
    PORTD_PCR1 = PORT_PCR_MUX(4);  
}
void init_PWM_LED()
{
	PWM_LED_GREEN();
	PWM_LED_RED();
	PWM_LED_BLUE();
}
void Set_LEDPWM(float RED , float GREEN, float BLUE)
{
	
	if(RED>1000)
		RED = 1000;
	else if(RED<0)
		RED = 0;
	
	if(GREEN>1000)
		GREEN = 1000;
	else if(GREEN<0)
		GREEN = 0;
	
	if(BLUE>1000)
		BLUE = 1000;
	else if(BLUE<0)
		BLUE = 0;
	
	TPM2_C0V = (uint16_t)(TPM2_MOD-(TPM2_MOD*((float)RED/1000)));
	TPM2_C1V = (uint16_t)(TPM2_MOD-(TPM2_MOD*((float)GREEN/1000)));
	TPM0_C1V = (uint16_t)(TPM0_MOD-(TPM0_MOD*((float)BLUE/1000)));

}
