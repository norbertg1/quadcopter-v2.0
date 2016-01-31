/*
 * PWM.h
 *
 *  Created on: Dec 19, 2014
 *      Author: Norbi
 */

#ifndef PWM_H_
#define PWM_H_

#define FTM_MOD_VALUE	(int)((float)(PERIPHERAL_BUS_CLOCK)/TFC_MOTOR_SWITCHING_FREQUENCY)

#define MOTOR_A		FTM1_C0V
#define MOTOR_B		FTM1_C1V
#define MOTOR_C		FTM2_C0V
#define MOTOR_D		FTM2_C1V

#define FTM_CLOCK                                   	      (CORE_CLOCK/2)
#define PWM_FREQUNECY 400								  //the frequeny of PWM
#define FTM_CLK_PRESCALE	4							// Prescale Selector value - see comments in Status Control (SC) section for more details
#define ESC_MINIMUM_POWER 1060							//Pulse width for minimum power
#define ESC_MAXIMUM_POWER 1860							//Pulse width for maximum power
#define ESC_POWER_DIFFERENCE (ESC_MAXIMUM_POWER-ESC_MINIMUM_POWER)	//Pulse width difference
#define FTM_MODn FTM_CLOCK/(1<<FTM_CLK_PRESCALE)/PWM_FREQUNECY
#define ELEMENTARY_PROC_TIME ((FTM_MODn)/(1000/(float)PWM_FREQUNECY))	//1500
/**********************************************************************************************/

void init_Motor_PWM();
void init_PWM_LED();
void Set_LEDPWM(float RED , float GREEN, float BLUE);


#endif /* PWM_H_ */
 
