/*
 * Interrupts.h
 *
 *  Created on: Dec 10, 2014
 *      Author: Norbi
 */

#ifndef INTERRUPTS_H_
#define INTERRUPTS_H_

#define enable_PID_interrupts		TPM1_SC |= TPM_SC_TOIE_MASK;
#define enable_SDcard_interrupts	TPM2_SC |= TPM_SC_TOIE_MASK;
#define clear_PID_interrupt			TPM1_SC |= TPM_SC_TOF_MASK;
#define clear_SDcard_interrupt		TPM2_SC |= TPM_SC_TOF_MASK;

void PID_Interrupt(void);
void SDcardw_Interrupt(void);
void bluetooth_getchar();
void bluetooth_lostconnection();
void ADC();
void ADC_DMA();

float Kp,Kd,Ki,zKp,zKd;	//2.0 0.6 1.0 1 0.3
int basepower,setpoint_x,setpoint_y,setpoint_z;
float setpoint_alt;
float batt1_vol,batt2_vol,batt3_vol,BATT_VOLT;

#endif /* INTERRUPTS_H_ */
