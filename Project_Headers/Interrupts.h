/*
 * Interrupts.h
 *
 *  Created on: Dec 10, 2014
 *      Author: Norbi
 */

#ifndef INTERRUPTS_H_
#define INTERRUPTS_H_

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
