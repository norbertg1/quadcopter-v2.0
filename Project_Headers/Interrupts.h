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
void UART_interrupt();
void bluetooth_lostconnection();
void ADC0();
void ADC1();
void ADC_DMA();
void capture_ppm_PORTA();
void capture_ppm_PORTE();
void FTM0_interrupt();

extern float Kp,Kd,Ki,Kp_prev,Kd_prev,Ki_prev,Kp_user,Kd_user,Ki_user,Kp_receiver,Kd_receiver,Ki_receiver,zKp,zKd;	//2.0 0.6 1.0 1 0.3
extern int basepower,setpoint_x,setpoint_y,setpoint_z;
extern float setpoint_alt;
extern  float bat1_volt,bat2_volt,bat3_volt,BAT_VOLT;
extern int32_t t_period;

#endif /* INTERRUPTS_H_ */
