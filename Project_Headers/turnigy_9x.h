/*
 * turnigy_9x.h
 *
 *  Created on: Jan 29, 2016
 *      Author: Norbert
 */

#ifndef TURNIGY_9X_H_
#define TURNIGY_9X_H_

#define TURNIGY_FREQUENCY	50

//------------------------Turnigy 9x---------------------------------------------
#define ch0_pin	PORTA_PCR14
#define ch1_pin	PORTA_PCR15
#define ch2_pin	PORTA_PCR16
#define ch3_pin	PORTA_PCR17

#define ch0_pulse	(ch0_pin & PORT_PCR_ISF_MASK == PORT_PCR_ISF_MASK)
#define ch1_pulse	(ch1_pin & PORT_PCR_ISF_MASK == PORT_PCR_ISF_MASK)
#define ch2_pulse	(ch2_pin & PORT_PCR_ISF_MASK == PORT_PCR_ISF_MASK)
#define ch3_pulse	(ch3_pin & PORT_PCR_ISF_MASK == PORT_PCR_ISF_MASK)
//--------------------------------------------------------------------------------

void synchronize_9x();
void init_turnigy_timer();
void init_turnigy9x();
void capture_ppm();

extern uint8_t ch0,ch1,ch2,ch3; 


#endif /* TURNIGY_9X_H_ */
