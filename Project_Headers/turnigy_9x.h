/*
 * turnigy_9x.h
 *
 *  Created on: Jan 29, 2016
 *      Author: Norbert
 */

#ifndef TURNIGY_9X_H_
#define TURNIGY_9X_H_

#define TURNIGY_CALIBRATE_NUMBER 20
#define	PPM_FILTER	0.9
//------------------------Turnigy 9x------------------------------------------------------------
#define ch1_pin	PORTA_PCR14
#define ch2_pin	PORTA_PCR15
#define ch3_pin	PORTA_PCR16
#define ch4_pin	PORTA_PCR17

#define ch1_pulse	((ch1_pin & PORT_PCR_ISF_MASK) == PORT_PCR_ISF_MASK)
#define ch2_pulse	((ch2_pin & PORT_PCR_ISF_MASK) == PORT_PCR_ISF_MASK)
#define ch3_pulse	((ch3_pin & PORT_PCR_ISF_MASK) == PORT_PCR_ISF_MASK)
#define ch4_pulse	((ch4_pin & PORT_PCR_ISF_MASK) == PORT_PCR_ISF_MASK)

#define ch1_rising_edge	((ch1_pin & PORT_PCR_IRQC(0b1010)) == PORT_PCR_IRQC(0b1010))
#define ch2_rising_edge	((ch2_pin & PORT_PCR_IRQC(0b1010)) == PORT_PCR_IRQC(0b1010))
#define ch3_rising_edge	((ch3_pin & PORT_PCR_IRQC(0b1010)) == PORT_PCR_IRQC(0b1010))
#define ch4_rising_edge	((ch4_pin & PORT_PCR_IRQC(0b1010)) == PORT_PCR_IRQC(0b1010))

#define ch1_falling_edge	((ch1_pin & PORT_PCR_IRQC(0b1010)) == PORT_PCR_IRQC(0b1001))
#define ch2_falling_edge	((ch2_pin & PORT_PCR_IRQC(0b1010)) == PORT_PCR_IRQC(0b1001))
#define ch3_falling_edge	((ch3_pin & PORT_PCR_IRQC(0b1010)) == PORT_PCR_IRQC(0b1001))
#define ch4_falling_edge	((ch4_pin & PORT_PCR_IRQC(0b1010)) == PORT_PCR_IRQC(0b1001))

#define ch1_set_falling_edge	ch1_pin = (ch1_pin & ~PORT_PCR_IRQC_MASK) | PORT_PCR_IRQC(0b1010)
#define ch2_set_falling_edge	ch2_pin = (ch2_pin & ~PORT_PCR_IRQC_MASK) | PORT_PCR_IRQC(0b1010)
#define ch3_set_falling_edge	ch3_pin = (ch3_pin & ~PORT_PCR_IRQC_MASK) | PORT_PCR_IRQC(0b1010)
#define ch4_set_falling_edge	ch4_pin = (ch4_pin & ~PORT_PCR_IRQC_MASK) | PORT_PCR_IRQC(0b1010)

#define ch1_set_rising_edge		ch1_pin = (ch1_pin & ~PORT_PCR_IRQC_MASK) | PORT_PCR_IRQC(0b1001)
#define ch2_set_rising_edge		ch2_pin = (ch2_pin & ~PORT_PCR_IRQC_MASK) | PORT_PCR_IRQC(0b1001)
#define ch3_set_rising_edge		ch3_pin = (ch3_pin & ~PORT_PCR_IRQC_MASK) | PORT_PCR_IRQC(0b1001)
#define ch4_set_rising_edge		ch4_pin = (ch4_pin & ~PORT_PCR_IRQC_MASK) | PORT_PCR_IRQC(0b1001)

#define ch1_clear_interrupt		ch1_pin |= PORT_PCR_ISF_MASK
#define ch2_clear_interrupt		ch2_pin |= PORT_PCR_ISF_MASK
#define ch3_clear_interrupt		ch3_pin |= PORT_PCR_ISF_MASK
#define ch4_clear_interrupt		ch4_pin |= PORT_PCR_ISF_MASK

#define ch2_high	(GPIOA_PDIR & 1<<14)
//-------------------------------------------------------------------------------------------

void turnigy_9x();
void init_turnigy_timer();
void init_turnigy9x();

extern int32_t ch1,ch2,ch3,ch4;
extern int32_t ch1_temp,ch2_temp,ch3_temp,ch4_temp;
extern int32_t ch1_offset,ch2_offset,ch3_offset,ch4_offset;

#endif /* TURNIGY_9X_H_ */
