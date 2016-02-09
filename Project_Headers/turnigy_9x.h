/*
 * turnigy_9x.h
 *
 *  Created on: Jan 29, 2016
 *      Author: Norbert
 */

#ifndef TURNIGY_9X_H_
#define TURNIGY_9X_H_

#define TURNIGY_CALIBRATE_NUMBER 20
#define	PPM_FILTER_CH1	0.85
#define	PPM_FILTER_CH2	0.85
#define	PPM_FILTER_CH3	0.85
#define	PPM_FILTER_CH4	0.85
#define	PPM_FILTER_CH5	0.98
#define	PPM_FILTER_CH6	0.98
#define	PPM_FILTER_CH7	0.98

//------------------------Turnigy 9x------------------------------------------------------------
#define ch1_pin	PORTA_PCR14
#define ch2_pin	PORTA_PCR15
#define ch3_pin	PORTA_PCR16
#define ch4_pin	PORTA_PCR17
#define ch5_pin	PORTE_PCR8
#define ch6_pin	PORTE_PCR9
#define ch7_pin	PORTE_PCR10

#define ch1_pulse	((ch1_pin & PORT_PCR_ISF_MASK) == PORT_PCR_ISF_MASK)
#define ch2_pulse	((ch2_pin & PORT_PCR_ISF_MASK) == PORT_PCR_ISF_MASK)
#define ch3_pulse	((ch3_pin & PORT_PCR_ISF_MASK) == PORT_PCR_ISF_MASK)
#define ch4_pulse	((ch4_pin & PORT_PCR_ISF_MASK) == PORT_PCR_ISF_MASK)
#define ch5_pulse	((ch5_pin & PORT_PCR_ISF_MASK) == PORT_PCR_ISF_MASK)
#define ch6_pulse	((ch6_pin & PORT_PCR_ISF_MASK) == PORT_PCR_ISF_MASK)
#define ch7_pulse	((ch7_pin & PORT_PCR_ISF_MASK) == PORT_PCR_ISF_MASK)


#define ch1_rising_edge	((ch1_pin & PORT_PCR_IRQC(0b1010)) == PORT_PCR_IRQC(0b1010))
#define ch2_rising_edge	((ch2_pin & PORT_PCR_IRQC(0b1010)) == PORT_PCR_IRQC(0b1010))
#define ch3_rising_edge	((ch3_pin & PORT_PCR_IRQC(0b1010)) == PORT_PCR_IRQC(0b1010))
#define ch4_rising_edge	((ch4_pin & PORT_PCR_IRQC(0b1010)) == PORT_PCR_IRQC(0b1010))
#define ch5_rising_edge	((ch5_pin & PORT_PCR_IRQC(0b1010)) == PORT_PCR_IRQC(0b1010))
#define ch6_rising_edge	((ch6_pin & PORT_PCR_IRQC(0b1010)) == PORT_PCR_IRQC(0b1010))
#define ch7_rising_edge	((ch7_pin & PORT_PCR_IRQC(0b1010)) == PORT_PCR_IRQC(0b1010))

#define ch1_falling_edge	((ch1_pin & PORT_PCR_IRQC(0b1010)) == PORT_PCR_IRQC(0b1001))
#define ch2_falling_edge	((ch2_pin & PORT_PCR_IRQC(0b1010)) == PORT_PCR_IRQC(0b1001))
#define ch3_falling_edge	((ch3_pin & PORT_PCR_IRQC(0b1010)) == PORT_PCR_IRQC(0b1001))
#define ch4_falling_edge	((ch4_pin & PORT_PCR_IRQC(0b1010)) == PORT_PCR_IRQC(0b1001))
#define ch5_falling_edge	((ch5_pin & PORT_PCR_IRQC(0b1010)) == PORT_PCR_IRQC(0b1001))
#define ch6_falling_edge	((ch6_pin & PORT_PCR_IRQC(0b1010)) == PORT_PCR_IRQC(0b1001))
#define ch7_falling_edge	((ch7_pin & PORT_PCR_IRQC(0b1010)) == PORT_PCR_IRQC(0b1001))

#define ch1_set_falling_edge	ch1_pin = (ch1_pin & ~PORT_PCR_IRQC_MASK) | PORT_PCR_IRQC(0b1010)
#define ch2_set_falling_edge	ch2_pin = (ch2_pin & ~PORT_PCR_IRQC_MASK) | PORT_PCR_IRQC(0b1010)
#define ch3_set_falling_edge	ch3_pin = (ch3_pin & ~PORT_PCR_IRQC_MASK) | PORT_PCR_IRQC(0b1010)
#define ch4_set_falling_edge	ch4_pin = (ch4_pin & ~PORT_PCR_IRQC_MASK) | PORT_PCR_IRQC(0b1010)
#define ch5_set_falling_edge	ch5_pin = (ch5_pin & ~PORT_PCR_IRQC_MASK) | PORT_PCR_IRQC(0b1010)
#define ch6_set_falling_edge	ch6_pin = (ch6_pin & ~PORT_PCR_IRQC_MASK) | PORT_PCR_IRQC(0b1010)
#define ch7_set_falling_edge	ch7_pin = (ch7_pin & ~PORT_PCR_IRQC_MASK) | PORT_PCR_IRQC(0b1010)

#define ch1_set_rising_edge		ch1_pin = (ch1_pin & ~PORT_PCR_IRQC_MASK) | PORT_PCR_IRQC(0b1001)
#define ch2_set_rising_edge		ch2_pin = (ch2_pin & ~PORT_PCR_IRQC_MASK) | PORT_PCR_IRQC(0b1001)
#define ch3_set_rising_edge		ch3_pin = (ch3_pin & ~PORT_PCR_IRQC_MASK) | PORT_PCR_IRQC(0b1001)
#define ch4_set_rising_edge		ch4_pin = (ch4_pin & ~PORT_PCR_IRQC_MASK) | PORT_PCR_IRQC(0b1001)
#define ch5_set_rising_edge		ch5_pin = (ch5_pin & ~PORT_PCR_IRQC_MASK) | PORT_PCR_IRQC(0b1001)
#define ch6_set_rising_edge		ch6_pin = (ch6_pin & ~PORT_PCR_IRQC_MASK) | PORT_PCR_IRQC(0b1001)
#define ch7_set_rising_edge		ch7_pin = (ch7_pin & ~PORT_PCR_IRQC_MASK) | PORT_PCR_IRQC(0b1001)

#define ch1_clear_interrupt		ch1_pin |= PORT_PCR_ISF_MASK
#define ch2_clear_interrupt		ch2_pin |= PORT_PCR_ISF_MASK
#define ch3_clear_interrupt		ch3_pin |= PORT_PCR_ISF_MASK
#define ch4_clear_interrupt		ch4_pin |= PORT_PCR_ISF_MASK
#define ch5_clear_interrupt		ch5_pin |= PORT_PCR_ISF_MASK
#define ch6_clear_interrupt		ch6_pin |= PORT_PCR_ISF_MASK
#define ch7_clear_interrupt		ch7_pin |= PORT_PCR_ISF_MASK

//-------------------------------------------------------------------------------------------

void turnigy_9x();
void init_turnigy_timer();
void init_turnigy9x();

extern int32_t ch1,ch2,ch3,ch4,ch5,ch6,ch7;
extern int32_t ch1_temp,ch2_temp,ch3_temp,ch4_temp,ch5_temp,ch6_temp,ch7_temp;
extern int32_t ch1_offset,ch2_offset,ch3_offset,ch4_offset,ch5_offset,ch6_offset,ch7_offset;
extern int32_t ch3_watchdog;

#endif /* TURNIGY_9X_H_ */
