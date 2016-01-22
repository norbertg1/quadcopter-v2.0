/*
 * misc.h
 *
 *  Created on: Aug 5, 2014
 *      Author: Norbi
 */

#ifndef MISC_H_
#define MISC_H_

//------------------------LED TOGGLE------------------------------------------
#define	LED_Toggle_Blue		GPIOD_PTOR = 1<<1;	//A kek ki-be kapcsolgatása	NOT TESTED
#define	LED_Toggle_RED		GPIOB_PTOR = 1<<18;	//A piros ki-be kapcsolgatása
#define	LED_Toggle_Green	GPIOB_PTOR = 1<<19;	//A kek ki-be kapcsolgatása
//------------------------LED On/off-----------------------------------------------
#define	LED_On_BLUE		GPIOD_PCOR = 1<<1;		//A kék be kapcsolása
#define	LED_Off_BLUE	GPIOD_PSOR = 1<<1;		//A kék ki kapcsolása
#define	LED_On_RED		GPIOB_PCOR = 1<<18;		//A piros be kapcsolása
#define	LED_Off_RED		GPIOB_PSOR = 1<<18;		//A piros ki kapcsolása
#define	LED_On_GREEN	GPIOB_PCOR = 1<<19;		//A zöld be kapcsolása
#define	LED_Off_GREEN	GPIOB_PSOR = 1<<19;		//A zöld ki kapcsolása
//------------------------LED_init---------------------------------------------
#define enable_PID_interrupts		PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK;
#define enable_SDcard_interrupts	PIT_TCTRL2 |= PIT_TCTRL_TIE_MASK;
#define clear_PID_interrupt			PIT_TFLG1  |= PIT_TFLG_TIF_MASK;
#define clear_SDcard_interrupt		PIT_TFLG2  |= PIT_TFLG_TIF_MASK;
//-----------------------Timer module settings---------------------------------
#define elapsed_s	RTC_TSR
#define ticker	PIT_CVAL0/100		//ezzel a szamlaloval tudok idor mértni ket esemeny közt 
#define PIT0_OVERFLOW_FREQUENCY 400
#define PIT1_OVERFLOW_FREQUENCY 400

void initRed();
void initGreen();
void initBlue();
void initLEDs();
void init_Ticker();
void RTC();
void initTimer0();
void initTimer1();
void initBluetooth();

#endif /* MISC_H_ */
