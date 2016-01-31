#include "init.h"
//#include "processor_specific_functions/KL25Z/sys/MKL25Z4.h"

#define TPM0_CLOCK			(CORE_CLOCK/2)
#define TPM0_CLK_PRESCALE	4  				// Prescale Selector value - see comments in Status Control (SC) section for more details
#define PID_frequency 		400				//PID frequency
#define LOG_frequency 		400				//SDcard frequency

/*
 * FRDM-KL25Z
 * LED:
 * 	initLEDS() - initialize RGB LEDs on freedom board, must called first!
 * 	xLED_Toggle() - Turn On/Off x LED, where x=R,G,B
 *  xLED_Off	  - Turn Off x LED where x=R,G,B
 *  xLED_On		  - Turn On x LED where x=R,G,B
 *  
 * -------------------------------------------------------------------------------------------------------------
 *  initTimer0 - setup timer module TPM1, #define PID_frequency 400	is the frequency
 *  initTimer1 - setup timer module TPM2, #define LOG_frequency 400	is the frequencz
 *  
 * Created by Norbi 2015.11
 */

void init_Ticker()	//Timer modul, idõmérési célokra
{
	/*Enable Timer, Value are in PIT_CVALn register, prescaler is 
	  PIT is downcounting from
	  1s = 2 990 428 value in counter with prescaler 16 - my mesurement
	  48Mhz/16 = 48 000 000/16=3 000 000 novekedes 1masodperc alatt, maximum 0,02184533s merese utanna tulcsordul
	*/
	SIM_SCGC6  |= SIM_SCGC6_PIT_MASK;	//Enable clock
	PIT_MCR    = 0;		//Timers are stopped in Debug mode.
	PIT_MCR    |= PIT_MCR_FRZ_MASK;		//Timers are stopped in Debug mode.
	PIT_LDVAL0	= 6553500;
	PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;	//Enable Timer 
}

void RTC()		//Increment RTC_TSR register every second	KL25
{
	
	SIM_SCGC6 |= SIM_SCGC6_RTC_MASK;	//enable clock to RTC
	RTC_TSR  = 1;	//Clear the time invalid flag (TIF)
	RTC_CR |= RTC_CR_OSCE_MASK;
	Delay_mS(100);
	RTC_SR |= RTC_SR_TCE_MASK;
	
/*
	SIM_SCGC6 |= SIM_SCGC6_RTC_MASK;	//Masodperc szamlalo
	RTC_CR |= RTC_CR_OSCE_MASK;
	Delay_mS(100);
	RTC_SR = 0;
	RTC_TSR =  0;
	RTC_SR |= RTC_SR_TCE_MASK;*/
}


/*-------------------------------PIT Timer--------------------------------------------------*/
//------------------------------------------------------------------------------------------//
/*------------------------------------------------------------------------------------------*/
/*
 * PIT0 - PID Interrupt
 * PIT1 - SD card write interrupt
 */

void initTimer0()		//PIT0 Timer for PID
{
	SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
	PIT_MCR = 0;
	PIT_MCR = PIT_MCR_FRZ_MASK;		// Enable PIT module clock with debug freeze
	PIT_LDVAL0 = PERIPHERAL_BUS_CLOCK / PIT0_OVERFLOW_FREQUENCY;	// Calculate and Load timer reset value
//	PIT_TCTRL0 = PIT_TCTRL_TIE_MASK;	// Enable timer interrupt
	PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;	// Enable timer
}

void initTimer1()		//TPM2 Timer for SDcard save
{
	SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
	PIT_MCR = 0;
	PIT_MCR = PIT_MCR_FRZ_MASK;		// Enable PIT module clock with debug freeze	
	PIT_LDVAL1 = PERIPHERAL_BUS_CLOCK / PIT1_OVERFLOW_FREQUENCY;	// Calculate and Load timer reset value
//	PIT_TCTRL1 = PIT_TCTRL_TIE_MASK;	// Enable timer interrupt
	PIT_TCTRL1 |= PIT_TCTRL_TEN_MASK;	// Enable timer
}

void initTimer2()		//TPM2 Timer for Turnigy 9x PPM
{
	SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
	PIT_MCR = 0;
	PIT_MCR = PIT_MCR_FRZ_MASK;		// Enable PIT module clock with debug freeze	
	PIT_LDVAL2 = 0xFFFFFFFF;	// Calculate and Load timer reset value
//	PIT_TCTRL1 = PIT_TCTRL_TIE_MASK;	// Enable timer interrupt
	PIT_TCTRL2 |= PIT_TCTRL_TEN_MASK;	// Enable timer
}

void initBluetooth()		//Configure interrupt on lost signal
{
	PORTA_PCR5 = PORT_PCR_MUX(1);
	GPIOA_PDDR &= ~(1<<5);
	PORTA_PCR5 |= PORT_PCR_IRQC(0b1011);
}

void initRed()		//A panelon levõ piros LED inicializálása
{
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	PORTB_PCR18 = PORT_PCR_MUX(1);
	GPIOB_PSOR |= 1<<18;
	GPIOB_PDDR |= 1<<18;
}

void initGreen()		//A panelon levõ piros LED inicializálása
{
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	PORTB_PCR19 = PORT_PCR_MUX(1);
	GPIOB_PSOR |= 1<<19;
	GPIOB_PDDR |= 1<<19;
}

void initBlue()		//A panelon levõ piros LED inicializálása, NOT TESTED
{
	SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
	PORTD_PCR1 = PORT_PCR_MUX(1);
	GPIOD_PSOR |= 1<<1;
	GPIOD_PDDR |= 1<<1;
}
void initLEDs()
{
	initRed();
	initGreen();
	initBlue();
}
