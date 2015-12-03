#ifndef ARM_SYSTICK_H_
#define ARM_SYSTICK_H_

void InitSysTick();
void Delay_mS(unsigned int TicksIn_mS);
void SysTickIrq();

#define SYSTICK_FREQUENCY 1000

//A TFC Ticker is a variable that will increment every 1mS in the Systick interrupt routine
//you can use it to for general purpose timing, scheduling events, etc.  The TFC_Ticker variable
//is just an array of 32-bit integers, use the Macro below to set how many tickers you need

#define NUM_TICKERS			4

extern volatile uint32_t Ticker[NUM_TICKERS];

#endif /* SYSTICK_H_ */

