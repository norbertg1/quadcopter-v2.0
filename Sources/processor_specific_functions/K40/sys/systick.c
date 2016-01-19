/*
 * systick.c
 *
 *  Created on: Jul 14, 2013
 *      Author: ML
 */

#include "processor_specific_functions\k40\sys\derivative.h"
#include "processor_specific_functions\k40\sys\systick.h"
#include "processor_specific_functions\k40\sys\clock.h"

static volatile uint32_t DelayTimerTick = 0;
volatile uint32_t Tick = 0;

void initSysTick(void)
{
	SYST_RVR = CORE_CLOCK / SYSTICK_FREQUENCY;	// Set the reload to our desired frequency
	SYST_CVR = 0; // Reset the current value
	SYST_CSR = SysTick_CSR_ENABLE_MASK | SysTick_CSR_TICKINT_MASK | SysTick_CSR_CLKSOURCE_MASK;
	
	// IMPORTANT! Systick is part of cortex core not a kinetis peripheral
	// its interrupt line is not passed through NVIC. You need to make sure that
	// the SysTickIRQ function is populated in the vector table. See the kinetis_sysinit.c file
}

void SysTick_Handler(void)
{
		DelayTimerTick++;
}

void clearSysTick(void)
{
	Tick = DelayTimerTick;
}

int getSysTick(void)
{
	return (DelayTimerTick - Tick);
}

void Delay_mS(uint32_t tick_Ms)
{
	DelayTimerTick = 0;
	
	while(DelayTimerTick < tick_Ms)
	{
		
	}
}
