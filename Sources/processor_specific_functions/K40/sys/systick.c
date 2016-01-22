#include "init.h"
#include "processor_specific_functions\K40\sys\derivative.h"


//Since this SysTick is part of the Cortex M4 Core,   you need to look in the 
//Cortex M4 Generic Users Guide (ARM DUI 0553A)

//See Section 4.4
void InitSysTick()
{
	SYST_RVR = CORE_CLOCK/SYSTICK_FREQUENCY;
	SYST_CSR = SysTick_CSR_ENABLE_MASK | SysTick_CSR_TICKINT_MASK | SysTick_CSR_CLKSOURCE_MASK;

	//Important!  Since the Systick is part of the Cortex core and NOT a kinetis peripheral
	// its Interrupt line is not passed through NVIC.   You need to make sure that
	//the SysTickIRQ function is poopulated in the vector table.  See the kinetis_sysinit.c file
}

static volatile unsigned int DelayTimerTick = 0;

void SysTickIrq()
{
	if(DelayTimerTick<0xFFFFFFFF)
	{
		DelayTimerTick++;
	}
}

void Delay_mS(unsigned int TicksIn_mS)
{
	DelayTimerTick = 0;

	while(DelayTimerTick<TicksIn_mS)
	{
		//asm("WFI"); // wait for an interrupt
	}
}

