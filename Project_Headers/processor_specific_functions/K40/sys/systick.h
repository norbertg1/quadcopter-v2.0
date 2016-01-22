
#ifndef SYSTICK_H_
#define SYSTICK_H_

void InitSysTick();
void Delay_mS(unsigned int TicksIn_mS);

void SysTickIrq();

#define SYSTICK_FREQUENCY 1000

#endif /* SYSTICK_H_ */
