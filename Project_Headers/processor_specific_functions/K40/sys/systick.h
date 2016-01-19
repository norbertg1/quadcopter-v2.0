/*
 * systick.h
 *
 *  Created on: Jul 14, 2013
 *      Author: ML
 */

#ifndef SYSTICK_H_
#define SYSTICK_H_



// Definitions
#define SYSTICK_FREQUENCY 1000



void initSysTick(void);
void SysTick_Handler(void);
void clearSysTick(void);
int getSysTick(void);
void Delay_mS(uint32_t tick_Ms);



#endif /* SYSTICK_H_ */
