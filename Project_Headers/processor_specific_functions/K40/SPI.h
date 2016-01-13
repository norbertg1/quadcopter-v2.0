/*
 * SPI.h
 *
 *  Created on: Dec 19, 2014
 *      Author: Norbi
 */

#ifndef SPI_H_
#define SPI_H_

#define CS_HIGH	GPIOC_PSOR |= 1<<12;		//+3.3V, Logical High, CS HIGH, PORTC9
#define CS_LOW	GPIOC_PCOR |= 1<<12;		//GND, Logical Low, CS LOW, PORTC9

void init_SPI0();
char spi_send(char spiMsg);

#endif /* SPI_H_ */
