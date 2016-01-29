/*
 * SPI.h
 *
 *  Created on: Dec 19, 2014
 *      Author: Norbi
 */

#ifndef SPI_H_
#define SPI_H_

#define CS_HIGH	GPIOE_PSOR |= 1<<4;		//+3.3V, Logical High, CS HIGH, PORTE4
#define CS_LOW	GPIOE_PCOR |= 1<<4;		//GND, Logical Low, CS LOW, PORTE4

void init_SPI1();
char spi_send(char spiMsg);

//--------------------------------------------hal spi-----------------------------------------
void hal_spi_init(void);
void hal_spi_transfe_start(void);
void hal_spi_transfe_stop(void);
uint8_t   hal_spi_transfer_one_byte(uint8_t v, uint8_t end);


#endif /* SPI_H_ */
