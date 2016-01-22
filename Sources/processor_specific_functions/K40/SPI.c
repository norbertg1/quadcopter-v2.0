/*
 * SPI.c
 *
 *  Created on: Dec 19, 2014
 *      Author: Norbi
 */

#include "init.h"

char spi_send(char spiMsg)  
{  
   return 0;
}

void init_SPI0()		//SPI1 for SDcard
{
/*	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;      //Turn on clock to E module  
	SIM_SCGC6 |= SIM_SCGC6_SPI1_MASK;       //Enable SPI0 clock  
	  
	PORTE_PCR4 = PORT_PCR_MUX(0x2);    //Set PTE4 to mux 2 [SPI0_PCS0]  
	PORTE_PCR2 = PORT_PCR_MUX(0x2);    //Set PTE2 to mux 2 [SPI0_SCK]  
	PORTE_PCR1 = PORT_PCR_MUX(0x2);    //Set PTE1 to mux 2 [SPI0_MOSI] on K40 [SPI0_SOUT]  
	PORTE_PCR3 = PORT_PCR_MUX(0x2);    //Set PTE3 to mux 2 [SPIO_MISO] on K40 [SPIO_SIN]
	
	PORTC_PCR12 = PORT_PCR_MUX(1);	//PORTC9 
	GPIOC_PDDR |= 1<<12;				//PORTC9 is output, for CS, Ha a CS nem mûködne!!!
	CONFLIKT*/ 
	
}
