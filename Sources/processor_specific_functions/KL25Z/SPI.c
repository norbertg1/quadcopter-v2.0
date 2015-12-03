/*
 * SPI.c
 *
 *  Created on: Dec 19, 2014
 *      Author: Norbi
 */

#include "init.h"

char spi_send(char spiMsg)  
{  
  while(!(SPI_S_SPTEF_MASK & SPI1_S))  
  {      
    asm("nop");  //While buffer is not empty do nothing  
  } 
  SPI1_D = spiMsg & 0xff;    //Write char to SPI  
  while(!(SPI_S_SPRF_MASK & SPI1_S))
  {
	  asm("nop");
  }
  return SPI1_D;
}

void init_SPI0()		//SPI1 for SDcard
{
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK | SIM_SCGC5_PORTC_MASK;      //Turn on clock to E module  
	SIM_SCGC4 |= SIM_SCGC4_SPI1_MASK;       //Enable SPI0 clock  
	  
	PORTE_PCR4 = PORT_PCR_MUX(0x2);    //Set PTE4 to mux 2 [SPI0_PCS0]  
	PORTE_PCR2 = PORT_PCR_MUX(0x2);    //Set PTE2 to mux 2 [SPI0_SCK]  
	PORTE_PCR1 = PORT_PCR_MUX(0x2);    //Set PTE1 to mux 2 [SPI0_MOSI]  
	PORTE_PCR3 = PORT_PCR_MUX(0x2);    //Set PTE3 to mux 2 [SPIO_MISO]  
	
	PORTC_PCR12 = PORT_PCR_MUX(1);	//PORTC9 
	GPIOC_PDDR |= 1<<12;				//PORTC9 is output, for CS
	
	SPI1_C1 = SPI_C1_MSTR_MASK | SPI_C1_SSOE_MASK;   //Set SPI0 to Master & SS pin to auto SS  
	    
	SPI1_C2 = SPI_C2_MODFEN_MASK;   //Master SS pin acts as slave select output   
	    
	SPI1_BR = (SPI_BR_SPPR(0x00) | SPI_BR_SPR(0x06));     //baud rate prescale divisor to 0 & set baud rate divisor to 128 for baud rate of: (0+1)*2^(0x6+1)=128; 24Mhz/128=187500Hz  
	    
	SPI1_C1 |= SPI_C1_SPE_MASK;    //Enable SPI1  
}
