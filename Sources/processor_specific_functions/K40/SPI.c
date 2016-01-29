/*
 * SPI.c
 *
 *  Created on: Dec 19, 2014
 *      Author: Norbi
 */

#include "init.h"

char spi_send(char spiMsg)  
{  
	char temp;
	while(!(SPI1_SR & SPI_SR_TFFF_MASK));  
	{      
		asm("nop");  //While buffer is not empty do nothing  
	} 
	SPI1_PUSHR = spiMsg & 0xff;    //Write char to SPI  
	while(!(SPI1_SR & SPI_SR_RFDF_MASK))
	{
		asm("nop");
	}
	temp = SPI1_POPR;
	SPI1_SR = SPI_SR_TFFF_MASK | (SPI1_SR & SPI_SR_RFDF_MASK);
	return temp;
}

void init_SPI1()		//SPI1 for SDcard
{
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;      //Turn on clock to module  
	SIM_SCGC6 |= SIM_SCGC6_SPI1_MASK;       //Enable SPI1 clock  
		
	PORTE_PCR6 = PORT_PCR_MUX(1);
	GPIOE_PDDR |= 1<<6;
	GPIOE_PSOR |= 1<<6;
	//Enable 3.3V to SDcard, see kwikstik schematics

//	PORTE_PCR4 = PORT_PCR_MUX(0x2);    //Set PTE4 to mux 2 [SPI1_PCS0]
	PORTE_PCR1 = PORT_PCR_MUX(0x2);    //Set PTE3 to mux 2 [SPI1_MOSI?]  
	PORTE_PCR2 = PORT_PCR_MUX(0x2);    //Set PTE1 to mux 2 [SPI1_SCK]
	PORTE_PCR3 = PORT_PCR_MUX(0x2);    //Set PTE2 to mux 2 [SPI1_MISO?]
	
	PORTE_PCR4 = PORT_PCR_MUX(1);	//PORTE1 is CS
	GPIOE_PDDR |= 1<<4;				//PORTE1 is output, for CS
	
	SPI1_MCR = 0;
	SPI1_MCR |= (SPI_MCR_MSTR_MASK | SPI_MCR_HALT_MASK | SPI_MCR_DCONF(0x0) | SPI_MCR_CLR_TXF_MASK | SPI_MCR_CLR_RXF_MASK);   //Set SPI0 to Master & SS pin to auto SS  
	SPI1_TCR =	SPI_TCR_SPI_TCNT(0x0);
	SPI1_CTAR0 = SPI_CTAR_FMSZ(7);
	SPI1_CTAR0 |= (SPI_CTAR_PBR(5) | SPI_CTAR_BR(8));
	SPI1_MCR &= ~SPI_MCR_HALT_MASK; 
}

//--------------------------------------hal spi-------------------------------

static void init_clock(void)
{
    SIM_SCGC6 |= SIM_SCGC6_SPI1_MASK;
}
static void init_io(void)
{
    PORTD_PCR11 &= ~PORT_PCR_MUX_MASK;
    PORTD_PCR11 |= PORT_PCR_MUX(2);
    PORTD_PCR12 &= ~PORT_PCR_MUX_MASK;
    PORTD_PCR12 |= PORT_PCR_MUX(2);
    PORTD_PCR13 &= ~PORT_PCR_MUX_MASK;
    PORTD_PCR13 |= PORT_PCR_MUX(2);
    PORTD_PCR14 &= ~PORT_PCR_MUX_MASK;
    PORTD_PCR14 |= PORT_PCR_MUX(2);
}
static void init_set_master(void)
{
    SPI1_MCR |= SPI_MCR_MSTR_MASK;
}
static void init_fifo(void)
{
    SPI1_MCR &= ~SPI_MCR_MDIS_MASK;

    SPI1_MCR |= SPI_MCR_DIS_RXF_MASK |
                SPI_MCR_DIS_TXF_MASK |
                SPI_MCR_CLR_RXF_MASK |
                SPI_MCR_CLR_TXF_MASK;

}
static void init_inactive_cs(void)
{
    SPI1_MCR |= SPI_MCR_PCSIS(1<<0);
}
static void init_inactive_clock(void)
{
    // must be low
}
static void init_frame_size(void)
{
    SPI1_CTAR0 &= ~SPI_CTAR_FMSZ_MASK;
    SPI1_CTAR0 |= SPI_CTAR_FMSZ(7);
}
static void init_clock_phase(void)
{
    // defalut = capture data on rising edge
}
static void init_baudrate(void)
{
	SPI1_CTAR0 |= (SPI_CTAR_PBR(3) | SPI_CTAR_BR(10));
	SPI1_CTAR1 |= (SPI_CTAR_PBR(3) | SPI_CTAR_BR(10));
	// default = sys clock/2/2 = 48/4 = 12M
}
static void init_msb_first(void)
{
    // default is msb first
}
void hal_spi_transfe_start(void)
{
    SPI1_MCR &= ~SPI_MCR_HALT_MASK;
}
void hal_spi_transfe_stop(void)
{
    SPI1_MCR |= SPI_MCR_HALT_MASK;
}
uint8_t hal_spi_transfer_one_byte(uint8_t v, uint8_t end)
{
    if(end)
        SPI1_PUSHR = //SPI_PUSHR_CONT_MASK |
                     SPI_PUSHR_EOQ_MASK  |
                     SPI_PUSHR_PCS(1<<0) |
                     (v);
    else
        SPI1_PUSHR = SPI_PUSHR_CONT_MASK |
                     SPI_PUSHR_PCS(1<<0) |
                     (v);

    while((SPI1_SR & SPI_SR_TCF_MASK)==0)
        ;
    SPI1_SR |= SPI_SR_TCF_MASK;
    return SPI1_POPR&0xff;
}
void hal_spi_init(void)
{
    init_clock();
    init_io();
    init_set_master();
    init_fifo();
    init_inactive_cs();
    init_inactive_clock();
    init_frame_size();
    init_clock_phase();
    init_msb_first();
    init_baudrate();
}
