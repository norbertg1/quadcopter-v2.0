/*
 * dma.c
 *
 *  Created on: Dec 5, 2015
 *      Author: Norbi
 */
#include "init.h"

char uart_data;
long adc_data[15];

//****************************************************************************
//**** DMA Receiver Routine ****************************************** ******
//***************************************************************************

void DMA_Init(DMA_MemMapPtr DMA_Channel, uint8_t dma_source, int index,
		uint32_t source_addr, uint32_t dest_addr, uint32_t num_bytes,uint32_t bytes_per_xfer)
{
	/* num_bytes is total bytes */
	/* setup depends on how many bytes per transfer */

	SIM_SCGC6 |= SIM_SCGC6_DMAMUX_MASK;
	SIM_SCGC7 |= SIM_SCGC7_DMA_MASK;

	uint32_t ssize_dsize_attr;

	/* setup ssize and dsize parameter for TCD ATTR register based on transfer size assigned above */
	/* 0:32bit; 1:8-bit, 2:16-bit  */
	switch (bytes_per_xfer) {
	case 8:
	default:
		ssize_dsize_attr = 1;
		break; /* 8-bit */
	case 16:
		ssize_dsize_attr = 2;
		break; /* 16-bit */
	case 32:
		ssize_dsize_attr = 0;
		break; /* 32-bit */

	}

	DMA_SAR_REG(DMA_Channel, index)= source_addr;              // Source address 
	DMA_DCR_REG(DMA_Channel, index)|= 0x0000; // Source address increments 0 bytes (uint32)
	DMA_DAR_REG(DMA_Channel, index)|= dest_addr;           // Destination address 
	DMA_DCR_REG(DMA_Channel, index)|= ssize_dsize_attr << DMA_DCR_DINC_SHIFT; // Destination offset increments 
	DMA_DCR_REG(DMA_Channel, index)|= DMA_DCR_ERQ_MASK|DMA_DCR_CS_MASK;
	I2C1_C1 |= I2C_C1_DMAEN_MASK|I2C_C1_IICIE_MASK;
	//DMA_DLAST_SGA_REG(DMA_Channel, index) = -num_bytes; // Destination address shift, go to back to beginning of buffer
	DMA_DSR_BCR_REG(DMA_Channel, index)= num_bytes; //Stop the transfer when all bytes is transfered
	//DMA_BITER_ELINKNO_REG(DMA_Channel, index)  = citer_biter;                                        // Major loop iterations
	//DMA_CITER_ELINKNO_REG(DMA_Channel, index)  = citer_biter;                                        // Set current interation count  
	DMA_DCR_REG(DMA_Channel, index)|= ((ssize_dsize_attr<<DMA_DCR_DSIZE_SHIFT)|(ssize_dsize_attr<<DMA_DCR_SSIZE_SHIFT)); // Source a destination size 0:32bit; 1:8-bit, 2:16-bit
	//DMA_CSR_REG(DMA_Channel, index)            = DMA_CSR_INTMAJOR_MASK | DMA_CSR_DREQ_MASK;          // Enable end of loop DMA interrupt, disable request at end of major iteration              

	DMAMUX_CHCFG_REG(DMAMUX0_BASE_PTR,index)= (DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(dma_source)); // DMA source DMA Mux to tie source to DMA channel

}

void DMA_UART_init(DMA_MemMapPtr DMA_Channel, uint8_t dma_source, int index,
		uint32_t source_addr, uint32_t dest_addr, uint32_t num_bytes,uint32_t bytes_per_xfer) {
	
	DMA_Init(DMA_Channel, dma_source, index,source_addr,dest_addr, num_bytes,bytes_per_xfer );
	UART0_C5 |= UART0_C5_RDMAE_MASK;	//Enable UART0 to generate DMA request
		}
void DMA_ADC_init(DMA_MemMapPtr DMA_Channel, uint8_t dma_source, int index,
		uint32_t source_addr, uint32_t dest_addr, uint32_t num_bytes,uint32_t bytes_per_xfer) {
	
	DMA_Init(DMA_Channel, dma_source, index,source_addr,dest_addr, num_bytes,bytes_per_xfer );
	UART0_C5 |= UART0_C5_RDMAE_MASK;	//Enable UART0 to generate DMA request
		}
/*void init_DMA() {
	DMA_RX_Init(DMA_BASE_PTR, I2C1_SOURCE, CHANNEL_0,
			(uint32_t) (&(I2C1_D )),(uint32_t)(&(MPU_6050_buffer[0])), 12,8 ); //DMA SLAVE

		}*/ //2015.12.10
void DMA_starttransfer(DMA_MemMapPtr DMA_Channel, uint8_t index, uint32_t num_bytes)
{
	DMA_DSR_REG(DMA_Channel, index) = 0;
	//DMA_DSR_BCR0 |= DMA_DSR_BCR_DONE_MASK;
	DMA_DAR0 = (uint32_t)(&MPU_6050_buffer[0]);
	DMA_DSR_BCR0|= num_bytes;
	DMA_DSR_BCR0|= num_bytes;
	//DMA_DSR_BCR0 |= DMA_DSR_BCR_DONE_MASK;
	DMA_DCR_REG(DMA_Channel, index)|= DMA_DCR_START_MASK;
}


