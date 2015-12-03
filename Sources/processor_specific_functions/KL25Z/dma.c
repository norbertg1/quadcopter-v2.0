/*
 * dma.c
 *
 *  Created on: Jun 21, 2014
 *      Author: B21665
 */
#include "init.h"


/** DMA Initialization **********************************************/
void dma_init(void)
{ 
	int j;

	//enable DMA clocks
	SIM_SCGC6 |= SIM_SCGC6_DMAMUX_MASK; 
	SIM_SCGC7 |= SIM_SCGC7_DMA_MASK;

	// Clear Destination memory  
	for( j=0; j < BUFF_SIZE; j=j+4)
		*((uint32_t *)(DESTINATION_ADDRESS+j)) = 0xAA;

	// Config DMA Mux for UART0 operation
	// Disable DMA Mux channel first
	DMAMUX0_CHCFG0 = 0x00;

	// Clear pending errors and/or the done bit 
	if (((DMA_DSR_BCR0 & DMA_DSR_BCR_DONE_MASK) == DMA_DSR_BCR_DONE_MASK)
			| ((DMA_DSR_BCR0 & DMA_DSR_BCR_BES_MASK) == DMA_DSR_BCR_BES_MASK)
			| ((DMA_DSR_BCR0 & DMA_DSR_BCR_BED_MASK) == DMA_DSR_BCR_BED_MASK)
			| ((DMA_DSR_BCR0 & DMA_DSR_BCR_CE_MASK) == DMA_DSR_BCR_CE_MASK))
		DMA_DSR_BCR0 |= DMA_DSR_BCR_DONE_MASK;

	// Set Source Address (this is the UART0_D register
	DMA_SAR0 = (uint32_t)&UART0_D;

	// Set BCR to know how many bytes to transfer
	DMA_DSR_BCR0 = DMA_DSR_BCR_BCR(4);

	// Clear Source size and Destination size fields.  
	DMA_DCR0 &= ~(DMA_DCR_SSIZE_MASK 
			| DMA_DCR_DSIZE_MASK
	);

	// Set DMA 
	DMA_DCR0 |= (DMA_DCR_EINT_MASK		// Int. enable... used if FREEDOM macro is set
			| DMA_DCR_ERQ_MASK			//Enable Peripheral request
			| DMA_DCR_CS_MASK			//Single read/write per request
			| DMA_DCR_EADREQ_MASK		//Enable Async. DMA Requests
			| DMA_DCR_SSIZE(1)			//Source size is 8-bit
			| DMA_DCR_DINC_MASK			//Destination address increments
			| DMA_DCR_DSIZE(1)			//Destination size is 8-bit
			| DMA_DCR_DMOD(2)			//32-bytes circular buffer enabled
			| DMA_DCR_D_REQ_MASK		//DMA request is cleared
	);

	// Set destination address
	DMA_DAR0 = DESTINATION_ADDRESS;

	// Enables the DMA channel and select the DMA Channel Source  
	DMAMUX0_CHCFG0 |= DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(2); 
	//Enable channel 0, Request source = UART 0 Receive
}

/*
/	Handles interrupt request of DMA channel 0 at the end of the data transfer
*/
void DMA0_IRQHandler(void)
{
	int j;
	int k;
	uint32_t word;
	DMA_DSR_BCR0 |= DMA_DSR_BCR_DONE_MASK; // Clear interrupt
//	put("Successful DMA transfer of 4 bytes from UART0_D to DESTINATION_ADDRESS 0x20001000 \n\r");
//	put("The following characters were successfully transferred \n\r");
	for(j = DESTINATION_ADDRESS; j < DESTINATION_END; j += 4 ){
		word = (*((uint32_t *)j));
		for(k = 0; k < 4; k += 1){
//			out_char(word);
			word = word >> 8;
		}
	}
	//put("\n\r DMA request disabled \n\r");
}
