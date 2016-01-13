/*
 * dma.h
 *
 *  Created on: Dec 5, 2015
 *      Author: Norbi
 */

#ifndef DMA_H_
#define DMA_H_

#define I2C1_SOURCE		23
#define UART0_SOURCE	2
#define ADC_SOURCE		40
#define CHANNEL_0	0
#define CHANNEL_1	1
#define CHANNEL_2	2

void init_DMA();
 
void DMA_starttransfer(DMA_MemMapPtr DMA_Channel, uint8_t index,
		uint32_t num_bytes);
void DMA_UART_init(DMA_MemMapPtr DMA_Channel, uint8_t dma_source, int index,
		uint32_t source_addr, uint32_t dest_addr, uint32_t num_bytes,uint32_t bytes_per_xfer);
void DMA_ADC_init(DMA_MemMapPtr DMA_Channel, uint8_t dma_source, int index,
		uint32_t source_addr, uint32_t dest_addr, uint32_t num_bytes,uint32_t bytes_per_xfer);

char uart_data;
long adc_data[15];

#endif /* DMA_H_ */
