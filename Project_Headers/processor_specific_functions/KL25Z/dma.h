/*
 * dma.h
 *
 *  Created on: Jun 21, 2014
 *      Author: B21665
 */

#ifndef DMA_H_
#define DMA_H_

#include "main.h"

// DMA Definitions
#define  DESTINATION_ADDRESS 0x20001000
#define  BUFF_SIZE 4
#define	 DESTINATION_END DESTINATION_ADDRESS + BUFF_SIZE


void dma_init(void);

#endif /* DMA_H_ */
