/****************************************************************************************************/
/**
\file       dma.h
\brief      
\author     Freescale Semiconductor
\date	    October 2012    
*/
/****************************************************************************************************/
/* Services performed by FREESCALE in this matter are performed AS IS and without any warranty.  	*/
/* CUSTOMER retains the final decision relative to the total design and functionality of the end 	*/
/* product.                                                                                      	*/
/* FREESCALE neither guarantees nor will be held liable by CUSTOMER for the success of this project.*/
/*                                                                                                  */
/* FREESCALE disclaims all warranties, express, implied or statutory including, but not limited to, */
/* implied warranty of merchantability or fitness for a particular purpose on any hardware,         */
/* software ore advise supplied to the project by FREESCALE, and or any product resulting from      */
/* FREESCALE services.                                                                              */
/* In no event shall FREESCALE be liable for incidental or consequential damages arising out of     */
/* this agreement. CUSTOMER agrees to hold FREESCALE harmless against any and all claims demands or */
/* actions by anyone on account of any damage,or injury, whether commercial, contractual, or        */
/* tortuous, rising directly or indirectly as a result of the advise or assistance supplied CUSTOMER*/ 
/* in connectionwith product, services or goods supplied under this Agreement.                      */
/*                                                                                                  */
/****************************************************************************************************/

/*****************************************************************************************************
* Module definition against multiple inclusion
*****************************************************************************************************/

/*****************************************************************************************************
* Include files
*****************************************************************************************************/
#include "common.h"
/*****************************************************************************************************
* Declaration of project wide TYPES
*****************************************************************************************************/

/*****************************************************************************************************
* Definition of project wide MACROS / #DEFINE-CONSTANTS 
*****************************************************************************************************/
#define SPI0_DMA_RX_CHANNEL             14
#define SPI0_DMA_TX_CHANNEL             15

#define SPI1_DMA_CHANNEL                16
#define SPI2_DMA_CHANNEL                17


#define ALWAYS_ENABLED_DMA_CH54         54      /* for memory to memory data transfer */
#define SPI0_DMA_TCD_TX_PGM_CHANNEL     1       /* send program buffer on SPI TX channel */
#define SPI0_DMA_TCD_TX_CHANNEL         3       /* send read buffer on SPI TX channel */
#define SPI0_DMA_TCD_RX_CHANNEL         4       /* receive data from SPI flash */

#define SPI1_DMA_TCD_TX_CHANNEL         7       /* send read buffer on SPI TX channel */
#define SPI1_DMA_TCD_RX_CHANNEL         6       /* receive data from SPI flash */

#define SPI2_DMA_TCD_TX_CHANNEL         5       /* send read buffer on SPI TX channel */
#define SPI2_DMA_TCD_RX_CHANNEL         2       /* receive data from SPI flash */


#define ONE_BYTE                        1
#define TWO_BYTES                       2
#define FOUR_BYTES                      4
#define SIXTEEN_BYTES                   16
#define THIRTY_TWO_BYTES                32
#define BUFFER_LENGTH1                   0x20U
#define BUFFER_LENGTH_16BIT             1024
#define BUFFER_LENGTH_8BIT              512
#define DESTINATION                     0x20000000

#define SPI_CHANNEL0                    0
#define SPI_CHANNEL1                    1
#define SPI_CHANNEL2                    2 


#define I2C1_SLAVE_ADDRESS  0x1C              //if LSB is 1----- master is reading from the slave, (master recieve)----- if LSB is 0, master is writing to slave (master transmit)
#define I2C2_SLAVE_ADDRESS  0x1A

#define I2C0_SRC_NUM       18                  //I2C Source Number -- Refer to the reference manual (3.9.3.1 Table 3-24 DMA Request Sources)
#define I2C1_SRC_NUM       19
#define I2C2_SRC_NUM       19


#define UART0_RX_SRC_NUM       2                  //UART source numbers -- Refer to the reference manual (3.9.3.1 Table 3-24 DMA Request Sources)
#define UART0_TX_SRC_NUM       3

#define UART1_RX_SRC_NUM       4                  //UART source numbers -- Refer to the reference manual (3.9.3.1 Table 3-24 DMA Request Sources)
#define UART1_TX_SRC_NUM       5
#define ONE_BYTE                        1
#define TWO_BYTES                       2
#define FOUR_BYTES                      4
#define SIXTEEN_BYTES                   16
#define THIRTY_TWO_BYTES                32
#define BUFFER_LENGTH1                   0x20U
#define BUFFER_LENGTH_16BIT             1024
#define BUFFER_LENGTH_8BIT              512

#define I2CDATA_SIZE                    512

#define MASTER_MODE 1
#define SLAVE_MODE  0

#define UART0_RX_SRC_NUM       2                  //UART source numbers -- Refer to the reference manual (3.9.3.1 Table 3-24 DMA Request Sources)
#define UART0_TX_SRC_NUM       3

#define UART1_RX_SRC_NUM       4                  //UART source numbers -- Refer to the reference manual (3.9.3.1 Table 3-24 DMA Request Sources)
#define UART1_TX_SRC_NUM       5

#define UART_DATA_SIZE 512 //64
/*****************************************************************************************************
* Definition of project wide VARIABLES
*****************************************************************************************************/
extern uint16_t transmit_buffer_16bit[BUFFER_LENGTH_16BIT/2];
extern uint8_t transmit_buffer_8bit[BUFFER_LENGTH_8BIT];

extern uint16_t spi0_rx_buffer[BUFFER_LENGTH_16BIT/2];
extern uint16_t spi1_rx_buffer[BUFFER_LENGTH_16BIT/2];
extern uint16_t spi2_rx_buffer[BUFFER_LENGTH_16BIT/2];

extern uint8_t i2c1_rx_buffer[BUFFER_LENGTH_8BIT];
extern uint8_t i2c2_rx_buffer[BUFFER_LENGTH_8BIT];
      
extern uint8_t uart1_rx_buffer[BUFFER_LENGTH_8BIT];
extern uint8_t uart2_rx_buffer[BUFFER_LENGTH_8BIT];
extern uint8_t uart3_rx_buffer[BUFFER_LENGTH_8BIT];
extern uint8_t uart5_rx_buffer[BUFFER_LENGTH_8BIT];


/*****************************************************************************************************
* Declaration of project wide FUNCTIONS
*****************************************************************************************************/

/*****************************************************************************************************
* Declaration of module wide FUNCTIONs - NOT for use in other modules
*****************************************************************************************************/
extern void i2c_demo(void);
extern void Init_I2C(uint8 mstr_slave, I2C_MemMapPtr I2C_Channel);
extern void init_arrays(void);
extern void DMA_RX_Init(DMA_MemMapPtr DMA_Channel,  uint8_t dma_source, int index, uint32_t source_addr, uint32_t dest_addr, uint32_t num_bytes, uint32_t bytes_per_xfer);
extern void DMA_TX_Init(DMA_MemMapPtr DMA_Channel,  uint8_t dma_source, int index, uint32_t source_addr, uint32_t dest_addr, uint32_t num_bytes, uint32_t bytes_per_xfer);



