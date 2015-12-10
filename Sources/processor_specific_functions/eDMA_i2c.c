/*
 * File:		edma_i2c.c
 * Purpose:		Main process. Initalizes the master and slave and DMA access.
 *
 * I2C0 ----> Configured as the Master
 * I2C1 ----> Configured as the Slave
 * I2C2 ----> Configured as the Master 
 * Connections:
 * I2C0_SCL ----> PTB0 --------> (pin 2, J18 Header)           
 * I2C0_SDA ----> PTB1 --------> (pin 4, J18 Header)
 *
 * I2C1_SCL ----> PTC10 -------> (A7 on the elevator) 
 * I2C1_SDA ----> PTC11 -------> (A8 on the elevator)
 *
 * I2C2_SCL ----> PTA14 --------> A42/B35
 * I2C2_SDA ----> PTA13 --------> A33
 * I2C2_SDA ----> PTA11 --------> no connection
 * I2C2_SCL ----> PTA12 --------> A34
 *
 * External pull up resistors needed from SCL0 & SCL1 to Vdd.
 * External pull up resistors needed from SDA0 & SDA1 to Vdd.
 *
 * DMA Channel 0 ----> I2C0 (Master)
 * DMA Channel 1 ----> I2C1 (Slave)
 * DMA Channel 5 ----> I2C2 (Master)
 * DMA Channel 6 ----> memory 
 *  
 * DMA Channel Request Source for I2C0 is 18. 
 * DMA Channel Request Source for I2C1 is 19.
 * DMA Channel Request Source for I2C2 is 19. 
*/

#include "common.h"
#include "eDMA.h"




uint8_t transmit_buffer_8bit[BUFFER_LENGTH_8BIT];
uint16_t transmit_buffer_16bit[BUFFER_LENGTH_8BIT];


uint8_t i2c1_rx_buffer[BUFFER_LENGTH_8BIT];
uint8_t i2c2_rx_buffer[BUFFER_LENGTH_8BIT];


void i2c_demo(void);
void Init_I2C(uint8 mstr_slave, I2C_MemMapPtr I2C_Channel);
void DMA_RX_Init(DMA_MemMapPtr DMA_Channel,  uint8_t dma_source, int index, uint32_t source_addr, uint32_t dest_addr, uint32_t num_bytes, uint32_t bytes_per_xfer);
void DMA_TX_Init(DMA_MemMapPtr DMA_Channel,  uint8_t dma_source, int index, uint32_t source_addr, uint32_t dest_addr, uint32_t num_bytes, uint32_t bytes_per_xfer);
void init_arrays(void);

  
  //****************************************************************************
  //**** DMA Receiver Routine ****************************************** ******
  //***************************************************************************

void DMA_RX_Init(DMA_MemMapPtr DMA_Channel, uint8_t dma_source, int index, uint32_t source_addr, uint32_t dest_addr, uint32_t num_bytes, uint32_t bytes_per_xfer){
  /* num_bytes is total bytes */
  /* setup depends on how many bytes per transfer */
  uint32_t      citer_biter = num_bytes / bytes_per_xfer; 
  uint32_t      ssize_dsize_attr;
  

  
  /* setup ssize and dsize parameter for TCD ATTR register based on transfer size assigned above */
  /* 0:8bit; 1:16-bit, 2:32-bit, 5:16-byte, 6:32-byte */
  switch(bytes_per_xfer)
  {
    case 1: default:
      ssize_dsize_attr = 0;  break;       /* 8-bit */
    case 2:
      ssize_dsize_attr = 1;  break;       /* 16-bit */
    case 4:
      ssize_dsize_attr = 2;  break;       /* 32-bit */
    case 16:
      ssize_dsize_attr = 5;  break;       /* 16-byte */
    case 32:
      ssize_dsize_attr = 6;  break;       /* 32-byte */
  }    


  
  DMA_SADDR_REG(DMA_Channel, index)          = source_addr;                                        // Source address 
  DMA_SOFF_REG(DMA_Channel, index)           = 0x0000;                                             // Source address increments 0 bytes (uint32)
  DMA_SLAST_REG(DMA_Channel, index)          = 0;                                                  // After the major loop ends the source address decrements the size of the buffer
  DMA_DADDR_REG(DMA_Channel, index)          = dest_addr;                                          // Destination address 
  DMA_DOFF_REG(DMA_Channel, index)           = bytes_per_xfer;                                     // Destination offset increments 
  DMA_DLAST_SGA_REG(DMA_Channel, index)      = -num_bytes;                                         // Destination address shift, go to back to beginning of buffer
  DMA_NBYTES_MLNO_REG(DMA_Channel, index)    = bytes_per_xfer;                                     // The minor loop moves by bytes per transfer
  DMA_BITER_ELINKNO_REG(DMA_Channel, index)  = citer_biter;                                        // Major loop iterations
  DMA_CITER_ELINKNO_REG(DMA_Channel, index)  = citer_biter;                                        // Set current interation count  
  DMA_ATTR_REG(DMA_Channel, index)           = (DMA_ATTR_SSIZE(ssize_dsize_attr)|DMA_ATTR_DSIZE(ssize_dsize_attr));   // Source a destination size 0:8bit; 1:16-bit, 2:32-bit, 5:16-byte, 6:32-byte
  DMA_CSR_REG(DMA_Channel, index)            = DMA_CSR_INTMAJOR_MASK | DMA_CSR_DREQ_MASK;          // Enable end of loop DMA interrupt, disable request at end of major iteration              
  
  DMAMUX_CHCFG_REG(DMAMUX_BASE_PTR,index)    = (DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(dma_source));     // DMA source DMA Mux to tie source to DMA channel


}

  
  //****************************************************************************
  //**** DMA Transmitter *******************************************************
  //****************************************************************************

void DMA_TX_Init(DMA_MemMapPtr DMA_Channel, uint8_t dma_source, int index, uint32_t source_addr, uint32_t dest_addr, uint32_t num_bytes, uint32_t bytes_per_xfer) {

/* num_bytes is total bytes */
  /* setup depends on how many bytes per transfer */
  uint32_t      citer_biter = num_bytes / bytes_per_xfer;
  uint32_t      ssize_dsize_attr;
  
  
  switch(bytes_per_xfer)
  {
    case 1: default:
      ssize_dsize_attr = 0;  break;       /* 8-bit */
    case 2:
      ssize_dsize_attr = 1;  break;       /* 16-bit */
    case 4:
      ssize_dsize_attr = 2;  break;       /* 32-bit */
    case 16:
      ssize_dsize_attr = 5;  break;       /* 16-byte */
    case 32:
      ssize_dsize_attr = 6;  break;       /* 32-byte */
  }
      

  DMAMUX_CHCFG_REG(DMAMUX_BASE_PTR,index)    = (DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(dma_source));     // DMA source DMA Mux
  DMA_SADDR_REG(DMA_Channel, index)          = source_addr;                                        // Source address 
  DMA_SOFF_REG(DMA_Channel, index)           = bytes_per_xfer;                                     // Source address increments by number of bytes per transfer
  DMA_SLAST_REG(DMA_Channel, index)          = -num_bytes;                                         // After the major loop ends, reset pointer to beginning of buffer
  DMA_DADDR_REG(DMA_Channel, index)          = dest_addr ;                                         // Destination address 
  DMA_DOFF_REG(DMA_Channel, index)           = 0x0;                                                // Destination offset increments 0 bytes (uint32)
  DMA_DLAST_SGA_REG(DMA_Channel, index)       = 0;                                                 // Destination address shift
  DMA_NBYTES_MLNO_REG(DMA_Channel, index)    = bytes_per_xfer;                                     // The minor loop moves 32 bytes per transfer
  DMA_BITER_ELINKNO_REG(DMA_Channel, index)  = citer_biter;                                        // Major loop iterations
  DMA_CITER_ELINKNO_REG(DMA_Channel, index)  = citer_biter;                                        // Set current interation count  
  DMA_ATTR_REG(DMA_Channel, index)           = (DMA_ATTR_SSIZE(ssize_dsize_attr) | DMA_ATTR_DSIZE(ssize_dsize_attr));   // Source a destination size 0:8bit; 1:16-bit, 2:32-bit, 5:16-byte, 6:32-byte
 
  DMA_CSR_REG(DMA_Channel, index)            = DMA_CSR_INTMAJOR_MASK | DMA_CSR_DREQ_MASK;          // Enable end of loop DMA interrupt; clear ERQ @ end of major iteration               
 


}

/************************************/
/********i2c_demo**********************/
//Using I2C0 as master and I2C1 as slave

#define I2C01 //Initalizes I2C0 as master and I2C1 as slave
//#define I2C20 //Initializes I2C2 as master and I2C0 as slave
//#define I2C02 //Initializes I2C0 as master and I2C2 as slave
//#define I2C2  //Initializes I2C2 as master and slave
  void i2c_demo(void){
  int size_of_data_i2c = 512;
  #ifdef I2C01           
         // DMA_Slave();
            DMA_RX_Init(DMA_BASE_PTR,  I2C1_SRC_NUM, 1,(uint32_t)(&(I2C1_D)),(uint32_t)(&(i2c1_rx_buffer[0])), size_of_data_i2c, ONE_BYTE ); //DMA SLAVE
            Init_I2C(SLAVE_MODE, I2C1_BASE_PTR);   // Initialize as a slave
          //DMA_Master();
            DMA_TX_Init(DMA_BASE_PTR,  I2C0_SRC_NUM, 0,(uint32_t)(&(transmit_buffer_8bit)), (uint32_t)(&(I2C0_D)), size_of_data_i2c, ONE_BYTE ); //DMA MASTER
            Init_I2C(MASTER_MODE, I2C0_BASE_PTR);   // Initialize as a master
            DMA_ERQ |= (DMA_ERQ_ERQ0_MASK | DMA_ERQ_ERQ1_MASK);
            I2C0_C1 |= I2C_C1_MST_MASK;
            I2C0_D = (I2C1_SLAVE_ADDRESS) << 1;  // <---- THIS SHOULD BE CHANGED TO YOUR SLAVE ADDRESS!!!!!!
#endif
            
#ifdef I2C02          
      
            DMA_RX_Init(DMA_BASE_PTR,  I2C2_SRC_NUM, 1,(uint32_t)(&(I2C2_D)), (uint32_t)(&(i2c2_rx_buffer)), size_of_data_i2c, ONE_BYTE); //DMA MASTER
            Init_I2C(0, I2C2_BASE_PTR);
            DMA_TX_Init(DMA_BASE_PTR,  I2C0_SRC_NUM, 0,(uint32_t)(&(transmit_buffer_8bit)), (uint32_t)(&(I2C0_D)), size_of_data_i2c, ONE_BYTE); //DMA MASTER
            Init_I2C(1, I2C0_BASE_PTR);   // Initialize as a master
            DMA_ERQ |= (DMA_ERQ_ERQ0_MASK | DMA_ERQ_ERQ1_MASK);
            I2C0_C1 |= I2C_C1_MST_MASK;
            I2C0_D = (I2C1_SLAVE_ADDRESS) << 1;
#endif
            
#ifdef I2C20   
           DMA_RX_Init(DMA_BASE_PTR,  I2C0_SRC_NUM, 1,(uint32_t)(&(I2C0_D)), (uint32_t)(&(i2c2_rx_buffer)), size_of_data_i2c, ONE_BYTE); //DMA MASTER
           Init_I2C(0, I2C0_BASE_PTR);
           DMA_TX_Init(DMA_BASE_PTR,  I2C2_SRC_NUM, 0,(uint32_t)(&(transmit_buffer_8bit)), (uint32_t)(&(I2C2_D)), size_of_data_i2c, ONE_BYTE ); //DMA MASTER
           Init_I2C(1, I2C2_BASE_PTR);   // Initialize as a master
           DMA_ERQ |= (DMA_ERQ_ERQ0_MASK | DMA_ERQ_ERQ1_MASK);
           I2C2_C1 |= I2C_C1_MST_MASK;
           I2C2_D = (I2C1_SLAVE_ADDRESS) << 1;  // <---- THIS SHOULD BE CHANGED TO YOUR SLAVE ADDRESS!!!!!!
           
#endif           
     
#ifdef I2C2
           DMA_RX_Init(DMA_BASE_PTR,  I2C2_SRC_NUM, 6,(uint32_t)(&(I2C2_D)), (uint32_t)(&(i2c2_rx_buffer)), size_of_data_i2c, ONE_BYTE); //DMA MASTER
           DMA_TX_Init(DMA_BASE_PTR,  I2C2_SRC_NUM, 5,(uint32_t)(&(transmit_buffer_8bit)), (uint32_t)(&(I2C2_D)), size_of_data_i2c, ONE_BYTE ); //DMA MASTER
           Init_I2C(1, I2C2_BASE_PTR);   // Initialize as a master
           DMA_ERQ |= (DMA_ERQ_ERQ5_MASK | DMA_ERQ_ERQ6_MASK);
           I2C2_C1 |= I2C_C1_MST_MASK;
           I2C2_D = (I2C2_SLAVE_ADDRESS) << 1;  // <---- THIS SHOULD BE CHANGED TO YOUR SLAVE ADDRESS!!!!!!  
           
#endif
  
  }  
  
	
//Initializes I2C as master and slave
void Init_I2C(uint8 mstr_slave, I2C_MemMapPtr I2C_Channel)
{
    //Master Initialization
    if (mstr_slave == MASTER_MODE)
    {
      if(I2C_Channel == I2C0_BASE_PTR){
        PORTB_PCR0 = PORT_PCR_MUX(2); //I2C0_SDA
        PORTB_PCR1 = PORT_PCR_MUX(2); //I2C0_SCL
      }
      
      
      if(I2C_Channel == I2C2_BASE_PTR){
      PORTA_PCR13 = PORT_PCR_MUX(5); //I2C2_SDA
      PORTA_PCR14 = PORT_PCR_MUX(5); //I2C2_SCL
      }
            
        I2C_F_REG(I2C_Channel)  = 0x15;   // 1MBit/sec
        I2C_C1_REG(I2C_Channel) = I2C_C1_RSTA_MASK;
        I2C_C1_REG(I2C_Channel) |= I2C_C1_IICEN_MASK |    // enable IIC 
                                   I2C_C1_IICIE_MASK |    
                                   I2C_C1_TX_MASK;     
        I2C_C1_REG(I2C_Channel) |= I2C_C1_DMAEN_MASK;
        
        printf("\nI2C0 initalized as Master \n");  
    }
    else
    {
      //Slave Initialization
      if(I2C_Channel == I2C0_BASE_PTR){
        PORTB_PCR0 = PORT_PCR_MUX(2) | PORT_PCR_ODE_MASK;
        PORTB_PCR1 = PORT_PCR_MUX(2) | PORT_PCR_ODE_MASK;
      }
      
       if(I2C_Channel == I2C2_BASE_PTR){
        PORTA_PCR13 = PORT_PCR_MUX(5) | PORT_PCR_ODE_MASK; //I2C2_SDA
        PORTA_PCR14 = PORT_PCR_MUX(5) | PORT_PCR_ODE_MASK; //I2C2_SCL    
        
        
      }
      if(I2C_Channel == I2C1_BASE_PTR)
       {
        PORTC_PCR10 = PORT_PCR_MUX(2) | PORT_PCR_ODE_MASK;
        PORTC_PCR11 = PORT_PCR_MUX(2) | PORT_PCR_ODE_MASK;
       } 
        I2C_C1_REG(I2C_Channel) = 0x00;            // Ensure IIC C1 is in reset state. 
        
        if(I2C_Channel==I2C1_BASE_PTR)
                {I2C_A1_REG(I2C_Channel) = (I2C1_SLAVE_ADDRESS << 1);} //USE I2C1 SLAVE ADDRESS
        else
                {I2C_A1_REG(I2C_Channel) = (I2C1_SLAVE_ADDRESS << 1);} //CHANGE IT TO THE DESIRED SLAVE ADDRESS

	      I2C_C1_REG(I2C_Channel) = (I2C_C1_IICIE_MASK);
        I2C_C1_REG(I2C_Channel) |= I2C_C1_DMAEN_MASK; //enable DMA
        
        
        I2C_C1_REG(I2C_Channel) |= I2C_C1_IICEN_MASK;
        printf("\nI2C1 initalized as Slave \n");  
    }
}
	
	

void init_arrays(void){
   volatile int i;
   
    /* Populate initial tx buffers */
   for(int k = 0; k < BUFFER_LENGTH_8BIT; k++){
        transmit_buffer_8bit[k] =  k;
   
   } 
   
   for(i = 0; i < BUFFER_LENGTH_16BIT/2; i++){
        transmit_buffer_16bit[i] = 0x0301 + (uint16)i;
    }
    
   
   /**************************************************/
   
   //populate initial rx buffers for I2C
   for (int k=0; k<BUFFER_LENGTH_8BIT; k++){
              i2c1_rx_buffer[k] = 0xFF;
            }
     for(int k=0; k<BUFFER_LENGTH_8BIT; k++){
          i2c2_rx_buffer[k] = 0xFF;
     
     }
   
      
   

}
