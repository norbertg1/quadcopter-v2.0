/*
 * ADC.c
 *
 *  Created on: Dec 21, 2014
 *      Author: Norbi
 */


/*
 * File:		a16dc.c
 * Purpose:		Simple Driver or API for AdC16/PGA
 * This file contains the following two functions:
 *  1. ADC_Cal: calibrates the ADC
 *  2. ADC_Config_Alt: Simply configures an ADC per a structure
 *
 * Typical usage: Fill the structure with the ADC and PGA register contents 
 * needed for the ADC usage.

 * a) Call the ADC_Config_Alt function to configure an ADC, (ADC0 or ADC1)
 * b) Call the ADC_Cal function to calibrate that ADC
 * c) Call the ADC_Config_Alt function again to restore desired configuation
 *    after a calibration
 *
 *
 */

#include "init.h"


/******************************************************************************
Function 1. Name	AUTO CAL ROUTINE   

Parameters		ADC module pointer points to adc0 or adc1 register map 
                         base address.
Returns			Zero indicates success.
Notes         		Calibrates the ADC16. Required to meet specifications 
                        after reset and before a conversion is initiated.
******************************************************************************/
uint8_t ADC_Cal(ADC_MemMapPtr adcmap)
{

  unsigned short cal_var;
  
  ADC_SC2_REG(adcmap) &=  ~ADC_SC2_ADTRG_MASK ; // Enable Software Conversion Trigger for Calibration Process    - ADC0_SC2 = ADC0_SC2 | ADC_SC2_ADTRGW(0);   
  ADC_SC3_REG(adcmap) &= ( ~ADC_SC3_ADCO_MASK & ~ADC_SC3_AVGS_MASK ); // set single conversion, clear avgs bitfield for next writing
  ADC_SC3_REG(adcmap) |= ( ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(AVGS_32) );  // Turn averaging ON and set at max value ( 32 )
  
  
  ADC_SC3_REG(adcmap) |= ADC_SC3_CAL_MASK ;      // Start CAL
  while ( (ADC_SC1_REG(adcmap,Ach) & ADC_SC1_COCO_MASK ) == COCO_NOT ); // Wait calibration end
  	
  if ((ADC_SC3_REG(adcmap)& ADC_SC3_CALF_MASK) == CALF_FAIL )
  {  
   return(1);    // Check for Calibration fail error and return 
  }
  // Calculate plus-side calibration
  cal_var = 0x00;
  
  cal_var =  ADC_CLP0_REG(adcmap); 
  cal_var += ADC_CLP1_REG(adcmap);
  cal_var += ADC_CLP2_REG(adcmap);
  cal_var += ADC_CLP3_REG(adcmap);
  cal_var += ADC_CLP4_REG(adcmap);
  cal_var += ADC_CLPS_REG(adcmap);

  cal_var = cal_var/2;
  cal_var |= 0x8000; // Set MSB

  ADC_PG_REG(adcmap) = ADC_PG_PG(cal_var);
 

  // Calculate minus-side calibration
  cal_var = 0x00;

  cal_var =  ADC_CLM0_REG(adcmap); 
  cal_var += ADC_CLM1_REG(adcmap);
  cal_var += ADC_CLM2_REG(adcmap);
  cal_var += ADC_CLM3_REG(adcmap);
  cal_var += ADC_CLM4_REG(adcmap);
  cal_var += ADC_CLMS_REG(adcmap);

  cal_var = cal_var/2;

  cal_var |= 0x8000; // Set MSB

  ADC_MG_REG(adcmap) = ADC_MG_MG(cal_var); 
  
  ADC_SC3_REG(adcmap) &= ~ADC_SC3_CAL_MASK ; /* Clear CAL bit */

  return(0);
}




/******************************************************************************
Function   : Name 	ADC_Config_Alt 
Parameters :		the register values to be set in the adc in a structure
Returns		 :	NONE
Notes       	Configures ADC0 or ADC1 depending on adcmap
              Prior to calling this function populate the structure
              elements with the desired ADC configuration.
******************************************************************************/
void ADC_Config(ADC_MemMapPtr adcmap, uint8_t CONFIG1, uint8_t CONFIG2, uint16_t COMPARE1, uint16_t COMPARE2, uint8_t STATUS2, uint8_t STATUS3, uint8_t STATUS1A, uint8_t STATUS1B, uint32_t PGA )
{
  ADC_CFG1_REG(adcmap) = CONFIG1;
  ADC_CFG2_REG(adcmap) = CONFIG2;
  ADC_CV1_REG(adcmap) = COMPARE1; 
  ADC_CV2_REG(adcmap) = COMPARE2;
  ADC_SC2_REG(adcmap) = STATUS2;
  ADC_SC3_REG(adcmap) = STATUS3;
  ADC_SC1_REG(adcmap,Ach) = STATUS1A;       
  ADC_SC1_REG(adcmap,Bch) = STATUS1B;      
}


void ADC_Config_Alt(ADC_MemMapPtr adcmap, tADC_ConfigPtr ADC_CfgPtr)
{
 ADC_CFG1_REG(adcmap) = ADC_CfgPtr->CONFIG1;
 ADC_CFG2_REG(adcmap) = ADC_CfgPtr->CONFIG2;
 ADC_CV1_REG(adcmap)  = ADC_CfgPtr->COMPARE1; 
 ADC_CV2_REG(adcmap)  = ADC_CfgPtr->COMPARE2;
 ADC_SC2_REG(adcmap)  = ADC_CfgPtr->STATUS2;
 ADC_SC3_REG(adcmap)  = ADC_CfgPtr->STATUS3;
// ADC_PGA_REG(adcmap)  = ADC_CfgPtr->PGA;  //not one on P0
 ADC_SC1_REG(adcmap,Ach)= ADC_CfgPtr->STATUS1A;       
 ADC_SC1_REG(adcmap,Bch)= ADC_CfgPtr->STATUS1B;
}


void ADC_Read_Cal(ADC_MemMapPtr adcmap, tADC_Cal_Blk *blk)
{
  blk->OFS  = ADC_OFS_REG(adcmap);
  blk->PG   = ADC_PG_REG(adcmap); 
  blk->MG   = ADC_MG_REG(adcmap); 
  blk->CLPD = ADC_CLPD_REG(adcmap); 
  blk->CLPS = ADC_CLPS_REG(adcmap); 
  blk->CLP4 = ADC_CLP4_REG(adcmap);
  blk->CLP3 = ADC_CLP3_REG(adcmap); 
  blk->CLP2 = ADC_CLP2_REG(adcmap); 
  blk->CLP1 = ADC_CLP1_REG(adcmap);
  blk->CLP0 = ADC_CLP0_REG(adcmap);
  blk->CLMD = ADC_CLMD_REG(adcmap); 
  blk->CLMS = ADC_CLMS_REG(adcmap); 
  blk->CLM4 = ADC_CLM4_REG(adcmap);
  blk->CLM3 = ADC_CLM3_REG(adcmap); 
  blk->CLM2 = ADC_CLM2_REG(adcmap); 
  blk->CLM1 = ADC_CLM1_REG(adcmap);
  blk->CLM0 = ADC_CLM0_REG(adcmap);
  
}

tADC_Config Master_Adc_Config;

void InitADC()
{
	//Enable the clocks to ADCs 
	SIM_SCGC6 |= (SIM_SCGC6_ADC0_MASK);
	SIM_SCGC3 |= (SIM_SCGC3_ADC1_MASK);
    //SIM_SCGC3 |= (SIM_SCGC3_ADC1_MASK);
    
    //Lets calibrate the ADC. 1st setup how the channel will be used.
    
    disable_irq(ADC0_irq_no);   
    //disable_irq(ADC1_irq_no);   
     
    Master_Adc_Config.CONFIG1 = ADLPC_NORMAL 			//No low power mode
								| ADC_CFG1_ADIV(ADIV_1) //divide input by 1
								| ADLSMP_LONG 			//long sample time
								| ADC_CFG1_MODE(MODE_16)//single ended 16-bit conversion
								| ADC_CFG1_ADICLK(ADICLK_BUS);
    
    Master_Adc_Config.CONFIG2 = MUXSEL_ADCA // select the A side of the ADC channel.
								| ADACKEN_DISABLED
								| ADHSC_NORMAL
								| ADC_CFG2_ADLSTS(ADLSTS_20);//Extra long sample Time (20 extra clocks)
    
    
    Master_Adc_Config.COMPARE1 = 00000; // Comparators don't matter for calibration
    Master_Adc_Config.COMPARE1 = 0xFFFF;
    
    Master_Adc_Config.STATUS2  = ADTRG_HW //hardware triggers for calibration
                               | ACFE_DISABLED //disable comparator
                               | ACFGT_GREATER
                               | ACREN_ENABLED
                               | DMAEN_DISABLED //Disable DMA
                               | ADC_SC2_REFSEL(REFSEL_EXT); //External Reference
        
    Master_Adc_Config.STATUS3 = CAL_OFF  
								| ADCO_SINGLE 
								| AVGE_ENABLED
								| ADC_SC3_AVGS(AVGS_32);
								
    Master_Adc_Config.PGA =     0; // Disable the PGA
       
    // Configure ADC as it will be used, but because ADC_SC1_ADCH is 31,
    // the ADC will be inactive.  Channel 31 is just disable function.
    // There really is no channel 31.
    
    Master_Adc_Config.STATUS1A = AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(31);
  //  Master_Adc_Config.STATUS1B = 0;//AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(31);
    
    ADC_Config_Alt(ADC0_BASE_PTR, &Master_Adc_Config);  
    ADC_Config_Alt(ADC1_BASE_PTR, &Master_Adc_Config);
    // config ADC
    
    // Calibrate the ADC in the configuration in which it will be used:
     ADC_Cal(ADC0_BASE_PTR);                   
     ADC_Cal(ADC1_BASE_PTR);                    
     // do the calibration

    // config the ADC again to desired conditions
    // ADC_Config_Alt(ADC1_BASE_PTR, &Master_Adc_Config);
     // Calibrate the ADC in the configuration in which it will be used:
    // ADC_Cal(ADC1_BASE_PTR);                    // do the calibration
    
       
     
     
     Master_Adc_Config.STATUS2  = ACFE_DISABLED //disable comparator
                                | ACFGT_GREATER
                                | ACREN_ENABLED
                                | DMAEN_DISABLED //Disable DMA
                                | ADC_SC2_REFSEL(REFSEL_EXT); //External Reference
    
     Master_Adc_Config.STATUS3 = CAL_OFF  
     							| ADCO_SINGLE 
     							| AVGE_ENABLED
     							| ADC_SC3_AVGS(AVGS_32);
     
     ADC_Config_Alt(ADC0_BASE_PTR, &Master_Adc_Config);
     ADC_Config_Alt(ADC1_BASE_PTR, &Master_Adc_Config);
     //ADC_Config_Alt(ADC1_BASE_PTR, &Master_Adc_Config);     
}

void InitADCpins()
{
	//SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
	//PORTB_PCR0 = PORT_PCR_MUX(0);	//ADC0_SE8
/*	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK;
	PORTE_PCR20 = PORT_PCR_MUX(0);	//ADC0_DP0, ADC0_SE0
	PORTE_PCR21 = PORT_PCR_MUX(0);	//ADC0_DM0, ADC0_SE4a
	PORTE_PCR22 = PORT_PCR_MUX(0);	//ADC0_DP3, ADC0_SE3
	PORTE_PCR23 = PORT_PCR_MUX(0);	//ADC0_DM3, ADC0_SE7a
	PORTE_PCR29 = PORT_PCR_MUX(0);	//ADC0_SE4b
	PORTD_PCR1 = PORT_PCR_MUX(0);	//ADC0_SE5b	KONFLIKT BLUE LED
	PORTD_PCR5 = PORT_PCR_MUX(0);	//ADC0_SE6b
	PORTD_PCR6 = PORT_PCR_MUX(0);	//ADC0_SE7b
	PORTB_PCR0 = PORT_PCR_MUX(0);	//ADC0_SE8
	PORTB_PCR1 = PORT_PCR_MUX(0);	//ADC0_SE9
	PORTC_PCR2 = PORT_PCR_MUX(0);	//ADC0_SE11
	PORTB_PCR2 = PORT_PCR_MUX(0);	//ADC0_SE12
	PORTB_PCR3 = PORT_PCR_MUX(0);	//ADC0_SE13
	PORTC_PCR0 = PORT_PCR_MUX(0);	//ADC0_SE14
	PORTC_PCR1 = PORT_PCR_MUX(0);	//ADC0_SE15
	PORTE_PCR30 = PORT_PCR_MUX(0);	//ADC0_SE23*/
}

void ADC0_IRQ()
{
	uint16_t junk = ADC0_RA;
}

/*void ADC1_IRQ()
{
	uint16_t junk =  ADC1_RA;
}*/



/*	ARGUMENTS:	Channel - selected ADC channel for conversion start
 *  
 *  OUTPUT:		ADC1_RA - Converted digital value of camera analog output
 *  
 *  NOTE:		Function makes conversion from analog camera output to digital
 */
uint16_t Read_ADC0(uint8_t Channel)
{
	//Writing to this register for the channel starts a conversion 
	ADC0_SC1A  =  Channel;
	
	// Wait until conversion complete
	while((ADC0_SC1A & ADC_SC1_COCO_MASK) == 0);

	return 	ADC0_RA;	
}

/*	ARGUMENTS:	Channel - selected ADC channel for conversion start
 *  
 *  OUTPUT:		ADC1_RA - Converted digital value of camera analog output
 *  
 *  NOTE:		Function makes conversion from analog camera output to digital
 */
/*uint16_t Read_ADC1(uint8_t Channel)
{
	//Writing to this register for the channel starts a conversion 
	ADC1_SC1A  =  Channel;
	
	// Wait until conversion complete
	while((ADC1_SC1A & ADC_SC1_COCO_MASK) == 0);
	
	return 	ADC1_RA;	
}*/





