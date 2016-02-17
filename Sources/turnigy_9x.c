/*
 * turnigy_9x.c
 *
 *  Created on: Jan 29, 2016
 *      Author: Norbert
 */
#include "init.h"

int32_t ch1=0,ch2=0,ch3=0,ch4=0,ch5=0,ch6=0,ch7=0; 
int32_t ch1_temp,ch2_temp,ch3_temp,ch4_temp,ch5_temp,ch6_temp,ch7_temp;
int32_t ch1_offset=0,ch2_offset=0,ch3_offset=0,ch4_offset=0,ch5_offset=0,ch6_offset=0,ch7_offset=0;
int32_t	ch3_watchdog=0;

/*----PPM----------------------------
 * 
 * 
 * 
 * 
 * 
 * 
*/

/*
 * chx_offset = 1 ha a taviranyito nem volt bekapcsolva a kopter bekapcsolasanal	
 * ch1 �rt�ke -20 000t�l +20 000ig lehet, a t�vir�nyitot a kopter bekapcsolasa el�tt kell bekapcsolni
 * A t�vir�nyit 
 * 
 * 
 * 
*/
void turnigy_9x()							
{												//The timer peri
	if((ch1_offset != 1) || (ch2_offset != 1) || (ch3_offset != 1) || (ch4_offset != 1))
	{
		basepower=(ch3/40)-20;
		setpoint_x = ch1/2000;
		setpoint_y = -ch2/2000;
		setpoint_z = ch4/2000;
		ch3_watchdog++;
		Kp_receiver=(float)ch5/1000;
		Kd_receiver=(float)ch6/2000;
		Ki_receiver=(float)ch7/500;
	}
	else ch3_watchdog=0;
}
void init_turnigy_timer(){
	SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
	PIT_MCR = 0;
	PIT_MCR = PIT_MCR_FRZ_MASK;		// Enable PIT module clock with debug freeze	
	PIT_LDVAL1 = PERIPHERAL_BUS_CLOCK / PIT1_OVERFLOW_FREQUENCY;	// Calculate and Load timer reset value
//	PIT_TCTRL1 = PIT_TCTRL_TIE_MASK;	// Enable timer interrupt
	PIT_TCTRL1 |= PIT_TCTRL_TEN_MASK;	// Enable timer
}

void calibrate_offset()
{
	int i;
	int32_t ch1_tmp=0,ch2_tmp=0,ch3_tmp=0,ch4_tmp=0,ch5_tmp=0,ch6_tmp=0,ch7_tmp=0;
	Delay_mS(1000);
	for(i=0;i<TURNIGY_CALIBRATE_NUMBER;i++)
	{
		Delay_mS(50);
		ch1_tmp+=ch1;
		ch2_tmp+=ch2;
		ch3_tmp+=ch3;
		ch4_tmp+=ch4;
		ch5_tmp+=ch5;
		ch6_tmp+=ch6;
		ch7_tmp+=ch7;
	}
	if((ch1_tmp!=0) && (ch2_tmp!=0) && (ch3_tmp!=0) && (ch4_tmp!=0))
	{
		ch1_offset=861932;//ch1_tmp/TURNIGY_CALIBRATE_NUMBER;
		ch2_offset=861472;//ch2_tmp/TURNIGY_CALIBRATE_NUMBER;
		ch3_offset=ch3_tmp/TURNIGY_CALIBRATE_NUMBER;
		ch4_offset=861816;//ch4_tmp/TURNIGY_CALIBRATE_NUMBER;
//		ch5_offset=881705;								//az ir�ny�t�n tal�llhat� (-) jelz�sr�l indul a ch5 �rt�ke, nincsenek negat�v �rt�k 
		ch5_offset=861866;								//az ir�ny�t�n tal�llhat� (0) jelz�sr�l indul a ch5 �rt�ke, negat�v �s pozit�v ir�nyba
//		ch5_offset=ch5_tmp/TURNIGY_CALIBRATE_NUMBER;	//az kopter bekapcsol�si pillanat�ban a potm�ter poz�ci�ja lesz a ch5 nulla �rt�ke
//		ch6_offset=881090;								//az ir�ny�t�n tal�llhat� (-) jelz�sr�l indul a ch6 �rt�ke, nincsenek negat�v �rt�k
		ch6_offset=860368;								//az ir�ny�t�n tal�llhat� (0) jelz�sr�l indul a ch6 �rt�ke, negat�v �s pozit�v ir�nyba
//		ch6_offset=ch6_tmp/TURNIGY_CALIBRATE_NUMBER;	//az kopter bekapcsol�si pillanat�ban a potm�ter poz�ci�ja lesz a ch5 nulla �rt�ke
//		ch7_offset=881056;								//az ir�ny�t�n tal�llhat� (-) jelz�sr�l indul a ch7 �rt�ke, nincsenek negat�v �rt�k
		ch7_offset=861403;								//az ir�ny�t�n tal�llhat� (0) jelz�sr�l indul a ch7 �rt�ke, negat�v �s pozit�v ir�nyba
//		ch7_offset=ch7_tmp/TURNIGY_CALIBRATE_NUMBER;	//az kopter bekapcsol�si pillanat�ban a potm�ter poz�ci�ja lesz a ch5 nulla �rt�ke
	}
	else{
		ch1_offset=1;
		ch2_offset=1;
		ch3_offset=1;
		ch4_offset=1;
		ch5_offset=1;
		ch6_offset=1;
		ch7_offset=1;
	}	
}

void init_turnigy9x()
{
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTE_MASK;
	ch1_pin = PORT_PCR_MUX(1) | PORT_PCR_IRQC(0b1010);
	ch2_pin	= PORT_PCR_MUX(1) | PORT_PCR_IRQC(0b1010);
	ch3_pin	= PORT_PCR_MUX(1) | PORT_PCR_IRQC(0b1010);
	ch4_pin	= PORT_PCR_MUX(1) | PORT_PCR_IRQC(0b1010);
	ch5_pin	= PORT_PCR_MUX(1) | PORT_PCR_IRQC(0b1010);
	ch6_pin	= PORT_PCR_MUX(1) | PORT_PCR_IRQC(0b1010);
	ch7_pin	= PORT_PCR_MUX(1) | PORT_PCR_IRQC(0b1010);

	calibrate_offset();
}
