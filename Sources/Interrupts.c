/*
 * Interrupts.c
 *
 *  Created on: Dec 10, 2014
 *      Author: Norbi
 */

#include "init.h"
#include "stdlib.h"
#define integral_max 0.5
#define integral_alt_max 10 

float Kp=12,Kd=2.0,Ki=100,zKp=2.5,zKd=0.7,Alt_Kp=0/*25*/,Alt_Kd=0/*20*/,Alt_Ki=0/*0.5*/,Kp_mem,Kd_mem;
int basepower=-50,setpoint_x=0,setpoint_y=0,setpoint_z=0,No=0,flag_kyb;
float setpoint_alt=0;
int A_f,B_f,C_f,D_f,A,B,C,D;
float errorSum_x=0,errorSum_y=0,errorSum_alt=0,errorPrev_alt;
short flag,LOW_BATT_FLAG;
char flag_landing=0;
float output_ALT=0;
float output_x,output_y,output_z;// csak debugoolas vegett deklaralva fent
long adc[8],adc_temp[8];
float batt1_vol,batt2_vol,batt3_vol,BATT_VOLT;
char adc_flag=0;

float PID_pitchroll(int setpoint, float measured_value,float *errorSum, float dErr)
{
	float error;
	error=setpoint+measured_value;
	*errorSum=*errorSum+error*dt;
	return Kp*error+Kd*dErr+Ki**errorSum;
}
float PID_yaw(int setpoint, float measured_value,float dErr)
{
	float error;
	error=setpoint+measured_value;
	return zKp*error+zKd*dErr;
}

float PID_ALT_HOLD(float setpoint, float measured_value,float *errorSum,float prev_error)
{
	float error,dErr;
	error=setpoint-measured_value;
	dErr=error-prev_error;
	*errorSum=*errorSum+error*dt;
	return Alt_Kp*error+Alt_Kd*dErr+Alt_Ki**errorSum;
}

void read_BMP180()
{
	if(No%(PIT0_OVERFLOW_FREQUENCY/200)==0 && flag==1)
		{
			BMP180_read_temp_interrupt();
			I2CWriteRegister(BMP180_ADDR,BMP180_REG_CONTROL,BMP180_CMD_READ_PRESS + (BMP180_OSS<<6));
			flag=0;
		}
		if(No%(PIT0_OVERFLOW_FREQUENCY/25)==0)	
		{
			BMP180_read_pressure_interrupt();
			I2CWriteRegister(BMP180_ADDR,BMP180_REG_CONTROL,BMP180_CMD_READ_TEMP);
			errorPrev_alt=setpoint_alt-relative_altittude;
			BMP180_get_altittude();
			output_ALT=PID_ALT_HOLD(setpoint_alt,relative_altittude,&errorSum_alt,errorPrev_alt);
			if(output_ALT>150) output_ALT=150;
			if(output_ALT<-150) output_ALT=-150;
			flag=1;
		}
}
int32_t t=0,tt[10];

void PID_Interrupt(void)
{
	//FTM1_CNT=0; 2015.11 idõmérésre van azthiszem
	tt[0]=t-ticker;
	t=ticker;
	No++;

	//DMAread_MPU6050();
	read_MPU6050();
	tt[1]=t-ticker;
	t=ticker;
	Get_Angles();	//882
	second_order_complementary_filter();
//	complementary_filter();	//68
//	Convert_Accel();
	tt[1]=t-ticker;
	t=ticker;
//	errorSum_x=((9.993/10.0)*errorSum_x)+(error_x*dt);
//	errorSum_y=((9.993/10.0)*errorSum_y)+(error_y*dt);
	if(errorSum_x>integral_max)	errorSum_x=integral_max;
	if(errorSum_x<-integral_max)	errorSum_x=-integral_max;
	if(errorSum_y>integral_max)	errorSum_y=integral_max;
	if(errorSum_y<-integral_max)	errorSum_y=-integral_max;
	if(errorSum_alt>integral_alt_max)	errorSum_alt=integral_alt_max;
	if(errorSum_alt<-integral_alt_max)	errorSum_alt=-integral_alt_max;
	if(basepower<170) {errorSum_x=0, errorSum_y=0;}
	tt[2]=t-ticker;
	t=ticker;
	output_x=PID_pitchroll(setpoint_x,COMPLEMENTARY_XANGLE,&errorSum_x,GYRO_XRATE);
	output_y=PID_pitchroll(setpoint_y,-COMPLEMENTARY_YANGLE,&errorSum_y,-GYRO_YRATE);
	output_z=PID_yaw(setpoint_z,-GYRO_ZANGLE,-GYRO_ZRATE);
	tt[3]=t-ticker;
	t=ticker;
	A_f=basepower-output_y-output_z+output_ALT;
	C_f=basepower+output_y-output_z+output_ALT;
	B_f=basepower-output_x+output_z+output_ALT; //ha a B motor van feljebb mint a D motor akkor a COMPLEMENTARY_XANGLE pozitiv
	D_f=basepower+output_x+output_z+output_ALT;
	A=Convert_FORCEtoPWM(A_f),B=Convert_FORCEtoPWM(B_f),C=Convert_FORCEtoPWM(C_f),D=Convert_FORCEtoPWM(D_f);
	tt[4]=t-ticker;
	t=ticker;
	SetMotorPWM(A,B,C,D);	//180
	if(abs(COMPLEMENTARY_XANGLE)>25 || 25<abs(COMPLEMENTARY_YANGLE) || flag_landing==1) 
		{
			//SetMotorPWM(0,0,0,0);
			//set_irq_priority (INT_PIT1 - 16, 1);
			//Delay_mS(10000);
			flag_landing=1;
			if(No%50 == 0 && (A>50 || B>50 || C>50 || D>50)) basepower-=5;
			Kp_mem=Kp;Kd_mem=Kd;
			Kp=5; Kd=1;
			setpoint_alt=1;
			if(A<50 && B<50 && C<50 && D<50) {flag_landing=0; Kp=Kp_mem; Kd=Kd_mem;}
			setpoint_alt=0;
		}
	if(abs(No-flag_kyb)>40) 
		{
		if(No%7 == 0 && setpoint_x>0) setpoint_x-=1;
		if(No%7 == 0 && setpoint_x<0) setpoint_x+=1;
		if(No%7 == 0 && setpoint_y>0) setpoint_y-=1;
		if(No%7 == 0 && setpoint_y<0) setpoint_y+=1;
		}
	tt[5]=t-ticker;
	t=ticker;
	//read_BMP180();	2015.12.01
	if(No%400==1 && LOW_BATT_FLAG) 
		{
		initRed();
		LED_Toggle_RED;	//enable RED LED
		basepower-=10;
		}
#if DATA_OVER_UART
	//uart_putchar(UART1_BASE_PTR,(char)(D));
	if(No%20==0)	{
		char c[sizeof(float)],i;
		memcpy(c, &COMPLEMENTARY_XANGLE, sizeof(float));
		for(i;i<sizeof(float);i++) uart_putchar(UART1_BASE_PTR,c[i]);
		uart_putchar(UART1_BASE_PTR,'\n');
	}
#endif
	//enable_irq(INT_ADC0 - 16); adc_flag=0;
	clear_PID_interrupt
	tt[6]=t-ticker;
	t=ticker;
	}
void SDcardw_Interrupt(void)
{
	clear_SDcard_interrupt
	char data[71];
	UINT x;
	int ttt=ticker;
	write_dT(data,12,(int)No,(int)(100*COMPLEMENTARY_XANGLE),(int)(100*COMPLEMENTARY_YANGLE),(int)(tt[0]),(int)(tt[1]),
			(int)(tt[2]),(int)tt[3],(int)(tt[4]),(int)(tt[7]),
			(int)(tt[8]),(int)(abs(ticker)),0/*elapsed_s*/);//,(int)RTC_TSR);	//963	
	tt[7]=ttt-ticker;
	ttt=ticker;
	f_write(&fil,data,71,&x);	//168	itt valami tortenik, mert minden 7. lefutasnal 165 helyett 7566ig tart 
	f_write(&fil,"\r\n",2,&x);	//27
	tt[8]=ttt-ticker;
	if(No%250==0)
		{
		f_sync(&fil);			//sync data with SD card	Vigyázz KL25nel mert megszakitas koyben nem mukodik a Delay_mS ezert valtoztatva van a int wait_ready a diskio.c-ben!!!!! 
		}
	
}

void bluetooth_getchar()
{
	char c;
	c=uart_getchar(UART0_BASE_PTR);
	
	if(c==0x23)				{setpoint_x+=3;setpoint_y+=3;	flag_kyb=No;}	//forward
	if(c==0x24)				{setpoint_x-=3;setpoint_y-=3;	flag_kyb=No;}	//back
	if(c==0x25)				{setpoint_x+=3;setpoint_y-=3;	flag_kyb=No;}	//right
	if(c==0x26)				{setpoint_x-=3;setpoint_y+=3;	flag_kyb=No;}	//left
	if(c==',')				setpoint_z-=4;		
	if(c=='.')				setpoint_z+=4;
	if(c=='+')				basepower+=2;
	if(c=='-')				basepower-=2;
	if(c=='*')				basepower+=10;
	if(c=='/')				basepower-=10;
	if(c=='w')				{setpoint_alt+=0.5;	uart_putchar(UART1_BASE_PTR,(char)(setpoint_alt));}
	if(c=='s')				{setpoint_alt-=0.5;	uart_putchar(UART1_BASE_PTR,(char)(setpoint_alt));}
	if(c=='c')				{setpoint_x=0;setpoint_y=0;setpoint_z=0;setpoint_alt=1;}
	if(c=='0')				flag_landing=1;
	if(c=='~')				{SetMotorPWM(0,0,0,0); disable_irq(INT_TPM1 - 16);}	//turn off 
#if PID_tuning	
	char dataout;
	if(c=='e')				{Kp+=0.5;				dataout=(char)(Kp);}	
	if(c=='d')				{Kp-=0.5;				dataout=(char)(Kp);}
	if(c=='r')				{Kd+=0.05;				dataout=(char)(Kd);}
	if(c=='f')				{Kd-=0.05;				dataout=(char)(Kd);}
	if(c=='t')				{Ki+=1;					dataout=(char)(Ki);}
	if(c=='g')				{Ki-=1;					dataout=(char)(Ki);}
	if(c=='z')				{timeConstant+=0.01;	dataout=(char)100*timeConstant;}
	if(c=='h')				{timeConstant-=0.01;	dataout=(char)100*timeConstant;}
	if(c=='y')				{Alt_Kp++;				dataout=(char)Alt_Kp;}
	if(c=='x')				{Alt_Kp--;				dataout=(char)Alt_Kp;}
	if(c=='v')				{Alt_Kd+=0.5;			dataout=(char)Alt_Kd;}
	if(c=='b')				{Alt_Kd-=0.5;			dataout=(char)Alt_Kd;}
	if(c=='n')				{Alt_Ki+=0.5;			dataout=(char)Alt_Ki;}
	if(c=='m')				{Alt_Ki-=0.5;			dataout=(char)Alt_Ki;}
	uart_putchar(UART0_BASE_PTR,dataout);
#endif
	if(setpoint_x>15) setpoint_x=15;
	if(setpoint_x<-15) setpoint_x=-15;
	if(setpoint_y>15) setpoint_y=15;
	if(setpoint_y<-15) setpoint_y=-15;
	if(basepower>500) basepower=500;
#if !PID_tuning
	uart_putchar(UART1_BASE_PTR,(char)(10*BATT_VOLT));
#endif
	//enable_irq(INT_PORTA - 16); 2015.12.9
}

void bluetooth_lostconnection()
{
	flag_landing=1;
	PORTA_PCR5 |= PORT_PCR_IRQC(5);
	disable_irq(INT_PORTA - 16);
}

void ADC()
{
	char mode=0,adc_register;
	//FTM1_CNT=0;	2015.11
#if ADC_avg==1	
	if((ADC0_SC1A & ADC_SC1_ADCH_MASK) == 23)	{adc[0]=ADC0_RA;	adc_register=21 | 0b1000000;}	//read conversation result; start new conversation
	if((ADC0_SC1A & ADC_SC1_ADCH_MASK) == 21)	{adc[1]=ADC0_RA;	adc_register=0 | 0b1000000;}
	if((ADC0_SC1A & ADC_SC1_ADCH_MASK) == 0)	{adc[2]=ADC0_RA;	adc_register=8 | 0b1000000;}
	if((ADC0_SC1A & ADC_SC1_ADCH_MASK) == 8)	{adc[3]=ADC0_RA;	adc_register=9 | 0b1000000;}
	if((ADC0_SC1A & ADC_SC1_ADCH_MASK) == 9)	{adc[4]=ADC0_RA;	adc_register=14 | 0b1000000;}
	if((ADC0_SC1A & ADC_SC1_ADCH_MASK) == 14)	{adc[5]=ADC0_RA;	adc_register=15 | 0b1000000;}
	if((ADC0_SC1A & ADC_SC1_ADCH_MASK) == 15)	{adc[6]=ADC0_RA;	adc_register=19 | 0b1000000;}
	if((ADC0_SC1A & ADC_SC1_ADCH_MASK) == 19)	{adc[7]=ADC0_RA;	adc_register=23 | 0b1000000; disable_irq(INT_ADC0 - 16);}
	batt1_vol=0.99*batt1_vol+0.01*(float)(adc[0]<<mode)/13538;
	batt2_vol=0.99*batt2_vol+0.01*((float)(adc[1]<<mode)/7019-batt1_vol);
	batt3_vol=0.99*batt3_vol+0.01*((float)(adc[2]<<mode)/4955-batt1_vol-batt2_vol);
	BATT_VOLT=batt1_vol+batt2_vol+batt3_vol;
	if((batt1_vol<BATTERY_MINIMUM_VOLTAGE || batt2_vol<BATTERY_MINIMUM_VOLTAGE || batt3_vol<BATTERY_MINIMUM_VOLTAGE) && No>400) LOW_BATT_FLAG=1;
#else 
	if((ADC0_SC1A & ADC_SC1_ADCH_MASK) == 0)	{adc_temp[2]+=ADC0_RA;	adc_register=4 | 0b1000000; ADC0_CFG2 &= ~ADC_CFG2_MUXSEL_MASK;}	//ADC0_DP0, ADC0_SE0, select ADxxa channels
	if((ADC0_SC1A & ADC_SC1_ADCH_MASK) == 4)	{adc_temp[2]+=ADC0_RA;	adc_register=3 | 0b1000000;}										//ADC0_DM0, ADC0_SE4a
	if((ADC0_SC1A & ADC_SC1_ADCH_MASK) == 3)	{adc_temp[2]+=ADC0_RA;	adc_register=7 | 0b1000000;}										//ADC0_DP0, ADC0_SE3
	if((ADC0_SC1A & ADC_SC1_ADCH_MASK) == 7)	{adc_temp[2]+=ADC0_RA;	adc_register=4 | 0b1000000; ADC0_CFG2 |= ADC_CFG2_MUXSEL_MASK;}		//ADC0_DP0, ADC0_SE7a, select ADxxb channels
	if((ADC0_SC1A & ADC_SC1_ADCH_MASK) == 4)	{adc_temp[2]+=ADC0_RA;	adc_register=5 | 0b1000000;}										//ADC0_SE4b
	if((ADC0_SC1A & ADC_SC1_ADCH_MASK) == 5)	{adc_temp[2]+=ADC0_RA;	adc_register=6 | 0b1000000;}										//ADC0_SE5b
	if((ADC0_SC1A & ADC_SC1_ADCH_MASK) == 6)	{adc_temp[2]+=ADC0_RA;	adc_register=8/*7*/ | 0b1000000; ADC0_CFG2 &= ~ADC_CFG2_MUXSEL_MASK;}	//ADC0_SE6b, select ADxxa channels
//	if((ADC0_SC1A & ADC_SC1_ADCH_MASK) == 7)	{adc_temp[2]+=ADC0_RA;	adc_register=8 | 0b1000000;}										//ADC0_SE7b	KONFLIKT UART0!!!!
	if((ADC0_SC1A & ADC_SC1_ADCH_MASK) == 0)	{adc_temp[2]+=ADC0_RA;	adc_register=8 | 0b1000000;}
	if((ADC0_SC1A & ADC_SC1_ADCH_MASK) == 0)	{adc_temp[2]+=ADC0_RA;	adc_register=8 | 0b1000000;}
	if((ADC0_SC1A & ADC_SC1_ADCH_MASK) == 23)	{adc_temp[0]+=ADC0_RA;	adc_register=21 | 0b1000000;}	//read conversation result; start new conversation
	if((ADC0_SC1A & ADC_SC1_ADCH_MASK) == 21)	{adc_temp[1]+=ADC0_RA;	adc_register=0 | 0b1000000;}
	if((ADC0_SC1A & ADC_SC1_ADCH_MASK) == 0)	{adc_temp[2]+=ADC0_RA;	adc_register=8 | 0b1000000;}
	if((ADC0_SC1A & ADC_SC1_ADCH_MASK) == 8)	{adc_temp[3]+=ADC0_RA;	adc_register=9 | 0b1000000;}
	if((ADC0_SC1A & ADC_SC1_ADCH_MASK) == 9)	{adc_temp[4]+=ADC0_RA;	adc_register=14 | 0b1000000;}
	if((ADC0_SC1A & ADC_SC1_ADCH_MASK) == 14)	{adc_temp[5]+=ADC0_RA;	adc_register=15 | 0b1000000;}
	if((ADC0_SC1A & ADC_SC1_ADCH_MASK) == 15)	{adc_temp[6]+=ADC0_RA;	adc_register=19 | 0b1000000;}
	if((ADC0_SC1A & ADC_SC1_ADCH_MASK) == 19)	{adc_temp[7]+=ADC0_RA;	adc_register=23 | 0b1000000; adc_flag++;}
	if(adc_flag>=ADC_avg) 
		{
		int i;
		for(i=0;i<8;i++)	{adc[i]=adc_temp[i]/(adc_flag); adc_temp[i]=0;}
		batt1_vol=0.99*batt1_vol+0.01*(float)(adc[0]<<mode)/13538;
		batt2_vol=0.99*batt2_vol+0.01*((float)(adc[1]<<mode)/7019-batt1_vol);
		batt3_vol=0.99*batt3_vol+0.01*((float)(adc[2]<<mode)/4955-batt1_vol-batt2_vol);
		BATT_VOLT=batt1_vol+batt2_vol+batt3_vol;
		if((batt1_vol<BATTERY_MINIMUM_VOLTAGE || batt2_vol<BATTERY_MINIMUM_VOLTAGE || batt3_vol<BATTERY_MINIMUM_VOLTAGE) && No>400) LOW_BATT_FLAG=1;
		disable_irq(INT_ADC0 - 16);
		}
#endif
	ADC0_SC1A=adc_register;	
}

void ADC_DMA()
{
	
}
