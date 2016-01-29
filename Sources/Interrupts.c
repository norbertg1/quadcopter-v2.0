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

float Kp=6.5,Kd=2.0/*2*/,Ki=0/*100*/,zKp=2.5,zKd=0.7,Alt_Kp=0/*25*/,Alt_Kd=0/*20*/,Alt_Ki=0/*0.5*/,Kp_mem,Kd_mem;
int basepower=-50,setpoint_x=0,setpoint_y=0,setpoint_z=0,No=0,flag_kyb;
float setpoint_alt=0;
int A_f,B_f,C_f,D_f,A,B,C,D;
float errorSum_x=0,errorSum_y=0,errorSum_alt=0,errorPrev_alt;
short flag,LOW_BATT_FLAG;
char flag_landing=0;
float output_ALT=0;
float output_x,output_y,output_z;// csak debugoolas vegett deklaralva fent
long adc[15],adc_temp[15];
float batt1_vol,batt2_vol,batt3_vol,BATT_VOLT;
char adc_flag=0;
int32_t t_period,t=0,tt[10];

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

void PID_Interrupt(void)
{
	t_period=ticker;
	//FTM1_CNT=0; 2015.11 idõmérésre van azthiszem
	tt[0]=t-ticker;
	t=ticker;
	No++;
	//DMAread_MPU6050();
	read_MPU6050();
	tt[1]=t-ticker;
	t=ticker;
	Get_Angles();	//882
	second_order_complementary_filter();		//kikiserletezeni 2015.12.23
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
	output_x=PID_pitchroll(setpoint_x,-COMPLEMENTARY_XANGLE,&errorSum_x,GYRO_XRATE);
	output_y=PID_pitchroll(setpoint_y,COMPLEMENTARY_YANGLE,&errorSum_y,-GYRO_YRATE);
	output_z=PID_yaw(setpoint_z,GYRO_ZANGLE,GYRO_ZRATE);
	tt[3]=t-ticker;
	t=ticker;
	A_f=basepower+output_y-output_z+output_ALT;
	C_f=basepower-output_y-output_z+output_ALT;
	B_f=basepower+output_x+output_z+output_ALT; //ha a B motor van feljebb mint a D motor akkor a COMPLEMENTARY_XANGLE pozitiv
	D_f=basepower-output_x+output_z+output_ALT;
	A=Convert_FORCEtoPWM(A_f),B=Convert_FORCEtoPWM(B_f),C=Convert_FORCEtoPWM(C_f),D=Convert_FORCEtoPWM(D_f);
	tt[4]=t-ticker;
	t=ticker;
	SetMotorPWM(A,B,C,D);	//180
	if(abs(COMPLEMENTARY_XANGLE)>35 || 35<abs(COMPLEMENTARY_YANGLE) || flag_landing==1) 
		{
			//SetMotorPWM(0,0,0,0);
			//set_irq_priority (INT_PIT1 - 16, 1);
			//Delay_mS(10000);
			flag_landing=1;
			if(No%50 == 0 && (A>50 || B>50 || C>50 || D>50)) basepower-=5;
			Kp_mem=Kp;Kd_mem=Kd;
			//Kp=5; Kd=1;
			setpoint_alt=1;
			//if(A<50 && B<50 && C<50 && D<50) {flag_landing=0; Kp=Kp_mem; Kd=Kd_mem;}
			setpoint_alt=0;
		}
	if(abs(No-flag_kyb)>40) 
		{
		if(No%5 == 0 && setpoint_x>0) setpoint_x-=1;
		if(No%5 == 0 && setpoint_x<0) setpoint_x+=1;
		if(No%5 == 0 && setpoint_y>0) setpoint_y-=1;
		if(No%5 == 0 && setpoint_y<0) setpoint_y+=1;
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
	if(No%15==0)	{
		char c[sizeof(float)],i;
		memcpy(c, &COMPLEMENTARY_YANGLE, sizeof(float));
		for(i=0;i<sizeof(float);i++) uart_putchar(UART2_BASE_PTR,c[i]);
		uart_putchar(UART2_BASE_PTR,255);
	}
#endif
	ADC0_SC1A |= ADC_SC1_AIEN_MASK;
	ADC0_SC1A |= ADC_SC1_AIEN_MASK;
	tt[6]=t-ticker;
	t=ticker;
	PIT_TFLG0  |= PIT_TFLG_TIF_MASK;
	PIT_LDVAL0 = PERIPHERAL_BUS_CLOCK / PIT0_OVERFLOW_FREQUENCY;
	t_period=t_period-ticker;	
}	
void SDcardw_Interrupt(void)
{
	char data[71];
	UINT x;
	int ttt=ticker;

	clear_SDcard_interrupt
	write_dT(data,12,(int)No,(int)(100*COMPLEMENTARY_XANGLE),(int)(100*COMPLEMENTARY_YANGLE),(int)(A),(int)(B),
			(int)(C),(int)D,(int)(basepower),(int)(100*batt1_vol),
			(int)(100*batt2_vol),(int)(abs(ticker)),0/*elapsed_s*/);//,(int)RTC_TSR);	//963	
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

void UART_interrupt()
{
	char c;
	char dataout;
	c=uart_getchar(UART4_BASE_PTR);
	
	if(c=='d')				{setpoint_x+=3;setpoint_y+=3;	flag_kyb=No;}	//forward
	if(c=='a')				{setpoint_x-=3;setpoint_y-=3;	flag_kyb=No;}	//back
	if(c=='w')				{setpoint_x+=3;setpoint_y-=3;	flag_kyb=No;}	//right
	if(c=='s')				{setpoint_x-=3;setpoint_y+=3;	flag_kyb=No;}	//left
	if(c==',')				setpoint_z-=4;		
	if(c=='.')				setpoint_z+=4;
	if(c=='+')				basepower+=2;
	if(c=='-')				basepower-=2;
	if(c=='9')				basepower+=10;
	if(c=='6')				basepower-=10;
//	if(c=='w')				{setpoint_alt+=0.5;	uart_putchar(UART1_BASE_PTR,(char)(setpoint_alt));}
//	if(c=='s')				{setpoint_alt-=0.5;	uart_putchar(UART1_BASE_PTR,(char)(setpoint_alt));}
	if(c=='c')				{setpoint_x=0;setpoint_y=0;setpoint_z=0;setpoint_alt=1;}
	if(c=='0')				flag_landing=1;
	if(c=='1' || c=='x')				{SetMotorPWM(0,0,0,0); disable_irq(INT_PIT0 - 16);}	//turn off 
#if PID_tuning	
	if(c=='r')				{Kp+=0.5;				dataout=(char)(Kp);}	
	if(c=='f')				{Kp-=0.5;				dataout=(char)(Kp);}
	if(c=='t')				{Kd+=0.05;				dataout=(char)(Kd);}
	if(c=='g')				{Kd-=0.05;				dataout=(char)(Kd);}
	if(c=='y')				{Ki+=1;					dataout=(char)(Ki);}
	if(c=='h')				{Ki-=1;					dataout=(char)(Ki);}
	if(c=='u')				{timeConstant+=0.1;	dataout=(char)10*timeConstant;}
	if(c=='j')				{timeConstant-=0.1;	dataout=(char)10*timeConstant;}
	if(c=='y')				{Alt_Kp++;				dataout=(char)Alt_Kp;}
	if(c=='x')				{Alt_Kp--;				dataout=(char)Alt_Kp;}
	if(c=='v')				{Alt_Kd+=0.5;			dataout=(char)Alt_Kd;}
	if(c=='b')				{Alt_Kd-=0.5;			dataout=(char)Alt_Kd;}
	if(c=='n')				{Alt_Ki+=0.5;			dataout=(char)Alt_Ki;}
	if(c=='m')				{Alt_Ki-=0.5;			dataout=(char)Alt_Ki;}
	uart_putchar(UART4_BASE_PTR,/*dataout*/'a');
#endif
	if(setpoint_x>15) setpoint_x=15;
	if(setpoint_x<-15) setpoint_x=-15;
	if(setpoint_y>15) setpoint_y=15;
	if(setpoint_y<-15) setpoint_y=-15;
	if(basepower>500) basepower=500;
#if !PID_tuning
	uart_putchar(UART4_BASE_PTR,(char)(10*BATT_VOLT));
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
	static uint8_t ADC_SEL=0;
	//read conversation result; start new conversation
	if(ADC_SEL == 0)		{adc_temp[0]+=ADC0_RA;	adc_register=1 	| 0b1000000; ADC_SEL = 1;	goto end;}										//read ADC0_SE5b,	start conversation ADC0_SE6b
	if(ADC_SEL == 1)		{adc_temp[1]+=ADC1_RA;	adc_register=0 	| 0b1000000; ADC_SEL = 2;	goto end;}
	if(ADC_SEL == 2)		{adc_temp[2]+=ADC1_RA;	adc_register=0 	| 0b1000000; ADC_SEL = 0;	adc_flag++;	goto end;}										//read ADC0_SE15,	start conversation ADC0_SE23
end:
	if(adc_flag>=ADC_avg) 
		{
		int i;
		for(i=0;i<15;i++)	{
			adc[i]=adc_temp[i]/(adc_flag); 
			adc_temp[i]=0;
		}
		batt1_vol=0.99*batt1_vol+0.01*(float)(adc[0]<<mode)/22172;	//17007, 16500, 22939
		batt2_vol=0.99*batt2_vol+0.01*((float)(adc[1]<<mode)/13442-batt1_vol);//10646
		batt3_vol=0.99*batt3_vol+0.01*((float)(adc[2]<<mode)/7347-batt1_vol-batt2_vol);
		BATT_VOLT=batt1_vol+batt2_vol+batt3_vol;
		if((batt1_vol<BATTERY_MINIMUM_VOLTAGE || batt2_vol<BATTERY_MINIMUM_VOLTAGE || batt3_vol<BATTERY_MINIMUM_VOLTAGE) && No>2000) 
			{
			//LOW_BATT_FLAG=1;
			}
		ADC0_SC1A = ADC0_SC1A & ~ADC_SC1_AIEN_MASK;
		ADC0_SC1A = ADC0_SC1A & ~ADC_SC1_AIEN_MASK;
//		disable_irq(INT_ADC0 - 16);
//		disable_irq(INT_ADC1 - 16);
		}
#endif
	if(ADC_SEL == 0) ADC0_SC1A=adc_register;
	else	ADC1_SC1A=adc_register;
}

void turnigy_timer(){
	
}

void turnigy_interrupt(){
	
}

void ADC_DMA()
{
	
}
