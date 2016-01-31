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
//long adc[15],adc_temp[15];
float bat1_volt,bat2_volt,bat3_volt,BAT_VOLT;
char adc_flag=0;
int32_t t_period,t=0,tt[10],t_period,t_period_temp;

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
	No++;
	//DMAread_MPU6050();
	t=ticker;
	read_MPU6050();
	tt[1]=t-ticker;
	t=ticker;
	Get_Angles();	//882
	second_order_complementary_filter();		//kikiserletezeni 2015.12.23
	turnigy_9x();
//	complementary_filter();	//68
//	Convert_Accel();
	tt[2]=t-ticker;
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
	tt[3]=t-ticker;
	t=ticker;
	output_x=PID_pitchroll(setpoint_x,-COMPLEMENTARY_XANGLE,&errorSum_x,GYRO_XRATE);
	output_y=PID_pitchroll(setpoint_y,COMPLEMENTARY_YANGLE,&errorSum_y,-GYRO_YRATE);
	output_z=PID_yaw(setpoint_z,GYRO_ZANGLE,GYRO_ZRATE);
	tt[4]=t-ticker;
	t=ticker;
	A_f=basepower+output_y-output_z+output_ALT;
	C_f=basepower-output_y-output_z+output_ALT;
	B_f=basepower+output_x+output_z+output_ALT; //ha a B motor van feljebb mint a D motor akkor a COMPLEMENTARY_XANGLE pozitiv
	D_f=basepower-output_x+output_z+output_ALT;
	A=Convert_FORCEtoPWM(A_f),B=Convert_FORCEtoPWM(B_f),C=Convert_FORCEtoPWM(C_f),D=Convert_FORCEtoPWM(D_f);
	tt[5]=t-ticker;
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
	tt[6]=t-ticker;
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
	ADC1_SC1A |= ADC_SC1_AIEN_MASK;
	tt[7]=t-ticker;
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
			(int)(C),(int)D,(int)(basepower),(int)(100*bat1_volt),
			(int)(100*bat2_volt),(int)(abs(ticker)),0/*elapsed_s*/);//,(int)RTC_TSR);	//963	
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
	uart_putchar(UART4_BASE_PTR,dataout);
#endif
	if(setpoint_x>15) setpoint_x=15;
	if(setpoint_x<-15) setpoint_x=-15;
	if(setpoint_y>15) setpoint_y=15;
	if(setpoint_y<-15) setpoint_y=-15;
	if(basepower>500) basepower=500;
#if !PID_tuning
	uart_putchar(UART4_BASE_PTR,(char)(10*BAT_VOLT));
#endif
	//enable_irq(INT_PORTA - 16); 2015.12.9
}

void ADC0()
{
	char i,adc_register;
	static uint8_t adc0_avg=0;
	static long adc0_temp[1];
	if((ADC0_SC1A & ADC_SC1_ADCH_MASK) == 0) {adc0_temp[0]+=ADC0_RA;	adc_register=0 | 0b1000000; adc0_avg++;}
	if(adc0_avg>=ADC_AVG){
		for(i=0;i<1;i++)	adc0_temp[i]=adc0_temp[i]/ADC_AVG;
		bat1_volt=0.99*bat1_volt+0.01*(float)(adc0_temp[0])/11717;
		for(i=0;i<1;i++)	adc0_temp[i]=0;
		ADC0_SC1A &= ~ADC_SC1_AIEN_MASK;	//Turn off interrupt
		adc0_avg=0;
	}
	ADC0_SC1A=adc_register;
}

void ADC1()
{
	char i,adc_register;
	static uint8_t adc1_avg=0;
	static long adc1_temp[2];
	if((ADC1_SC1A & ADC_SC1_ADCH_MASK) == 0) {adc1_temp[0]+=ADC1_RA;	adc_register=1 | 0b1000000;}
	if((ADC1_SC1A & ADC_SC1_ADCH_MASK) == 1) {adc1_temp[1]+=ADC1_RA;	adc_register=0 | 0b1000000; adc1_avg++;}
	if(adc1_avg>=ADC_AVG){
		for(i=0;i<2;i++)	adc1_temp[i]=adc1_temp[i]/ADC_AVG;
		bat2_volt=0.99*bat2_volt+0.01*((float)(adc1_temp[1])/7086-bat1_volt);//10646
		bat3_volt=0.99*bat3_volt+0.01*((float)(adc1_temp[0])/3862-bat1_volt-bat2_volt);
		BAT_VOLT=bat1_volt+bat2_volt+bat3_volt;
		for(i=0;i<2;i++)	adc1_temp[i]=0;
		ADC1_SC1A &= ~ADC_SC1_AIEN_MASK;		//Turn off interrupt
		adc1_avg=0;
	}
	if((bat1_volt<BATTERY_MINIMUM_VOLTAGE || bat2_volt<BATTERY_MINIMUM_VOLTAGE || bat3_volt<BATTERY_MINIMUM_VOLTAGE) && No>2000) 
			{
			LOW_BATT_FLAG=1;
			}
	ADC1_SC1A=adc_register;
	
}

void capture_ppm(){
	if(ch1_pulse){
		if(ch1_rising_edge)	{ch1_temp=PIT_CVAL2; 		ch1_set_rising_edge;}
		else{ch1=ch1_temp-PIT_CVAL2;	ch1_set_falling_edge;	ch1=(ch1-ch1_offset);
							if(ch1_offset!=0)	ch1=(-1*ch1);
		} //877281
		ch1_clear_interrupt;
	}
	if(ch2_pulse){
		if(ch2_rising_edge)	{ch2_temp=PIT_CVAL2; 	ch2_set_rising_edge;}
		else{
			ch2=ch2_temp-PIT_CVAL2;	
			ch2_set_falling_edge;	
			ch2=(ch2-ch2_offset); 
			if(ch2_offset!=0)	ch2=(-1*ch2);
		}
		ch2_clear_interrupt;
	}
	if(ch3_pulse){
		if(ch3_rising_edge)	{ch3_temp=PIT_CVAL2; 	ch3_set_rising_edge;}
		else{
			ch3=ch3_temp-PIT_CVAL2;
			ch3_set_falling_edge;
			ch3=(ch3-ch3_offset);
			if(ch3_offset!=0)	ch3=(-1*ch3);
		}
		ch3_clear_interrupt;
	}
	if(ch4_pulse){
		if(ch4_rising_edge)	{ch4_temp=PIT_CVAL2; 		ch4_set_rising_edge;}
		else{
			ch4=ch4_temp-PIT_CVAL2;
			ch4_set_falling_edge;
			ch4=(ch4-ch4_offset);
			if(ch4_offset!=0)	ch4=(-1*ch4);
		}
		ch4_clear_interrupt;
	}
	
	if((ch1_offset == 1) || (ch2_offset == 1) || (ch3_offset == 1) || (ch4_offset == 1))	{ch1=0;	ch2=0;	ch3=0;	ch4=0;}	//Prevent using remote control without calibration
}

int sz=0;
void FTM0_interrupt(){
	sz++;
	if(sz%2==0)	GPIOE_PSOR = 1<<24;
	else		GPIOE_PCOR = 1<<24;
	GPIOE_PTOR = 1<<25;
	FTM0_SC &= ~FTM_SC_TOF_MASK;
}

void ADC_DMA()
{
	
}
