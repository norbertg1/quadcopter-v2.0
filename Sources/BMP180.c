/*
 * BMP180.c
 *
 *  Created on: Oct 25, 2014
 *      Author: Norbi
 */

#include "init.h"
//#include "processor_specific_functions\KL25Z\sys\derivative.h"


short AC1,AC2,AC3,B1,B2,B5,MB,MC,MD;
unsigned short AC4,AC5,AC6;
long T;
float absolute_altittude,start_altittude,relative_altittude;
float p,P;
double P_stat[100];
int h=0;
float k_alt=0.96;


void BMP180_read_temp_interrupt()
{
	char temp[2];
	long X1,X2,UT;
//	I2CWriteRegister(BMP180_ADDR,BMP180_REG_CONTROL,BMP180_CMD_READ_TEMP);	//mert interruptban van
//	Delay_mS(5);															//ez is
	I2CReadMultiRegisters(BMP180_ADDR,BMP180_REG_RESULT,temp,2);
	UT=temp[0]<<8 | temp[1];
	X1=(float)((UT-AC6)*AC5)/32768;
	X2=(float)(MC*2048)/(X1+MD);
	B5=X1+X2;
	T=(B5+8)/16;
}
void BMP180_read_temp()
{
	char temp[2];
	long X1,X2,UT;
	I2CWriteRegister(BMP180_ADDR,BMP180_REG_CONTROL,BMP180_CMD_READ_TEMP);
	Delay_mS(5);
	I2CReadMultiRegisters(BMP180_ADDR,BMP180_REG_RESULT,temp,2);
	UT=temp[0]<<8 | temp[1];
	X1=(float)((UT-AC6)*AC5)/32768;
	X2=(float)(MC*2048)/(X1+MD);
	B5=X1+X2;
	T=(B5+8)/16;
}
float BMP180_read_pressure_interrupt()
{
	char temp[3];
	long B3,B6,UP;
	long long B7;
	float X1,X2,X3,B4;
//	I2CWriteRegister(BMP180_ADDR,BMP180_REG_CONTROL,BMP180_CMD_READ_PRESS + (BMP180_OSS<<6));	//mert interruptban van
//	Delay_mS(26);																				//ez is
	
	I2CReadMultiRegisters(BMP180_ADDR,BMP180_REG_RESULT,temp,3);
	UP=(((temp[0]<<16) | (temp[1]<<8) | (temp[2]))>>(8-BMP180_OSS));
	B6=B5-4000;
	X1=((float)B2*(B6*B6/4096))/2048;
	X2=((float)(AC2*B6)/2048);
	X3=X1+X2;
	B3=(((AC1*4+(int)X3)<<BMP180_OSS)+2)/4;
	X1=(float)(AC3*B6)/8192;
	X2=(float)(B1*(B6*B6/4096))/65536;
	X3=((X1+X2)+2)/4;
	B4=(float)(AC4*(X3+32768))/32768;
	B7=(UP-B3)*(50000>>BMP180_OSS);
	if(B7<0x80000000) p=(float)(B7*2)/B4;
	else p=(double)((float)B7/B4)*2.0;
	X1=(p/256)*(p/256);
	X1=(X1*3038)/65536;
	X2=(-7357*p)/65536;
	p=p+(float)(X1+X2+3791)/16;
	return p;
}
float BMP180_read_pressure()
{
	char temp[3];
	long B3,B6,UP;
	long long B7;
	float X1,X2,X3,B4;
	I2CWriteRegister(BMP180_ADDR,BMP180_REG_CONTROL,BMP180_CMD_READ_PRESS + (BMP180_OSS<<6));
	Delay_mS(26);
	I2CReadMultiRegisters(BMP180_ADDR,BMP180_REG_RESULT,temp,3);
	UP=(((temp[0]<<16) | (temp[1]<<8) | (temp[2]))>>(8-BMP180_OSS));
	B6=B5-4000;
	X1=((float)B2*(B6*B6/4096))/2048;
	X2=((float)(AC2*B6)/2048);
	X3=X1+X2;
	B3=(((AC1*4+(int)X3)<<BMP180_OSS)+2)/4;
	X1=(float)(AC3*B6)/8192;
	X2=(float)(B1*(B6*B6/4096))/65536;
	X3=((X1+X2)+2)/4;
	B4=(float)(AC4*(X3+32768))/32768;
	B7=(UP-B3)*(50000>>BMP180_OSS);
	if(B7<0x80000000) p=(float)(B7*2)/B4;
	else p=(double)((float)B7/B4)*2.0;
	X1=(p/256)*(p/256);
	X1=(X1*3038)/65536;
	X2=(-7357*p)/65536;
	p=p+(float)(X1+X2+3791)/16;
	return p;
}

void BMP180_get_pressure()
{
/*	
	int g;
	long P_sum=0;
	for(g=0;g<10;g++) 
		{
		BMP180_read_temp();
		P_sum=P_sum+BMP180_read_pressure();
		}
	P=P_sum/10;
*/	
	//P=(p+P)/2;
	p=BMP180_read_pressure();
	P=k_alt*P+(1.0-k_alt)*p;
	
	P_stat[h]=P;
	h++;
	if(h==100)
	{
		int i;
		double P_avg=0,P_error=0,P_error2=0;
		for(i=0;i<100;i++)	P_avg=P_avg+P_stat[i];
		P_avg=P_avg/100;
		for(i=0;i<100;i++)
			{
			P_error=P_error+pow((P_stat[i]-P_avg),2);
			P_error2=P_error2+(P_avg-abs(P_stat[i]));
			}
		P_error=P_error/100;
		P_error=pow(P_error,((float)1/2));
		h=0;
	}
}

void get_start_altittude()
{
	int i;
	double P_start[100],P_sum=0;
	for(i=0;i<100;i++)
	{
		BMP180_read_temp();
		P_start[i]=BMP180_read_pressure();
		P_sum=P_sum+P_start[i];
	}
	P_sum=P_sum/100;
	start_altittude=44330*(1-pow((float)P_sum/101325,1/5.255));
	P=BMP180_read_pressure();
}

float BMP180_get_altittude()
{
	//BMP180_read_temp();
	//BMP180_read_pressure();
	//BMP180_get_pressure();
	P=k_alt*P+(1.0-k_alt)*p;
	absolute_altittude=44330*(1-pow((float)P/101325,1/5.255));
	relative_altittude=absolute_altittude-start_altittude;
	return absolute_altittude;
}
void BMP180_read_calibrate_data()
{
	char MSB,LSB,i;
	short AC[12];
	for(i=0;i<11;i++)
	{
		MSB=I2CReadRegister(BMP180_ADDR,170+(2*i));
		LSB=I2CReadRegister(BMP180_ADDR,171+(2*i));
		AC[i+1]=MSB<<8 | LSB;
	}
	AC1=AC[1];
	AC2=AC[2];
	AC3=AC[3];
	AC4=AC[4];
	AC5=AC[5];
	AC6=AC[6];
	B1=AC[7];
	B2=AC[8];
	MB=AC[9];
	MC=AC[10];
	MD=AC[11];
}

void init_BMP180()
{
	BMP180_read_calibrate_data(); //BMP180
	get_start_altittude();
}
