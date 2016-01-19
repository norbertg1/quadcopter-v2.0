/*
 * main implementation: use this 'C' sample to create your own application
 *
 */



#include "main.h"
#include "init.h"

void Zero_Sensors()
{
	float BUFFER_XANGLE = 0;
	float BUFFER_YANGLE = 0;
	int x = 0;
	for(x=0; x<100; x++)
	{
		Get_Accel_Values();
		Get_Angles();
		BUFFER_XANGLE += ACCEL_XANGLE;
		BUFFER_YANGLE += ACCEL_YANGLE;
		Delay_mS(1);
	}
	COMPLEMENTARY_XANGLE = BUFFER_XANGLE/100.0;
	COMPLEMENTARY_YANGLE = BUFFER_YANGLE/100.0;
	GYRO_XANGLE = BUFFER_XANGLE/100.0;
	GYRO_YANGLE = BUFFER_YANGLE/100.0;
}
void calibrate_ESC()						//ESC kalibrasa, 1. Az MCU-t ne a kalibralando ESCrol taplaljuk
{
	initLEDs();			//!!!!!-----MPU6050 plug off%!!!---!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	LED_Toggle_RED;						
	Delay_mS(100);						
	LED_Toggle_RED;						
	Delay_mS(100);
	LED_Toggle_RED;
	Delay_mS(100);
	LED_Toggle_RED;
	SetMotorPWM(999,999,999,999);	//2. Adjunk 100% PWM-et majd csatlakoztassuk a kalibrálni kivant ESC-et(tápellátása meg kell hogy legyen szakitva)
	Delay_mS(7000);						//3. Csatlakoztassuk a kalibralando ESCet a tapfeszültséghez
	LED_On_RED;
	SetMotorPWM(1,1,1,1);
	Delay_mS(2000);
	int A=0,B=0,C=0,D=0;
	int x=190;
	while(1)							//4. Varjuk ki még lefut a program
	{
		SetMotorPWM(A,B,C,D);
		A++,B++,C++,D++;
		Delay_mS(50);
		if(A>x) A=0,B=0,C=0,D=0;
	}
}
void motor_test()
{
	int i,j;
	for(i=0;i<4;i++)
	{
		for(j=0;j<150;j++)
		{
			Delay_mS(5);
			//if(i==0) SetMotorPWM(j,0,0,0);
			//if(i==1) SetMotorPWM(0,j,0,0);
			//if(i==2) SetMotorPWM(0,0,j,0);
			//if(i==3) SetMotorPWM(0,0,0,j);
			SetMotorPWM(j,j,j,j);
		}
	}
}
void motor_start(int basepower)
{
	SetMotorPWM(75,75,75,75);
	Delay_mS(100);
	SetMotorPWM(84,84,84,84);
	Delay_mS(2000);
	SetMotorPWM(75,75,75,75);
	Delay_mS(1000);

/*	while(A<Convert_FORCEtoPWM(0.75*basepower))
	{
		SetMotorPWM(A,B,C,D);
		A++,B++,C++,D++;
		Delay_mS(25);
	}*/
}

void calibrate_BATT_voltmeter()
{
	int i;
	while(1){
		long temp1=0,temp2=0,temp3=0;
		for(i=0;i<1000;i++)
		{
			temp1=temp1+Read_ADC0(23);
			Delay_mS(1);
		}
		temp1=temp1/1000;
		for(i=0;i<1000;i++)
		{
			temp2=temp2+Read_ADC0(21);
			Delay_mS(1);
		}
		temp2=temp2/1000;
		for(i=0;i<1000;i++)
		{
			temp3=temp3+Read_ADC0(0);
			Delay_mS(1);
		}
		temp3=temp3/1000;
	}
}

void bluetoothSetup(void)
{
	uart_transmitdata(UART0_BASE_PTR,"AT");
    Delay_mS(1000);
    uart_transmitdata(UART0_BASE_PTR,"AT+NAMEquadcopter");
    Delay_mS(1000);
    uart_transmitdata(UART0_BASE_PTR,"AT+PIN1234");
    Delay_mS(1000);
    uart_transmitdata(UART0_BASE_PTR,"AT+BAUD8");
}

int abs(int number)
{
	if(number<0) return number=-number*-1;
	else return number;
}

void error(FRESULT *fr)
{
	//LED_Off_RED
	//if(*fr==0)	Set_LEDPWM(0,0,5);
}

int main(void)
   {	
	Init();
	//Set_LEDPWM(0,0,0); helyett LCDre valami
//	calibrate_ESC();	//MPU6050 plug off!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//	calibrate_BATT_voltmeter();
	UINT x=5;
	FRESULT fr;    /* FatFs return code */
	
	fr=create_log(&fil);
	fr=f_write(&fil," No.    A     B     C     D   acc.x acc.y acc.z res.x res.y res.z timer\r\n",70,&x);	
	
	Zero_Sensors();
	motor_test();
	a=1.0/(1+(1.0/400.0));
	dt=0.0025;
	
	ADC0_SC1A = (8 | 0b1000000);		//start ADC conversion
	enable_PID_interrupts
	enable_SDcard_interrupts
	uint16_t p=0,t=0;
	while(1)
	{	  
		UART1_S1 &= UART_S1_OR_MASK;		//UARTra kell debuggolásnal, receive overrun
		error(&fr);
		t++;
		//Set_LEDPWM(0,0,p);
		SDcardw_Interrupt();
	//	ADC();
	//	FTM0_C4V=(15600/2)-200*COMPLEMENTARY_YANGLE;	//GREEN LED PWM gyro+acc
	//	FTM0_C2V=(15600/2)+200*COMPLEMENTARY_YANGLE;	//RED LED PWM	gyro+acc
	//	FTM0_C4V=(15600/2)-200*(relative_altittude-setpoint_alt)*10;	//GREEN LED PWM press
	//	FTM0_C2V=(15600/2)+200*(relative_altittude-setpoint_alt)*10;	//RED LED PWM	press

	}
}
