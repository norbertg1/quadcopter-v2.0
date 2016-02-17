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
	int A=0; int B=0; int C=0;int D=0;
	int x=190;
	
//	initLEDs();			//!!!!!-----MPU6050 plug off%!!!---!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//	LED_Toggle_RED;						
	_SLCDModule_TurnOffAllSegments();
	_SLCDModule_PrintString(".",5);		
	Delay_mS(1000);
//	LED_Toggle_RED;
	_SLCDModule_TurnOffAllSegments();
	_SLCDModule_PrintString("..",5);
	Delay_mS(1000);
//	LED_Toggle_RED;
	_SLCDModule_TurnOffAllSegments();
	_SLCDModule_PrintString("...",5);
	Delay_mS(1000);
//	LED_Toggle_RED;
	_SLCDModule_TurnOffAllSegments();
	_SLCDModule_PrintString("MAX",0);
	SetMotorPWM(999,999,999,999);	//2. Adjunk 100% PWM-et majd csatlakoztassuk a kalibrálni kivant ESC-et(tápellátása meg kell hogy legyen szakitva)
	Delay_mS(7000);						//3. Csatlakoztassuk a kalibralando ESCet a tapfeszültséghez
//	LED_On_RED;
	_SLCDModule_TurnOffAllSegments();
	_SLCDModule_PrintString("MIN",0);
	SetMotorPWM(1,1,1,1);
	Delay_mS(2000);
	_SLCDModule_TurnOffAllSegments();
	_SLCDModule_PrintString("END",0);
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
		for(j=0;j<135;j++)	//150
		{
			Delay_mS(5);
			SetMotorPWM(j,j,j,j);
		}
	}
}

void motor_identify()
{
	int i=0,j;
	while(1)
	{
		for(j=0;j<150;j++)	{					//150
			_SLCDModule_TurnOffAllSegments();
			_SLCDModule_PrintNumber(i%4+1,5);
			if(i%4==0)	SetMotorPWM(j,0,0,0);
			if(i%4==1)	SetMotorPWM(0,j,0,0);
			if(i%4==2)	SetMotorPWM(0,0,j,0);
			if(i%4==3)	SetMotorPWM(0,0,0,j);
			Delay_mS(5);
		}
		i++;
		//SetMotorPWM(175,0,0,0);
		Delay_mS(1000);
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
	if(number<0) return number=-number;
	else return number;
}

void error(FRESULT *fr)
{
	//LED_Off_RED
	//if(*fr==0)	Set_LEDPWM(0,0,5);
}

int main(void)
{	
	uint32_t 	cnt=0;
	uint8_t		cnt_1=0;
	UINT x=5;
	FRESULT fr;    /* FatFs return code */
	uint16_t p=0,t=0;
	char string_lcd[6];
	Init();
//Set_LEDPWM(0,0,0); helyett LCDre valami
//	calibrate_ESC();	//MPU6050 plug off!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//	calibrate_BATT_voltmeter();

//	fr=create_log(&fil);
//	fr=f_write(&fil," No.    A     B     C     D   acc.x acc.y acc.z res.x res.y res.z timer\r\n",70,&x);	
	
	Zero_Sensors();
	motor_test();
//	kalman_init(&x_kalmandata);
//	kalman_init(&y_kalmandata);
	a=1.0/(1+(1.0/400.0));
	dt=0.0025;
	a=0.9675;
	ADC0_SC1A = (0 | 0b1000000);		//start ADC conversion
	ADC1_SC1A = (0 | 0b1000000);
	enable_PID_interrupts

	_SLCDModule_TurnOffAllSegments();
	_SLCDModule_TurnOnFreescaleSign();
	
	while(1)
	{	  
		a=Kp_prev/Kp;
		if(cnt%10000==0){
			if(LOW_BATT_FLAG){
				cnt_1++;
				if(cnt_1%80==0)		{ _SLCDModule_TurnOffAllSegments(); _SLCDModule_PrintString("LOW BAT",0); }
				if(cnt_1%80==20)	{ _SLCDModule_TurnOffAllSegments();	_SLCDModule_PrintString("1:",0);	sprintf(string_lcd,"%d",(int)(bat1_volt*100)); _SLCDModule_PrintString(string_lcd,15); }
				if(cnt_1%80==40)	{ _SLCDModule_TurnOffAllSegments(); _SLCDModule_PrintString("2:",0);	sprintf(string_lcd,"%d",(int)(bat2_volt*100)); _SLCDModule_PrintString(string_lcd,15); }
				if(cnt_1%80==60)	{ _SLCDModule_TurnOffAllSegments(); _SLCDModule_PrintString("3:",0);	sprintf(string_lcd,"%d",(int)(bat3_volt*100)); _SLCDModule_PrintString(string_lcd,15); }
			}
			else{
				_SLCDModule_TurnOffAllSegments();
				if(errorSum_x<0) _SLCDModule_TurnOnClockSign();
				sprintf(string_lcd,"%d",(int)(errorSum_x*10));
				_SLCDModule_PrintString(string_lcd,0);	
				sprintf(string_lcd,"%d",(int)(errorSum_y*10));
				_SLCDModule_PrintString(string_lcd,25);
				
			}
		}
		cnt++;
		UART4_S1 &= UART_S1_OR_MASK;		//UARTra kell debuggolásnal, receive overrun
		error(&fr);

	}
}
