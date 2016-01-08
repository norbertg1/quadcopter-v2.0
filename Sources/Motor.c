#include "init.h"
//#include "derivative.h"

void SetMotorPWM(int MotorA , int MotorB, int MotorC, int MotorD)
{
	
	if(MotorA>MOTOR_MAX)
		MotorA = MOTOR_MAX;
	else if(MotorA<=MOTOR_MIN)
		MotorA = MOTOR_MIN;
	
	if(MotorB>MOTOR_MAX)
			MotorB = MOTOR_MAX;
		else if(MotorB<MOTOR_MIN)
			MotorB = MOTOR_MIN;
	
	if(MotorC>MOTOR_MAX)
		MotorC = MOTOR_MAX;
		else if(MotorC<MOTOR_MIN)
			MotorC = MOTOR_MIN;
	
	if(MotorD>MOTOR_MAX)
		MotorD = MOTOR_MAX;
		else if(MotorD<MOTOR_MIN)
			MotorD = MOTOR_MIN;
#if MOTOR	
	//ELEMENTARY_PROC_TIME*((ESC_MINIMUM_POWER+(ESC_POWER_DIFFERENCE)*((float)MotorA/1000))/1000);			//Set PWM
	TPM0_C0V = ELEMENTARY_PROC_TIME*((ESC_MINIMUM_POWER+(ESC_POWER_DIFFERENCE)*((float)MotorA/1000))/1000);
	TPM0_C2V = ELEMENTARY_PROC_TIME*((ESC_MINIMUM_POWER+(ESC_POWER_DIFFERENCE)*((float)MotorB/1000))/1000);
	TPM0_C5V = ELEMENTARY_PROC_TIME*((ESC_MINIMUM_POWER+(ESC_POWER_DIFFERENCE)*((float)MotorC/1000))/1000);
	TPM0_C4V = ELEMENTARY_PROC_TIME*((ESC_MINIMUM_POWER+(ESC_POWER_DIFFERENCE)*((float)MotorD/1000))/1000);
#else
	TPM0_C1V = 0;
	TPM0_C2V = 0;
	TPM0_C0V = 0;
	TPM0_C4V = 0;
#endif
}

float Convert_FORCEtoPWM(float force)
{
	return -0.0006*force*force+1.607*force+58.149;
	return 8.6518*pow(force,0.7005);	//Kiserletileg meghatarozott, lassu
}
