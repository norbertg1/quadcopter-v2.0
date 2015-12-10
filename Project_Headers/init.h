/*
 * INIT.h
 *
 *  Created on: AUG 1, 2014
 *      Author: Norbi
 */

#ifndef INIT_H_
#define INIT_H_

#define KL25Z

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "stdarg.h"
#include <math.h>
//K20
#ifdef K20
#include <MK20D5.h>
#include "config\arm_cm4.h"
#include "config\sysTick.h"
#include "config\BoardSupport.h"
#include "config\clock.h"
#endif

//KL25Z
#ifdef KL25Z
#include "processor_specific_functions\KL25Z\sys\arm_cm0.h"
#include "processor_specific_functions\KL25Z\sys\ARM_SysTick.h"
//#include "processor_specific_functions\KL25Z\sys\bme.h"
#include "processor_specific_functions\KL25Z\sys\CrystalClock.h"
#include "processor_specific_functions\KL25Z\sys\derivative.h"
#include "processor_specific_functions\KL25Z\sys\MKL25Z4.h"

#include "processor_specific_functions/KL25Z/i2c.h"
#include "processor_specific_functions/KL25Z/misc.h"
#include "processor_specific_functions/KL25Z/PWM.h"
#include "processor_specific_functions/KL25Z/SPI.h"
#include "processor_specific_functions/KL25Z/uart.h"
#include "processor_specific_functions/KL25Z/ADC.h"
#include "processor_specific_functions/KL25Z/dma.h"
#endif

#include "interrupts.h"
#include "main.h"
#include "Motor.h"
#include "SDcard.h"
#include "diskio.h"//FATfs
#include "MPU6050.h"
#include "BMP180.h"

void Init();
void initClock();
int abs(int number);

#endif
