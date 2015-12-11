#include "init.h"

void initInterrupts()		//MAX priority is 3!
{
	enable_irq(INT_TPM1 - 16);		//PID Interrupt, Initialize the NVIC to enable the specified IRQ
	set_irq_priority (INT_TPM1 - 16, 1);	//Set priority 2
	enable_irq(INT_UART0-16);			//Interrupt from data on bleutooth module	
	set_irq_priority (INT_UART0 - 16, 0);
/*	enable_irq(INT_PORTA - 16);					//Interrupt from bluetooth module when lost signal
	set_irq_priority (INT_PORTA - 16, 3);*/
//	enable_irq(INT_ADC0 - 16);		// Initialize the NVIC to enable the specified IRQ
//   set_irq_priority (INT_ADC0 - 16, 4); //Set priority 4 */
	//enable_irq(INT_TPM2 - 16);		//SDcard log interrupt Initialize the NVIC to enable the specified IRQ
	//set_irq_priority (INT_TPM2 - 16, 3); //Set priority 3
}

void Init()
{
	InitClock();
	InitSysTick();
	//initSysTick(); //K20
//	InitADC();
//	InitADCpins();
//	DMA_ADC_init(DMA_BASE_PTR,ADC_SOURCE,CHANNEL_2,(uint32_t)(&ADC0_RA),(uint32_t)(&adc_data[0]),15,32); //atalakitas nagysaga 32bit??
	initLEDs();			//A panelon levo LEDek inicializalasa	vagy ez
	init_Motor_PWM();
	init_Ticker();		//FTM1 Timer idõmérésre használlom, csak bekapcsolom
	init_SPI0();		//SPI0 inicialzálása beálitasa
	init_SD();			//SD kártya inicializalasa
	Init_I2C();		//
	//RTC();				//RTC_TSR regiszter másodperc számláló bekapcsolása
	initTimer0();		//Timer for PID interrupt
	initTimer1();		//Timer for SDcard log interrupt 
	init_PWM_LED();		//A panelon levo LEDek PWM meghajtasa 	vagy ez
	InitUARTs();
//	DMA_uart_init(DMA_BASE_PTR,UART0_SOURCE,CHANNEL_1,(uint32_t)(&UART0_D),(uint32_t)(&uart_data),1,8);	//beallitani az interruptot még!!!
//	initBluetooth(); //Interrupt on lost signal
	initInterrupts();
	init_MPU6050();
//	init_BMP180();

}

/*
 * for K20
void initClock(void)
{
	SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK); // Enable clock gate for ports to enable pin routing 

	pll_init(8000000, LOW_POWER, CRYSTAL, 4, 25, MCGOUT);
}
*/
