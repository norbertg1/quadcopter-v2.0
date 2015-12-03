/*
 * BMP180.h
 *
 *  Created on: Oct 25, 2014
 *      Author: Norbi
 */

#ifndef BMP180_H_
#define BMP180_H_

#define BMP180_ADDR 0x77 // 7-bit address

#define	BMP180_REG_CONTROL 0xF4				//Ide kell irni a parancsot ami hatasara 
#define	BMP180_REG_RESULT 0xF6				//Ebbol a regiszterol ki lehet olvasni az erteket MSBt, +1(0xF7-ben) van az LSB
											//0xF8ban pedig az xLSB az extra pontossagert( MSB<<16+LSB<<8+xLSB)>>8-oss )
#define	BMP180_CMD_READ_TEMP 0x2E			//Read TEMP data
#define	BMP180_CMD_READ_PRES0 0x34			//Read pressure data with precision 0oversamples => 0.5m accuracy
#define	BMP180_CMD_READ_PRES1 0x74			//Read pressure data with precision 2oversamples => 0.4m accuracy
#define	BMP180_CMD_READ_PRES2 0xB4			//Read pressure data with precision 4oversamples => 0.3m accuracy
#define	BMP180_CMD_READ_PRES3 0xF4			//Read pressure data with precision 8oversamples => 0.25m accuracy

#define	BMP180_CMD_READ_PRESS 0x34			//Read pressure data with precision:
#define BMP180_OSS 0b11						//0b11 = 8oversamples => 0.5m accuracy
											//0b10 = 4oversamples => 0.4m accuracy
											//0b01 = 2oversamples => 0.3m accuracy
											//0b00 = 1oversamples => 0.25m accuracy
void BMP180_read_temp();
float BMP180_read_pressure();
void BMP180_read_temp_interrupt();
float BMP180_read_pressure_interrupt();
float BMP180_get_altittude();
void BMP180_read_calibrate_data();
void init_BMP180();

short AC1,AC2,AC3,B1,B2,MB,MC,MD;
unsigned short AC4,AC5,AC6;
long T;
float absolute_altittude,start_altittude,relative_altittude;
float P;

#define Get_Altittude	44330*(1-pow(p/1013.25,1/5.255))	//expression for calculate absolite altittude in m, 1013.25hPa is pressure at sea leve


#endif /* BMP180_H_ */
