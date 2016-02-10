/*
 * SDcard.c
 *
 *  Created on: Aug 7, 2014
 *      Author: Norbi
 */

#include "init.h"
#include "ff.h"

#define COMMAND_TIMEOUT	5000

#define SDCARD_FAIL	0
#define SDCARD_V1	1
#define SDCARD_V2	4

	FATFS FatFs;
	FIL fil;       /* File object */


/*
 * SD card						KL25Z-FRDM
 * 		pin 1(CS)-------------PORTC9
 * 		pin 2(MOSI)-----------PORTE1
 * 		pin 3(GND)------------GND
 * 		pin 4(VCC)------------3.3V
 * 		pin 5(SCK)------------PORTE2
 * 		pin 6(GND)------------NOT CONNECTED
 * 		pin 7(MISO)-----------PORTE3
 * 		pin 8-----------------NOT CONNECTED
 * 		pin 9-----------------NOT CONNECTED 
 * 		
 * 		KL25 VERSION
*/



char command(char cmd, int arg)
{
	uint16 i;
	CS_LOW
	spi_send( cmd );		//
	spi_send(arg>>24);
	spi_send(arg>>16);
	spi_send(arg>>8);
	spi_send(arg>>0);
	spi_send(0x95);
	
	for(i=0;i<COMMAND_TIMEOUT;i++)
	{
		char response=spi_send(0xFF);
		if(!(response & 0x80))
		{
			CS_HIGH
			spi_send(0xFF);
			return response;
		}
	}
	CS_HIGH
	spi_send(0xFF);
	return -1;	
}
int command58()						//return OCR value
{
	int i,ocr;
	CS_LOW
	spi_send(SD1_CMD58);
	spi_send(0);
	spi_send(0);
	spi_send(0);
	spi_send(0);
	spi_send(0x95);
	for(i=0;i<COMMAND_TIMEOUT;i++)
	{
		if(!(spi_send(0xFF) & 0x80))
		{
			ocr = spi_send(0xFF)<<24;
			ocr |= spi_send(0xFF)<<16;
			ocr |= spi_send(0xFF)<<8;
			ocr |= spi_send(0xFF);
			CS_HIGH
			spi_send(0xFF);
			return ocr;
		}
	}
	CS_HIGH
	spi_send(0xFF);
	return -1;
	
}
char command8()
{
	int i;
	char response[5];
	CS_LOW
	spi_send(0x48);
	spi_send(0);
	spi_send(0);
	spi_send(0x01);
	spi_send(0xAA);
	spi_send(0x87);
	
	for(i=0;i<COMMAND_TIMEOUT*1000;i++)
	{
		response[0]=spi_send(0xFF);
		if(!(response[0]==0x80))
		{
			for(i=0;i<5;i++)	response[i]=spi_send(0xFF);
			CS_HIGH
			spi_send(0xFF);
			return response[0];
		}
	}
	CS_HIGH
	spi_send(0xFF);
	return -1;
}

char init_SD_v1()
{
	int i;
	for(i=0;i<COMMAND_TIMEOUT;i++)
	{
		command(55,0);
		if(command(SD1_CMD1,0)==0) return SDCARD_V1;
	}
	return SDCARD_FAIL;
}
char init_SD_v2()
{
	int i,temp;
	for(i=0;i<COMMAND_TIMEOUT/100;i++)
	{
		Delay_mS(50);
		command58();
		temp=command(SD1_CMD55,0);
		temp=command(SD1_ACMD41,0x40000000);
		if(temp==0)
		{
			command58();
			return SDCARD_V2;
		}
	}
	return SDCARD_FAIL;
}
char init_SDcard()
{
	char i,r,j=0;
	while(j<3)
	{
		Delay_mS(100);
		CS_HIGH
		Delay_mS(100);
		for(i=0;i<11;i++)	//74 or more clock pulses to SCLK, when MOSI and CS is HIGH
			{
			spi_send(255);
			}
		CS_LOW
		Delay_mS(100);
		r=command(SD1_CMD0,0);
		if(r!=0x00000001) goto ide;//return SDCARD_FAIL;
		r=command8();
		if(r==0x00000001) return init_SD_v2();
		if(r==0x00000101) return init_SD_v1();
	ide:j++;
	}
	return SDCARD_FAIL;
}
char init_SD()		//piros villogsas jelzi a hibat!!!
{
	char r=init_SDcard();
	if(!r) init_SD_FAIL();
	if(command(SD1_CMD16,0x200)) init_SD_block512Fail();
	//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------	
		//Define SPI clock speed:
		#define BAUD_DIV 0 /* 24MHz SPI */
		//#define BAUD_DIV 1 /* 12MHz SPI */
		//#define BAUD_DIV 2 /* 8MHz SPI */
		//#define BAUD_DIV 3 /* 6MHz SPI */
		//#define BAUD_DIV 4 /* 3MHz SPI */
		SPI1_CTAR0 = SPI_CTAR_FMSZ(7) | SPI_CTAR_PBR(0) | SPI_CTAR_BR(BAUD_DIV) | SPI_CTAR_CSSCK(BAUD_DIV) | SPI_CTAR_DBR_MASK;
		SPI1_CTAR1 = SPI_CTAR_FMSZ(15) | SPI_CTAR_PBR(0) | SPI_CTAR_BR(BAUD_DIV) | SPI_CTAR_CSSCK(BAUD_DIV) | SPI_CTAR_DBR_MASK;
	//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//	SPI0_BR = (SPI_BR_SPPR(0x00) | SPI_BR_SPR(0x01));		//baud rate prescale divisor to 0 & set baud rate divisor to 1 for baud rate of: (0+1)*2^(0x01+1)=4; 24Mhz/4=6MHz ~187.5KB/s szamitasaim szerint  
	return r;
}
void init_SD_FAIL()
{
	int i;
	for(i=0;i<10;i++)
	{
		LED_Toggle_RED;
		Delay_mS(200);		
	}
}
void init_SD_block512Fail()
{
	int i;
	for(i=0;i<5;i++)
	{
		LED_Toggle_RED;
		Delay_mS(100);
		LED_Toggle_RED;
		Delay_mS(300);
	}
}

//if start=0xFE - single data block, 0xFC - next data block in multiple data block mode
// 0xFD - stop data transfer in multiple data block mode
// return 0 - error; return 1 - succesful
char write(const uint8_t*buffer, char start) 
{											
	int i;									
	CS_LOW
    // indicate start of block
    spi_send(start);
    
    if(start==0xFD) return 1;
	
	// write the data
    for (i = 0; i < 512; i++) {
        spi_send(buffer[i]);
    }
    
    // write the checksum
    spi_send(0xFF);
    spi_send(0xFF);
    
    // check the response token
    if ((spi_send(0xFF) & 0x1F) != 0x05) 
    {
    	CS_HIGH
        spi_send(0xFF);
        return 0;
    }
    
    // wait for write to finish
    while (spi_send(0xFF) == 0);
    
    CS_HIGH
    spi_send(0xFF);
    return 1;
}
char write_disk(const uint8_t *buffer, uint64_t block_number)		//SDv1 nel lehet nem mukodik!!!!
	{	
// set write address for single block (CMD24)
    if (command(SD1_CMD24, block_number * 512) != 0) {
        return 1;
    }
    
    // send the data block
    write(buffer, 0xFE);
    return 0;
	}
char read(BYTE *buffer,DWORD lenght)
{
	int i;
	CS_LOW
	while (spi_send(0xFF) != 0xFE);
	    
	    // read data
		for (i = 0; i < lenght; i++) buffer[i] = spi_send(0xFF);
	    spi_send(0xFF); // checksum
	    spi_send(0xFF);
	    
	    CS_HIGH
	    spi_send(0xFF);
	    return 1;
}

char read_disk(BYTE *buffer, DWORD block_number, UINT offset, UINT count)
{
	if (command(SD1_CMD17, block_number * 512) != 0) 
	{
		return 1;
	}
	    
	// receive the data
	read(buffer, 512);
	return 0;
}
char set_block_512()
{
	return command(SD1_CMD16,0x200);
}
/*
void Create_logfile(FILE *fil)
{
	
	
}*/


FRESULT create_log(FIL* file)
{
	UINT sequence=1;
	char str[2],filename[7];
	FRESULT fr;
	fr=f_mount(&FatFs, " ", 0);
	if(fr)	return fr;
	fr=1;
	while(fr!=0 && !(sequence>99) && fr!=FR_NOT_READY)			//ha létezik n.txt nevu fajl letrehozza (n+1).txt nevut
	{
		sprintf(str,"%d",sequence);	//sequencebol str nevu string lesz
		snprintf(filename, sizeof(filename), "%s.txt", str); //a filename valami olyasmi lesz hogy 1.txt, 2.txt.....
		fr=f_open(file,filename,FA_CREATE_NEW);
		sequence++;
	}
	fr=f_open(&fil,filename,FA_WRITE);
	return fr;
}
void write_dT(char *data,int n,...)
{
	int i, adat;
	va_list lista;
	data[0]=0;
	va_start(lista, n);
	for(i=0;i<n;i++)
	{
		adat=va_arg(lista, int);
		if(adat>99999) adat=99999;
		else if(adat<-9999) adat=-9999;
		sprintf(data + strlen(data), "%05d ", adat);
	}
}
