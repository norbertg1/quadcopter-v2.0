/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2013        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control module to the FatFs module with a defined API.        */
/*-----------------------------------------------------------------------*/

#include "diskio.h"		/* FatFs lower layer API */
//#include "usbdisk.h"	/* Example: USB drive control */
//#include "atadrive.h"	/* Example: ATA drive control */
//#include "derivative.h" /* include peripheral declarations */
#include "init.h"

/* Definitions of physical drive number for each media */
#define ATA		0
#define MMC		1
#define USB		2





/*-----------------------------------------------------------------------*/
/* Wait for card ready                                                   */
/*-----------------------------------------------------------------------*/

static
int wait_ready (	/* 1:Ready, 0:Timeout */
	UINT wt			/* Timeout [ms] */
)
{
	BYTE d;


	wt = wt / 10;
	do{
		//Delay_mS(10);
		wt--;
		d = spi_send(0xFF);
	}while (d != 0xFF && wt);
	
	return (d == 0xFF) ? 1 : 0;
}



/*-----------------------------------------------------------------------*/
/* Deselect the card and release SPI bus                                 */
/*-----------------------------------------------------------------------*/

static
void deselect (void)
{
	CS_HIGH;
	spi_send(0xFF);	/* Dummy clock (force DO hi-z for multiple slave SPI) */
}



/*-----------------------------------------------------------------------*/
/* Select the card and wait for ready                                    */
/*-----------------------------------------------------------------------*/

static
int select (void)	/* 1:Successful, 0:Timeout */
{
	CS_LOW;
	spi_send(0xFF);	/* Dummy clock (force DO enabled) */

	if (wait_ready(500)) return 1;	/* OK */
	deselect();
	return 0;	/* Timeout */
}




/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber (0..) */
)
{

/*
	switch (pdrv) {
	case ATA :
		result = ATA_disk_initialize();

		// translate the reslut code here

		return stat;

	case MMC :
		result = MMC_disk_initialize();

		// translate the reslut code here

		return stat;

	case USB :
		result = USB_disk_initialize();

		// translate the reslut code here

		return stat;
	}
*/
	if(init_SDcard()) return 0;
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber (0..) */
)
{
	
/*
	switch (pdrv) {
	case ATA :
		result = ATA_disk_status();

		// translate the reslut code here

		return stat;

	case MMC :
		result = MMC_disk_status();

		// translate the reslut code here

		return stat;

	case USB :
		result = USB_disk_status();

		// translate the reslut code here

		return stat;
	}
*/
//	if (pdrv) return STA_NOINIT();
//	return STA_NOINIT;
	return 0;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address (LBA) */
	UINT count		/* Number of sectors to read (1..128) */
)
{
	
	BYTE cmd;


	if (pdrv || !count) return RES_PARERR;
//	if (Stat & STA_NOINIT) return RES_NOTRDY;

	sector *= 512;	/* Convert to byte address if needed if (!(CardType & CT_BLOCK)) sector *= 512; */

	cmd = count > 1 ? SD1_CMD18 : SD1_CMD17;			/*  READ_MULTIPLE_BLOCK : READ_SINGLE_BLOCK */
	if (command(cmd, sector) == 0) {
		do {
			if (!read(buff, 512)) break;
			buff += 512;
		} while (--count);
		if (cmd == SD1_CMD18) command(SD1_CMD12, 0);	/* STOP_TRANSMISSION */
	}
//	deselect();

	return count ? RES_ERROR : RES_OK;

/*
	switch (pdrv) {
	case ATA :
		// translate the arguments here

		result = ATA_disk_read(buff, sector, count);

		// translate the reslut code here

		return res;

	case MMC :
		// translate the arguments here

		result = MMC_disk_read(buff, sector, count);

		// translate the reslut code here

		return res;

	case USB :
		// translate the arguments here

		result = USB_disk_read(buff, sector, count);

		// translate the reslut code here

		return res;
	}
*/
	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _USE_WRITE
DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber (0..) */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address (LBA) */
	UINT count			/* Number of sectors to write (1..128) */
)
{


	if (pdrv || !count) return RES_PARERR;
//	if (Stat & STA_NOINIT) return RES_NOTRDY;
//	if (Stat & STA_PROTECT) return RES_WRPRT;

	sector *= 512;	/* Convert to byte address if needed if (!(CardType & CT_BLOCK)) sector *= 512; */

	if (count == 1) {	/* Single block write */
		if ((command(SD1_CMD24, sector) == 0)	/* WRITE_BLOCK */
			&& write(buff, 0xFE))
			count = 0;
	}
	else {				/* Multiple block write */
		command(SD1_ACMD23, count);	//		if (CardType & CT_SDC) send_cmd(ACMD23, count);
		if (command(SD1_CMD25, sector) == 0) {	/* WRITE_MULTIPLE_BLOCK */
			do {
				if (!write(buff, 0xFC)) break;
				buff += 512;
			} while (--count);
			if (!write(0, 0xFD))	/* STOP_TRAN token */
				count = 1;
		}
	}
	deselect();

	return count ? RES_ERROR : RES_OK;
}
/*
	switch (pdrv) {
	case ATA :
		// translate the arguments here

		result = ATA_disk_write(buff, sector, count);

		// translate the reslut code here

		return res;

	case MMC :
		// translate the arguments here

		result = MMC_disk_write(buff, sector, count);

		// translate the reslut code here

		return res;

	case USB :
		// translate the argume nts here

		result = USB_disk_write(buff, sector, count);

		// translate the reslut code here

		return res;
	}
*/
//	return RES_PARERR;
//}
#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

#if _USE_IOCTL
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{


	DRESULT res;
	BYTE n, csd[16];
	DWORD csize;


	if (pdrv) return RES_PARERR;

	res = RES_ERROR;

//	if (Stat & STA_NOINIT) return RES_NOTRDY;

	switch (cmd) {
	case CTRL_SYNC :		/* Make sure that no pending write process. Do not remove this or written sector might not left updated. */
		if (select()) res = RES_OK;
		break;

	case GET_SECTOR_COUNT :	/* Get number of sectors on the disk (DWORD) */
		if ((command(SD1_CMD9, 0) == 0) && read(csd, 16)) {
			if ((csd[0] >> 6) == 1) {	/* SDC ver 2.00 */
				csize = csd[9] + ((WORD)csd[8] << 8) + ((DWORD)(csd[7] & 63) << 16) + 1;
				*(DWORD*)buff = csize << 10;
			} else {					/* SDC ver 1.XX or MMC*/
				n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
				csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
				*(DWORD*)buff = csize << (n - 9);
			}
			res = RES_OK;
		}
		break;

	case GET_BLOCK_SIZE :	/* Get erase block size in unit of sector (DWORD) */
		if (1) {	/* SDv2? if (CardType & CT_SD2) { */
			if (command(SD1_ACMD13, 0) == 0) {	/* Read SD status */
				spi_send(0xFF);
				if (read(csd, 16)) {				/* Read partial block */
					for (n = 64 - 16; n; n--) spi_send(0xFF);	/* Purge trailing data */
					*(DWORD*)buff = 16UL << (csd[10] >> 4);
					res = RES_OK;
				}
			}
		} else {					/* SDv1 or MMCv3 */
			if ((command(SD1_CMD9, 0) == 0) && read(csd, 16)) {	/* Read CSD */
				if (0) {	/* SDv1 if (CardType & CT_SD1) { */
					*(DWORD*)buff = (((csd[10] & 63) << 1) + ((WORD)(csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
				} else {					/* MMCv3 */
					*(DWORD*)buff = ((WORD)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
				}
				res = RES_OK;
			}
		}
		break;

	/* Following commands are never used by FatFs module */







	default:
		res = RES_PARERR;
	}

	deselect();

	return res;


/*
	switch (pdrv) {
	case ATA :
		// pre-process here

		result = ATA_disk_ioctl(cmd, buff);

		// post-process here

		return res;

	case MMC :
		// pre-process here

		result = MMC_disk_ioctl(cmd, buff);

		// post-process here

		return res;

	case USB :
		// pre-process here

		result = USB_disk_ioctl(cmd, buff);

		// post-process here

		return res;
	}
*/

	return RES_PARERR;
}
#endif
