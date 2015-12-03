/*
 * SDcard.h
 *
 *  Created on: Aug 16, 2014
 *      Author: Norbi
 */

#ifndef SDCARD_H_
#define SDCARD_H_
#include "ff.h"

/******************************* SD Card Standard Commands **********************************/
#define SD1_CMD0  (0x40+0)              /* Resets the SD Memory Card */
#define SD1_CMD1  (0x40+1)              /* Sends host capacity support information and activates the card's
                                           initialization process. HCS is effective when card receives SEND_IF_COND
                                           command. Reserved bits shall be set to '0'. */
#define SD1_CMD6  (0x40+6)              /* Checks switchable function (mode 0) and switches card function (mode 1).*/
#define SD1_CMD8  (0x40+8)              /* Sends SD Memory Card interface condition that includes host supply voltage
                                           information and asks the accessed card whether card can operate in supplied
                                           voltage range. Reserved bits shall be set to '0'.*/
#define SD1_CMD9  (0x40+9)              /* Asks the selected card to send its cardspecific data (CSD)*/
#define SD1_CMD10 (0x40+10)             /* Asks the selected card to send its card identification (CID) */
#define SD1_CMD12 (0x40+12)             /* Forces the card to stop transmission in Multiple Block Read Operation */
#define SD1_CMD13 (0x40+13)             /* Asks the selected card to send its status register. */
#define SD1_CMD16 (0x40+16)             /* Sets a block length (in bytes) for all following block commands (read and
                                           write) of a Standard Capacity Card. Block length of the read and write
                                           commands are fixed to 512 bytes in a High Capacity Card. The length of
                                           LOCK_UNLOCK command is set by this command in both capacity cards.*/
#define SD1_CMD17 (0x40+17)             /* Reads a block of the size selected by the SET_BLOCKLEN command.*/
#define SD1_CMD18 (0x40+18)             /* Continuously transfers data blocks from card to host until interrupted by a
                                           STOP_TRANSMISSION command.*/
#define SD1_CMD24 (0x40+24)             /* Writes a block of the size selected by the SET_BLOCKLEN command. */
#define SD1_CMD25 (0x40+25)             /* Continuously writes blocks of data until �Stop Tran� token is sent
                                          (instead �Start Block�).*/
#define SD1_CMD27 (0x40+27)             /* Programming of the programmable bits of the CSD. */
#define SD1_CMD28 (0x40+28)             /* If the card has write protection features, this command sets the write protection bit
                                           of the addressed group. The properties of write protection are coded in the card
                                           specific data (WP_GRP_SIZE). The High Capacity Card does not support this command.*/
#define SD1_CMD29 (0x40+29)             /* If the card has write protection features, this command clears the write protection
                                           bit of the addressed group. The High Capacity Card does not support this command. */
#define SD1_CMD30 (0x40+30)             /* If the card has write protection features, this command asks the card to send the
                                           status of the write protection bits.6 The High Capacity Card does not support this command. */
#define SD1_CMD32 (0x40+32)             /* Sets the address of the first write block to be erased.*/
#define SD1_CMD33 (0x40+33)             /* Sets the address of the last write block of the continuous range to be erased. */
#define SD1_CMD38 (0x40+38)             /* Erases all previously selected write blocks */
#define SD1_CMD42 (0x40+42)             /* Used to Set/Reset the Password or lock/unlock the card. A transferred data block includes
                                           all the command details - refer to Chapter 4.3.7. The size of the Data Block is defined
                                           with SET_BLOCK_LEN command. Reserved bits in the argument and in Lock Card Data Structure
                                           shall be set to 0. */
#define SD1_CMD55 (0x40+55)             /* Defines to the card that the next command is an application specific command
                                           rather than a standard command */
#define SD1_CMD56 (0x40+56)             /* Used either to transfer a Data Block to the card or to get a Data Block from the card
                                           for general purpose/application specific commands. In case of Standard Capacity SD
                                           Memory Card, the size of the Data Block shall be defined with SET_BLOCK_LEN command.
                                           Block length of this command is fixed to 512-byte in High Capacity Card. */
#define SD1_CMD58 (0x40+58)             /* Reads the OCR register of a card. CCS bit is assigned to OCR[30]. */
#define SD1_CMD59 (0x40+59)             /* Turns the CRC option on or off. A �1� in the CRC option bit will turn the option on,
                                           a �0� will turn it off */
#define SD1_ACMD41 (0x40+41)            /* SEND_OP_COND (SDC) */
#define SD1_ACMD13 (0x40+13)            /* SD_STATUS (SDC) */
#define SD1_ACMD23 (0x40+23)            /* SET_WR_BLK_ERASE_COUNT (SDC) */

FATFS FatFs;
FIL fil;       /* File object */

char init_SD();
char init_SDcard();
char write_disk(const uint8_t *buffer, uint64_t block_number);
char read_disk(BYTE *buffer, DWORD block_number, UINT offset, UINT count);
char write(const uint8_t*buffer, char start);
char read(BYTE *buffer,DWORD lenght);
void init_SD_FAIL();
void init_SD_block512Fail();
char command(char cmd, int arg);
FRESULT create_log(FIL* file);
void write_dT(char *data,int n,...);

#endif /* SDCARD_H_ */
