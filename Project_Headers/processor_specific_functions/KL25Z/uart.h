/*
 * UART.h
 *
 *  Created on: Mar 18, 2015
 *      Author: Norbi
 */

#ifndef UART_H_
#define UART_H_

#include <stdint.h>

#define SDA_SERIAL_BAUD		115200
#define SDA_SERIAL_OUTGOING_QUEUE_SIZE	2048
#define SDA_SERIAL_INCOMING_QUEUE_SIZE	128

#define QUEUE_FULL       -1
#define QUEUE_EMPTY      -2
#define QUEUE_OK          0

typedef struct {
    
	uint16_t ReadPtr;
    uint16_t WritePtr;
    uint16_t QueueSize;
    uint8_t *QueueStorage;
    
} ByteQueue;


void InitUARTs();
void UART_Process();
void uart_putchar (UART_MemMapPtr channel, char ch);
char uart_getchar (UART_MemMapPtr channel);
int uart_getchar_present (UART_MemMapPtr channel);
void uart_transmitdata(UART_MemMapPtr channel,char TransData[]);

extern ByteQueue SDA_SERIAL_OUTGOING_QUEUE;
extern ByteQueue SDA_SERIAL_INCOMING_QUEUE;

uint8_t SDA_SERIAL_OUTGOING_QUEUE_Storage[SDA_SERIAL_OUTGOING_QUEUE_SIZE];
uint8_t SDA_SERIAL_INCOMING_QUEUE_Storage[SDA_SERIAL_INCOMING_QUEUE_SIZE];

#endif /* UART_H_ */
