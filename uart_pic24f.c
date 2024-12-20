/*------------------------------------------------------------------------/
/  UART control module for PIC24F                          (C)ChaN, 2010
/-------------------------------------------------------------------------/
/
/  Copyright (C) 2010, ChaN, all right reserved.
/
/ * This software is a free software and there is NO WARRANTY.
/ * No restriction on use. You can use, modify and redistribute it for
/   personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
/ * Redistributions of source code must retain the above copyright notice.
/
/-------------------------------------------------------------------------*/

#include "xc.h"
#include <p33Fxxxx.h> 
#include <libpic30.h> 
#include <string.h>
#include "pps.h"
#include "clock.h"
#include "uart_pic24f.h"

#define BUFFER_SIZE 256
#define _DI()		__asm__ volatile("disi #0x3FFF") // Disable global interrput for 0x3FFF machine cycles
#define _EI()		__asm__ volatile("disi #0")

static volatile int TxRun;		/* Tx running flag */
static volatile struct {
	int		ri, wi, ct;			/* Read index, Write index, Data counter */
	unsigned char	buff[BUFFER_SIZE];	/* FIFO buffer */
} TxFifo, RxFifo;




/* UART1 Rx interrupt ISR */

void __attribute__((interrupt, no_auto_psv)) _U2RXInterrupt (void)
{
	unsigned char d;
	int i;


	d = (unsigned char)U2RXREG;	/* Get received data */
	_U2RXIF = 0;				/* Clear Rx interrupt flag */
	i = RxFifo.ct;				/* Number of bytes in the FIFO */
	if (i < BUFFER_SIZE) {		/* Skip if FIFO is full */
		RxFifo.ct = ++i;
		i = RxFifo.wi;
		RxFifo.buff[i++] = d;	/* Store data into the FIFO */
		RxFifo.wi = i % BUFFER_SIZE;	/* Next write ptr */
	}
}




void __attribute__((interrupt, no_auto_psv)) _U2TXInterrupt (void)
{
	int i;


	_U2TXIF = 0;		/* Clear Tx interrupt flag */

	i = TxFifo.ct;		/* Number of data in the FIFO */
	if (i) {			/* If any data is available, pop a byte and send it. */
		TxFifo.ct = --i;
		i = TxFifo.ri;
		U2TXREG = TxFifo.buff[i++];		/* Send a byte */
		TxFifo.ri = i % BUFFER_SIZE;	/* Next read ptr */
	} else {			/* No data in the Tx FIFO */
		TxRun = 0;		/* Stop transmission sequense */
	}
}

/* Check number of bytes in the Rx FIFO */

int uart_test (void)
{
	return RxFifo.ct;	/* Returns number of bytes in the Rx FIFO */
}



/* Get a byte from Rx FIFO */

unsigned char uart_getc (void)
{
	unsigned char d;
	int i;


	while (!RxFifo.ct) ;		/* Wait while Rx FIFO empty */

	i = RxFifo.ri;				/* Get a byte from Rx FIFO */
	d = RxFifo.buff[i++];
	RxFifo.ri = i % BUFFER_SIZE;
	_DI();
	RxFifo.ct--;
	_EI();

	return d;
}



/* Put a byte into Tx FIFO */

void uart_putc (unsigned char d)
{
	int i;


	while (TxFifo.ct >= BUFFER_SIZE) ;	/* Wait while Tx FIFO is full */

	i = TxFifo.wi;		/* Put a data into the Tx FIFO */
	TxFifo.buff[i++] = d;
	TxFifo.wi = i % BUFFER_SIZE;
	_DI();
	TxFifo.ct++;
	if (!TxRun) {		/* If transmission sequense is not running, start the tx sequense */
		TxRun = 1;
		_U2TXIF = 1;	/* Force trigger Tx interrupt */
	}
	_EI();
}



/* Initialize UART module */

void uart_init (unsigned long bps)
{
    /* Disable UART2 Tx/Rx interrupts */
	_U2RXIE = 0;
	_U2TXIE = 0;
    
	/* Initialize UART2 */
	U2BRG = FCY / 16 / bps - 1;
	U2MODEbits.UARTEN = 1;
    U2STAbits.UTXEN = 1;

	/* Enable UART Tx/Rx interruptrs */
	_U2RXIE = 1;
	_U2TXIE = 1;
}


