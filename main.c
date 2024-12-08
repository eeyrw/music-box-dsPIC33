/*
 * File:   newmainXC16.c
 * Author: yuan
 *
 * Created on July 15, 2023, 4:09 PM
 */


#include "xc.h"
#include <p33Fxxxx.h>
#include "clock.h"  
#include <libpic30.h> 
#include <string.h>
#include "pps.h"
#include "uart_pic24f.h"
#include "xprintf.h"
#include "ff.h"

// FOSCSEL
#pragma config FNOSC = PRIPLL                         // Oscillator Mode (Internal Fast RC (FRC))
#pragma config IESO = ON                // Two-speed Oscillator Start-Up Enable (Start up with FRC, then switch)

// FOSC
#pragma config POSCMD = XT            // Primary Oscillator Source (Primary Oscillator Enabled)
#pragma config OSCIOFNC = ON           // OSC2 Pin Function (OSC2 pin has digital IO)
#pragma config IOL1WAY = ON
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are disabled)

// FWDT
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)

// FICD
#pragma config ICS = PGD2               // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)

volatile UINT Timer;	/* 1kHz increment timer */
volatile WORD rtcYear = 2024;
volatile BYTE rtcMon = 5, rtcMday = 14, rtcHour, rtcMin, rtcSec;

#define CODEC_SAMPLE_RATE 32000
#define DCI_BCG_VALUE (((FCY/32)/CODEC_SAMPLE_RATE)-1)
#define FRAME 8
short txBufferA[FRAME] __attribute__((space(dma)));
short txBufferB[FRAME] __attribute__((space(dma)));

volatile int rxBufferIndicator = 0;
void DCIInit(void);
void processRxData(int * sourceBuffer, int * targetBuffer);
void DMAInit(void);

int main(void) {

    //  PLL??????? ------------------------------------
    PLLFBD = 38; // M = 40  8MHz x40/2/2
    CLKDIVbits.PLLPOST = 0; // N2 = 2          = 80MHz
    CLKDIVbits.PLLPRE = 0; // N1 = 2
    while (OSCCONbits.LOCK != 1); // PLL????LOCK)???
    PPSUnLock;
    _U1RXR = 13; //UART1 RX ON PB13
    _U2RXR = 19; //UART2 RX ON PC3
    _RP0R = 0; //SPI2 CS ON PB0 as normal GPIO
    _RP25R = 0b01111; //DCI Frame Sync Output (LRCLK for I2S) on PC9
    _RP1R = 0b01101; //DCI Serial Data Output (DIN for I2S) on PB1
    _RP14R = 0b01110; //DCI Serial Clock Output (BCLK for I2S) on PB14
    _RP21R = 0b00011; //UART1 TX on PC5
    _RP20R = 0b00101; //UART2 TX on PC4
    _RP24R = 0b01010; //SPI2 SDO on PC8
    _SDI2R = 23; //SPI2  on PC7
    //_RP17R=0b10011; //PWM OC2 on PC1
    //_RP16R=0b10010; //PWM OC1 on PC0
    PPSLock;


    AD1PCFGLbits.PCFG3 = 1;
    AD1PCFGLbits.PCFG9 = 1;
    AD1PCFGLbits.PCFG10 = 1;
    AD1PCFGLbits.PCFG11 = 1;
    TRISC = 0x00; // make sure PWM pins are set to be outputs
    _TRISB4 = 0;
    _LATB4 = 1;

//    RxWptr = 0; // ??????????????
//    RxRptr = 0; // ??????????????
//    //_PCFG5=1;                         // SW Port????????
//    U1BRG = (FCY / (16 * 115200)) - 1;
//    U1MODEbits.STSEL = 0; // 1-Stop bit
//    U1MODEbits.PDSEL = 0; // No Parity, 8-Data bits
//    U1MODEbits.ABAUD = 0; // Auto-Baud disabled
//    U1MODEbits.BRGH = 0; // Standard-Speed mode
//
//    U1MODEbits.UARTEN = 1; // Enable UART
//    U1STAbits.UTXEN = 1; // Enable UART TX
//    __delay_ms(200); // UART?????????
//    //_U1RXIP = 4; // Rx??????4??? (???)
//    //_U1RXIF = 0; // Rx??????????
//    //_U1RXIE = 1; // Rx?????
    
    
    uart_init(115200);
    DMAInit();
    DCIInit();
    
    	/* Start Timer1 in interval time of 1ms */
	PR1 = FCY / 8 / 1000;
	_TCKPS0 = 1;	/* Select prescaler Fcy/8 */
	_TON = 1;		/* Start Timer1 */
	_T1IE = 1;		/* Enable Timer1 interrupt */
    
    xdev_in(uart_getc);		/* Join UART and console */
	xdev_out(uart_putc);
    for (int i = 0; i < FRAME / 8; i++) {
        txBufferA[i * 8] = 1;
        txBufferA[i * 8 + 1] = 2;
        txBufferA[i * 8 + 2] = 3;
        txBufferA[i * 8 + 3] = 4;
        txBufferA[i * 8 + 4] = 5;
        txBufferA[i * 8 + 5] = 6;
        txBufferA[i * 8 + 6] = 7;
        txBufferA[i * 8 + 7] = 8;

        txBufferB[i * 8] = 9;
        txBufferB[i * 8 + 1] = 10;
        txBufferB[i * 8 + 2] = 11;
        txBufferB[i * 8 + 3] = 12;
        txBufferB[i * 8 + 4] = 13;
        txBufferB[i * 8 + 5] = 14;
        txBufferB[i * 8 + 6] = 15;
        txBufferB[i * 8 + 7] = 16;
    }
    //xputs("Start\r\n"); // ????????>???


    while (1) {
        _LATC0 = 0;
        _LATC1 = 1;
        __delay_ms(500);
        xputs("Good"); // ????????>???
        _LATC0 = 1;
        _LATC1 = 0;
        __delay_ms(500);
        xputs("Baad"); // ????????>???
       // xfputs("Bad:%d\r\n",rtcSec); // ????????>???

    }
    return 0;
}


void DCIInit(void) {
    DCICON1 = 0;
    DCICON1bits.DCISIDL = 0; /* Continue operation in idle */
    DCICON1bits.DLOOP = 0; /* Loopback mode is disabled */
    DCICON1bits.CSCKD = 0; /* DCI is master - CSCK is output */
    DCICON1bits.CSCKE = 1; /* Data is sampled on raising edge */
    DCICON1bits.COFSD = 0; /* DCI is master - COFS is output */
    DCICON1bits.UNFM = 0; /* Transmit zeroes on TX underflow */
    DCICON1bits.CSDOM = 0; /* Transmit 0 on disabled time slots */
    DCICON1bits.DJST = 0; /* = Data transmission/reception begins one serial clock cycle after frame synchronization pulse */
    DCICON1bits.COFSM = 0b01; /* DCI mode is I2S Frame Sync mode */
    DCICON2 = 0;
    DCICON2bits.BLEN = 0; /* Interrupt on one buffer */
    DCICON2bits.COFSG = 0; /* Data frame has 1 word */
    DCICON2bits.WS = 0xF; /* Word size is 16 bits */

    TSCON = 0;
    TSCONbits.TSE0 = 1; // Transmit on Time Slot 0

    DCICON3 = DCI_BCG_VALUE;
    _DCIIE = 0; /* Disabled since DMA is used */

    //IPC15bits.DCIIP=6; /* Enable the interrupts */
    //IFS3bits.DCIIF=0;
    //IEC3bits.DCIIE=1;
    /* Force First a word to fill-in TX buffer/shift register */
    //DMA0REQbits.FORCE = 1;
    //while(DMA0REQbits.FORCE == 1);
    DCICON1bits.DCIEN = 1; // Enable DCI
    TXBUF0 = 0x00; //Dummy word to keep sync

}

void __attribute__((__interrupt__, no_auto_psv)) _DCIInterrupt(void) {
    IFS3bits.DCIIF = 0;

    if (rxBufferIndicator == 0) {
        TXBUF0 = 255; /* Write some data */
    } else if (rxBufferIndicator == 1) {
        TXBUF0 = 255; /* Write some data */
    } else if (rxBufferIndicator == 2) {
        TXBUF0 = -255; /* Write some data */
    } else {
        TXBUF0 = -255; /* Write some data */
        rxBufferIndicator = 0;
    }


    rxBufferIndicator++; /* Toggle the indicator */
}

void DMAInit(void) {
    /* DMA 0 - DPSRAM to DCI */
    DMA0CONbits.SIZE = 0; /* Word transfers */
    DMA0CONbits.DIR = 1; /* From DPSRAM to DCI */
    DMA0CONbits.AMODE = 0; /* Register Indirect with post-increment mode */
    DMA0CONbits.MODE = 0b10; /* Continuous ping pong mode enabled */
    DMA0CONbits.HALF = 0; /* Interrupt when all the data has been moved */
    DMA0CONbits.NULLW = 0;
    DMA0REQbits.FORCE = 0; /* Automatic transfer */
    DMA0REQbits.IRQSEL = 0b0111100; /* Codec transfer done */
    DMA0STA = __builtin_dmaoffset(txBufferA);
    DMA0STB = __builtin_dmaoffset(txBufferB);
    DMA0PAD = (volatile unsigned int) &TXBUF0;
    DMA0CNT = FRAME - 1;
    IFS0bits.DMA0IF = 0; // Clear DMA Interrupt Flag
    IEC0bits.DMA0IE = 1; // Enable DMA interrupt
    DMA0CONbits.CHEN = 1; // Enable DMA Channel
}

void __attribute__((__interrupt__, no_auto_psv)) _DMA0Interrupt(void) {
    _DMA0IF = 0;
    if (rxBufferIndicator == 0) {
        //processRxData(rxBufferA,txBufferA);
    } else {
        //processRxData(rxBufferB,txBufferB);
    }
    rxBufferIndicator ^= 1; /* Toggle the indicator */
}

/*---------------------------------------------------------*/
/* 1000Hz timer interrupt generated by Timer1              */
/*---------------------------------------------------------*/


void __attribute__((interrupt, no_auto_psv)) _T1Interrupt (void)
{
	static const BYTE samurai[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	static UINT div1k;
	BYTE n;


	_T1IF = 0;			/* Clear irq flag */
	Timer++;			/* Performance counter for this module */
	disk_timerproc();	/* Drive timer procedure of low level disk I/O module */

	/* Real Time Clock */
	if (++div1k >= 1000) {
		div1k = 0;
		if (++rtcSec >= 60) {
			rtcSec = 0;
			if (++rtcMin >= 60) {
				rtcMin = 0;
				if (++rtcHour >= 24) {
					rtcHour = 0;
					n = samurai[rtcMon - 1];
					if ((n == 28) && !(rtcYear & 3)) n++;
					if (++rtcMday > n) {
						rtcMday = 1;
						if (++rtcMon > 12) {
							rtcMon = 1;
							rtcYear++;
						}
					}
				}
			}
		}
	}
}