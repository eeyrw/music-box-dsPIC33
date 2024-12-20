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
#include "diskio.h"


#define	LD_WORD(ptr)		(WORD)(((WORD)*((BYTE*)(ptr)+1)<<8)|(WORD)*(BYTE*)(ptr))
#define	LD_DWORD(ptr)		(DWORD)(((DWORD)*((BYTE*)(ptr)+3)<<24)|((DWORD)*((BYTE*)(ptr)+2)<<16)|((WORD)*((BYTE*)(ptr)+1)<<8)|*(BYTE*)(ptr))
#define	ST_WORD(ptr,val)	*(BYTE*)(ptr)=(BYTE)(val); *((BYTE*)(ptr)+1)=(BYTE)((WORD)(val)>>8)
#define	ST_DWORD(ptr,val)	*(BYTE*)(ptr)=(BYTE)(val); *((BYTE*)(ptr)+1)=(BYTE)((WORD)(val)>>8); *((BYTE*)(ptr)+2)=(BYTE)((DWORD)(val)>>16); *((BYTE*)(ptr)+3)=(BYTE)((DWORD)(val)>>24)


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


#define CODEC_SAMPLE_RATE 44100
#define DCI_BCG_VALUE (((FCY/64)/CODEC_SAMPLE_RATE)-1)
#define FRAME_SAMPLE_NUM 256
#define FRAME_BYTE_NUM FRAME_SAMPLE_NUM*2



short txBufferA[FRAME_SAMPLE_NUM] __attribute__((space(dma)));
short txBufferB[FRAME_SAMPLE_NUM] __attribute__((space(dma)));


volatile BYTE frameCount = 0;
volatile BYTE txBufIdx = 0;


void DCIInit(void);
void DMAInit(void);


extern int asmFunction(int a,int b);
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
    _RP15R = 0b01011; //SPI2 SCK on PB15
    _SDI2R = 23; //SPI2 SDI on PC7
    //_RP17R = 0b10011; //PWM OC2 on PC1
    //_RP16R = 0b10010; //PWM OC1 on PC0
    PPSLock;
    
    int q = asmFunction(12,3);

    AD1PCFGLbits.PCFG2 = 1;
    AD1PCFGLbits.PCFG3 = 1;
    AD1PCFGLbits.PCFG9 = 1;
    AD1PCFGLbits.PCFG10 = 1;
    AD1PCFGLbits.PCFG11 = 1;
    _TRISC1 = 0;
    _TRISC0 = 0;
    _TRISC7 = 1;
    _TRISB4 = 0;
    _TRISB0 = 0;
    _LATB4 = 1;


    T3CONbits.TON = 0; // Disable Timer
    T3CONbits.TCS = 0; // Select internal instruction cycle clock
    T3CONbits.TGATE = 0; // Disable Gated Timer mode
    T3CONbits.TCKPS = 0b00; // Select 1:1 Prescaler
    TMR3 = 0x00; // Clear timer register
    PR3 = 1024; // Load the period value
    IEC0bits.T3IE = 0; // Enable Timer1 interrupt
    T3CONbits.TON = 1; // Start Timer


    OC1CON = 0; /* It is a good practice to clear off the control bits initially */
    OC1CONbits.OCTSEL = 1; /* This selects the peripheral clock as the clock input to the OC
     module */
    //OC1R = 7000; /* This is just a typical number, user must calculate based on the
    // waveform requirements and the system clock */
    OC1RS = 70; /* Determines the Period */
    OC1CONbits.OCM = 6; /* This selects and starts the Edge Aligned PWM mode*/


    OC2CON = 0; /* It is a good practice to clear off the control bits initially */
    OC2CONbits.OCTSEL = 1; /* This selects the peripheral clock as the clock input to the OC
     module */
    //OC2R = 7000; /* This is just a typical number, user must calculate based on the
    // waveform requirements and the system clock */
    OC2RS = 100; /* Determines the Period */
    OC2CONbits.OCM = 6; /* This selects and starts the Edge Aligned PWM mode*/

    uart_init(115200);
    DMAInit();
    DCIInit();

    /* Start Timer1 in interval time of 1ms */
    PR1 = FCY / 8 / 1000;
    _TCKPS0 = 1; /* Select prescaler Fcy/8 */
    _TON = 1; /* Start Timer1 */
    _T1IE = 1; /* Enable Timer1 interrupt */

    xdev_in(uart_getc); /* Join UART and console */
    xdev_out(uart_putc);
    for (int i = 0; i < FRAME_SAMPLE_NUM / 8; i++) {
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

    disk_initialize(0);

    volatile FRESULT res;
    DIR dir;
    FILINFO fno;
    volatile DWORD sz;
    for (;;) {
        if (f_mount(&FatFs, "", 0) == FR_OK) { /* Initialize FS */
            Buff[0] = 0;
            res = f_opendir(&dir, "wav"); /* Open sound file directory */
            if (res != FR_OK)
                res = f_opendir(&dir, ""); /* Open root directory */

            while (res == FR_OK) { /* Repeat in the dir */
                res = f_readdir(&dir, 0); /* Rewind dir */
                while (res == FR_OK) { /* Play all wav files in the dir */
                    res = f_readdir(&dir, &fno); /* Get a dir entry */
                    if (res || !fno.fname[0]) break; /* Break on error or end of dir */
                    if (!(fno.fattrib & (AM_DIR | AM_HID)) && strstr(fno.fname, ".wav")) {
                        res = play("", fno.fname); /* Play file */
                    }
                }
            }
        }
        __delay_ms(500);
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
    TXBUF0 = 32767; //Dummy word to keep sync

}

void __attribute__((__interrupt__, no_auto_psv)) _DCIInterrupt(void) {
    //IFS3bits.DCIIF = 0;

    //    if (rxBufferIndicator == 0) {
    //        TXBUF0 = 255; /* Write some data */
    //    } else if (rxBufferIndicator == 1) {
    //        TXBUF0 = 255; /* Write some data */
    //    } else if (rxBufferIndicator == 2) {
    //        TXBUF0 = -255; /* Write some data */
    //    } else {
    //        TXBUF0 = -255; /* Write some data */
    //        rxBufferIndicator = 0;
    //    }
    //
    //
    //    rxBufferIndicator++; /* Toggle the indicator */
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
    DMA0CNT = FRAME_SAMPLE_NUM - 1;
    //IPC1bits.DMA0IP=1;
    IFS0bits.DMA0IF = 0; // Clear DMA Interrupt Flag
    IEC0bits.DMA0IE = 1; // Enable DMA interrupt
    DMA0CONbits.CHEN = 1; // Enable DMA Channel
}

void __attribute__((__interrupt__, auto_psv)) _DMA0Interrupt(void) {
    // _LATC0 = 1;
    frameCount++; /* Toggle the indicator */
    if (_PPST0 == 0) //txbufA finished
    {
        txBufIdx = 0;
    } else //txbufB finished
    {
        txBufIdx = 1;
    }
    _DMA0IF = 0;
    //_LATC0 = 0;
}

void __attribute__((__interrupt__, auto_psv)) _DMACError(void) {
    static unsigned int ErrorLocation;
    // Peripheral Write Collision Error Location
    if (DMACS0 & 0x0100) {
        ErrorLocation = DMA0STA;
    }
    // DMA RAM Write Collision Error Location
    if (DMACS0 & 0x0002) {
        ErrorLocation = DMA1STA;
    }
    DMACS0 = 0; //Clear Write Collision Flag
    INTCON1bits.DMACERR = 0; //Clear Trap Flag
}
/*---------------------------------------------------------*/
/* 1000Hz timer interrupt generated by Timer1              */

/*---------------------------------------------------------*/


void __attribute__((interrupt, auto_psv)) _T1Interrupt(void) {

    _T1IF = 0; /* Clear irq flag */
}