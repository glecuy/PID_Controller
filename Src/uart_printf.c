/*****************************************************************************
//  File Name    : uart_printf.c
//  Version      : 1.0
//
*****************************************************************************/
#include <avr/io.h>
#include <string.h>
#include <stdio.h>
#include <util/delay.h>


#define BAUD_RATE 19200

int uart_putch(char ch,FILE *stream);
int uart_getch(FILE *stream);

// Assign I/O stream to UART
FILE uart_stream = FDEV_SETUP_STREAM(uart_putch, uart_getch, _FDEV_SETUP_RW);



void uart_printf_init(void)
{
    // See ClockSetup()
    //unsigned long fosc = F_CPU/4;
    unsigned long fosc = F_CPU;
    //unsigned long fosc = 8400000U;


    // set baud rate
    unsigned long ubrr=(((fosc/BAUD_RATE)/16)-1);
    //unsigned long ubrr=26;
    UBRR0H = (ubrr>>8)&0xFF;
    UBRR0L = (ubrr>>0)&0xFF;

    UCSR0B = (1<<RXEN0)|(1<<TXEN0);         // enable Rx & Tx
    //UCSR0C=  (1<<UCSZ01)|(1<<UCSZ00);       // config USART; 8N1

    UCSR0C = 0
        | (0<<UMSEL01) | (0<<UMSEL00)   // Asynchronous USART
        | (0<<UPM01) | (0<<UPM00)       // Parity Disabled
        | (0<<USBS0)                    // 1 stop bit
        | (1<<UCSZ01) | (1<<UCSZ00)     // 8-bit character size
        //| (0<<UCPOL)                    // Rising TX, falling RX
        ;


    // Define Output/Input Stream
    stdout = stdin = &uart_stream;
}


int uart_putch(char ch,FILE *stream)
{
    if (ch == '\n')
        uart_putch('\r', stream);

    while (!(UCSR0A & (1<<UDRE0)))
        ;
    UDR0=ch;

    return 0;
}

int uart_getch(FILE *stream)
{
    unsigned char ch;

    while (!(UCSR0A & (1<<RXC0)))
        ;
    ch=UDR0;

    /* Echo the Output Back to terminal */
    uart_putch(ch,stream);

    return ch;
}




