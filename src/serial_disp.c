#include "serial_disp.h"
#include "common.h"
#include <avr/io.h>
#include <string.h>

#ifndef DISP_WIDTH
#define DISP_WIDTH 20
#endif // DISP_WIDTH

#ifndef DISP_HEIGHT
#define DISP_HEIGHT 4
#endif // DISP_HEIGHT

static uint8_t cur_line_pos = 0;
static const uint8_t CMD_CHAR1 = 0x7C;
static const uint8_t CMD_CHAR2 = 0xFE;
static void _send_char(const char c);

static int uart_putc(char c, FILE* stream);
static FILE uart_io = FDEV_SETUP_STREAM(uart_putc, NULL, _FDEV_SETUP_WRITE);

static void _send_char(const char c)
{
    while(IS_CLEAR(UCSR1A, UDRE1))
    {
        // Wait for empty transmit buffer
    }
    
    UDR1 = c;    
}

int uart_init(const enum UARTMode mode, const uint32_t baud, const enum CharSize bits,
            const enum Parity parity, const enum StopBits stopbits)
{
    // Use ASYNC UART
    UCSR1C = (mode << UMSEL10) |
             (parity << UPM10) |
             (stopbits << USBS1) |
             (bits << UCSZ10);
    
    const uint32_t scaled_baud = (F_CPU / (16L * baud)) - 1L;
    UBRR1H = (scaled_baud >> 8) & 0xFF;
    UBRR1L = scaled_baud & 0xFF;

    // Enable transmit only
    UCSR1B = _BV(TXEN1);

    // Don't use 2x
    UCSR1A &= ~_BV(U2X1);
    
    // set the standard output streams to send to the UART
    stdout = stderr = &uart_io;

    return EXIT_SUCCESS;
}

static int uart_putc(char c, FILE* stream)
{
    if((uint8_t)c == CMD_CHAR1 || (uint8_t)c == CMD_CHAR2)
    {
        c = 0x0;
    }
    else if(c == '\n')
    {
        _send_char('\r');
//        c = '.';
    }
    
    _send_char(c);
    
    return 0;
}

void lcd_send_cmd1(const char c)
{
    _send_char(CMD_CHAR1);
    _send_char(c);
}

void lcd_send_cmd2(const char c)
{
    _send_char(CMD_CHAR2);
    _send_char(c);
}

void lcd_clear()
{
    cur_line_pos = 0;

    lcd_send_cmd2(0x1);  // Clear display
    lcd_send_cmd2(0xD);  // Blinking cursor on
    lcd_send_cmd2(0x80); // Set cursor to position 0
}

