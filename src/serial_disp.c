#include "serial_disp.h"
#include "common.h"
#include <avr/io.h>
#include <string.h>

const uint8_t CMD_CHAR = 254;

int serial_init(const enum UARTMode mode, const uint32_t baud, const enum CharSize bits,
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

    return EXIT_SUCCESS;
}

static void serial_send_char(const char c)
{
    while(IS_CLEAR(UCSR1A, UDRE1))
    {
        // Wait for empty transmit buffer
    }
    
    UDR1 = c;    
}

int serial_display(char* string)
{
    for(uint8_t i = 0; i < strlen(string); ++i)
    {
        serial_send_char(string[i]);
    }
    
    return EXIT_SUCCESS;
}

int serial_clear()
{
    serial_send_char(CMD_CHAR);
    serial_send_char(0x1);
    serial_send_char(CMD_CHAR);
    serial_send_char(0xD);
    serial_send_char(CMD_CHAR);
    serial_send_char(0x80);
    
    return EXIT_SUCCESS;
}
