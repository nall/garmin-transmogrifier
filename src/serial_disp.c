#include "serial_disp.h"
#include "common.h"
#include <avr/io.h>
#include <string.h>

int serial_init(const enum UARTMode mode, const uint32_t baud, const enum CharSize bits,
            const enum Parity parity, const enum StopBits stopbits)
{
    // Use ASYNC UART
    UCSR1C = (mode << UMSEL10) |
             (parity << UPM10) |
             (stopbits << USBS1) |
             (bits << UCSZ10);
    
    const uint16_t scaled_baud = (F_CPU / (16 * baud)) - 1;
    UBRR1H = scaled_baud >> 8;
    UBRR1L = scaled_baud;


    // Enable transmit only
    UCSR1B = _BV(TXEN1);

    return EXIT_SUCCESS;
}

static void serial_send_char(const char c)
{
    while((UCSR1A & _BV(UDRE1)) == 0)
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
    serial_send_char(0x01);
}