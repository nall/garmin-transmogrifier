#ifndef __SERIAL_DISP_H__
#define __SERIAL_DISP_H__

#include <stdio.h>
#include <inttypes.h>

enum UARTMode
{
    umAsync = 0,
    umSync = 1,
    umReserved = 2,
    umMasterSPI = 3
};

enum Parity
{
    pNoParity = 0,
    pReserved = 1,
    pEvenParity = 2,
    pOddParity = 3
};

enum StopBits
{
    sbOneStopBit = 0,
    sbTwoStopBits = 1
};

enum CharSize
{
    csSize5 = 0,
    csSize6 = 1,
    csSize7 = 2,
    csSize8 = 3,
    csReserved0 = 4,
    csReserved1 = 5,
    csReserved2 = 6,
    csSize9 = 7
};

int uart_init(const enum UARTMode mode, const uint32_t baud, const enum CharSize bits,
    const enum Parity parity, const enum StopBits stopbits);
    
void lcd_clear(); 

#endif // __SERIAL_DISP_H__
