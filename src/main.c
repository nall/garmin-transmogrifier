#include <avr/interrupt.h>
#include <stdio.h>
#include "serial_disp.h"
#include "common.h"

#define DEBUG 1

void device_connected_irq();
void device_disconnected_irq();

void show_error(char* const msg)
{
    serial_clear();
    serial_display("ERROR: ");
    serial_display(msg);
}

int main()
{
    serial_init(umAsync, 9600, csSize8, pNoParity, sbOneStopBit);
    return 0;
}
