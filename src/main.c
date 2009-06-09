#include "config.h"
#include "modules/common/scheduler/scheduler.h"
#include "lib_mcu/wdt/wdt_drv.h"
#include "lib_mcu/power/power_drv.h"
#include <util/delay.h>

#include <avr/interrupt.h>
#include "serial_disp.h"

int main(void)
{
    const uint16_t reset_status = MCUSR;

    uart_init(umAsync, 9600, csSize8, pNoParity, sbOneStopBit);
    if(Is_POR_reset() || Is_ext_reset())
    {
        lcd_clear();
    }
    else
    {
        printf("MCU:%d\n", reset_status);
        while(1){}
    }
    printf("MCU:%d\n", reset_status);

    MCUSR = 0;
    wdtdrv_disable();
    Clear_prescaler();
    scheduler();
    return 0;
}

#ifdef __GNUC__
    char __low_level_init(void) __attribute__ ((section (".init3"), naked));
#endif // __GNUC__

char __low_level_init(void)
{
    Clear_prescaler();
    return 1;
}
