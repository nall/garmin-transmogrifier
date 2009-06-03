#include "config.h"
#include "modules/common/scheduler/scheduler.h"
#include "lib_mcu/wdt/wdt_drv.h"
#include "lib_mcu/power/power_drv.h"

int main(void)
{
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
