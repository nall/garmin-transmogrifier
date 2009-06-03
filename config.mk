
# Project name
PROJECT = garmin-transmogrifier

# CPU architecture : {avr0|...|avr6}
# Parts : {at90usb646|at90usb647|at90usb1286|at90usb1287|at90usb162|at90usb82}
MCU = at90usb647

# Source files
CSRCS = \
  src/main.c\
  src/nmeagen.c\
  src/garmin_transmogrifier.c\
  src/serial_disp.c\
  src/atmel-usb-lib/lib_mcu/usb/usb_drv.c\
  src/atmel-usb-lib/lib_mcu/power/power_drv.c\
  src/atmel-usb-lib/lib_mcu/wdt/wdt_drv.c\
  src/atmel-usb-lib/modules/common/scheduler/scheduler.c\
  src/atmel-usb-lib/modules/usb/host_chap9/usb_host_enum.c\
  src/atmel-usb-lib/modules/usb/host_chap9/usb_host_task.c\
  src/atmel-usb-lib/modules/usb/usb_task.c

# Assembler source files
ASSRCS = \

