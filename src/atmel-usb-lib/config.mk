
# Project name
PROJECT = ../../garmin-transmogrifier

# CPU architecture : {avr0|...|avr6}
# Parts : {at90usb646|at90usb647|at90usb1286|at90usb1287|at90usb162|at90usb82}
MCU = at90usb647

# Source files
CSRCS = \
  ../main.c\
  ../host_template_task.c\
  lib_mcu/usb/usb_drv.c\
  lib_mcu/power/power_drv.c\
  modules/common/scheduler/scheduler.c\
  modules/usb/host_chap9/usb_host_enum.c\
  modules/usb/host_chap9/usb_host_task.c\
  modules/usb/usb_task.c

# Assembler source files
ASSRCS = \

