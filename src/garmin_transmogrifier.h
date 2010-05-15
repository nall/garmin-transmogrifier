#ifndef __GARMIN_TRANSMOGRIFIER_H__
#define __GARMIN_TRANSMOGRIFIER_H__

#include "LUFA/Drivers/USB/USB.h"
#include "LUFA/Scheduler/Scheduler.h"

TASK(USB_Garmin_Host);
void EVENT_USB_HostError(const uint8_t ErrorCode);
void EVENT_USB_DeviceAttached(void);
void EVENT_USB_DeviceUnattached(void);
void EVENT_USB_DeviceEnumerationFailed(const uint8_t ErrorCode, const uint8_t SubErrorCode);
void EVENT_USB_DeviceEnumerationComplete(void);


#endif // __GARMIN_TRANSMOGRIFIER_H__