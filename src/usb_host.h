#ifndef __USB_HOST_H__
#define __USB_HOST_H__

#include "usb_defines.h"
void usb_init();
void usb_reset_bus();
void usb_device_attached();
void usb_device_detached();
int usb_configure_pipe(pipe_descriptor_t* pipe);

int usb_write_data(const enum PidName token, pipe_descriptor_t* pipe, void* data,
    const uint8_t size);
int usb_read_data(const enum PidName token, pipe_descriptor_t* pipe, void** data,
    const uint8_t size);

#endif // __USB_HOST_H__
