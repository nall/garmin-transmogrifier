
#include "usb_defines.h"
#include "usb_host.h"
#include "serial_disp.h"
#include "common.h"

void show_error(const uint16_t code)
{
    
}

void device_connected_irq()
{
    usb_device_attached();

    // Setup Default Control Pipe
    pipe_descriptor_t ctrl_pipe;
    {
        ctrl_pipe.id = 0;
        ctrl_pipe.type = Control;
        ctrl_pipe.token = SETUP;
        ctrl_pipe.num_banks = OneBank;
        ctrl_pipe.size = Size64;
        ctrl_pipe.endpoint_target = 0;
        ctrl_pipe.int_freq = 0;

        usb_configure_pipe(&ctrl_pipe);        
    }
    
    // Get the device descriptor
    {
        usb_setup_data_t get_descriptor;
        get_descriptor.bmRequestType = 0x80;
        get_descriptor.bRequest = GET_DESCRIPTOR;
        get_descriptor.wValue = (DEVICE << 8) | 0;
        get_descriptor.wIndex = 0;
        get_descriptor.wLength = sizeof(usb_device_descriptor_t);
        
        // SETUP phase
        int rc = usb_write_data(SETUP, &ctrl_pipe, &get_descriptor, sizeof(usb_setup_data_t));
        if(rc == EXIT_FAILURE)
        {
            show_error(0);
        }
        
        // Data phase
        usb_device_descriptor_t dev_descr;
        rc = usb_read_data(IN, &ctrl_pipe, &dev_descr, sizeof(usb_device_descriptor_t));
        if(rc == EXIT_FAILURE)
        {
            show_error(0);
        }

        // STATUS phase (send ZLP)
        rc = usb_write_data(OUT, &ctrl_pipe, 0, 0);        
        if(rc == EXIT_FAILURE)
        {
            show_error(0);
        }
    }
}

int main()
{
    serial_init(umAsync, 9600, csSize8, pNoParity, sbOneStopBit);
    usb_init();
}
