#include "usb_host.h"
#include "common.h"
#include <util/delay.h>
#include <avr/io.h>

static void enable_pll()
{
    // Enable PLL and wait for it to lock
    PLLCSR = 0; // Clear PLOCK
    
    // Valid PLL values:
    // PLLP2    PLLP1   PLLP0   ExtXTAL
    //     0        1       1         8MHz
    //     1        0       1         16MHz  // AT90USB128x ONLY
    //     1        1       0         16MHz  // AT90USB64x * ATmega32U6 ONLY
    PLLCSR |= ~_BV(PLLP2) | _BV(PLLP1) | _BV(PLLP0) | _BV(PLLE);

    // Poll until PLL locks
    do
    {
        _delay_ms(100);
    } while((PLLCSR & _BV(PLOCK)) == 0);
}

static inline void disable_pll()
{
    PLLCSR = 0;
}

void usb_init()
{
    // Ensure USB is disabled before mucking with UWHCON
    USBCON &= ~_BV(USBE);
    
    // Select USB mode (host/device) via UIMOD
    UHWCON &= ~_BV(UIDE);
    
    // Implement workaround in spec 21.10.2
    UHWCON &= ~(_BV(UVCONE) | _BV(UVREGE));
    OTGCON &= ~_BV(VBUSHWC);
    PORTE |= _BV(7);
    OTGIEN |= _BV(SRPE);
    // End 21.10.2 workaround
    
    // Enable being notified on disconnect
    UHIEN |= _BV(DDISCE);
    
    UHWCON |= _BV(UIMOD) | _BV(UVREGE);    // host mode and enable pad regulator

    // Disable ID transtion and VBUS transition interrupts
    USBCON &= ~(_BV(IDTE) | _BV(VBUSTE));
    
    // Being a host means clearing the detach bit before setting HOST below
    // (spec 23.2)
    UDCON &= ~_BV(DETACH);
    
    // Enable USB, set us as a host and enable VBUS pad.
    USBCON |= _BV(USBE) | _BV(HOST) | _BV(OTGPADE);
}

void usb_reset_bus()
{
    UHCON |= _BV(RESET);
    
    // Wait for reset to complete
    while(IS_SET(UHCON, RESET))
    {
        _delay_ms(1);
    }
}

void usb_device_attached()
{
    enable_pll();
                  
    // Delay for USB spec's wait time for VBus to stabilize upon insertion
    // USB 2.0 Section 9.1.2
    _delay_ms(100);
    
    usb_reset_bus();
}

void usb_device_detached()
{
    disable_pll();
}

int usb_read_data(const enum PidName token, pipe_descriptor_t* pipe, void* data,
    const uint8_t size)
{
    UPNUM = pipe->id;
    if(IS_CLEAR(UPSTAX, CFGOK))
    {
        // This pipe isn't configured
        return EXIT_FAILURE;
    }

    // Override token to specified token
    UPCFG0X &= ~(_BV(PTOKEN1) | _BV(PTOKEN0));
    UPCFG0X |= (token << PTOKEN0);

    const uint8_t num_in_xfers = (size / PIPE_SIZE(pipe)) - 1; // Zero-Based
    if(num_in_xfers > 0)
    {
        // FIXME: be able to buffer longer requests
        return EXIT_FAILURE;
    }
    
    UPCONX &= ~_BV(INMODE);
    UPINRQX = num_in_xfers;
    
    if(IS_SET(UPINTX, FIFOCON))
    {
        // This is an error -- why isn't this bank free?
        return EXIT_FAILURE;
    }

    // Start the transfer
    UPCONX |= _BV(PFREEZE);
    
    // Sanity check that all the bytes we read made it into the FIFO count
    uint16_t bytecount = (UPBCHX << 8) | UPBCLX;
    while(bytecount < size)
    {
        // Wait for all data to arrive
        _delay_ms(1);
        bytecount = (UPBCHX << 8) | UPBCLX;
    }
    
    // Read the data from the fifo
    uint8_t* bytedata = (uint8_t*)data;
    for(uint16_t i = 0; i < size; ++i)
    {
        bytedata[i] = UPDATX;
    }
    
    // Free the current bank
    UPINTX &= ~_BV(FIFOCON);
    
    return EXIT_SUCCESS;
}

int usb_write_data(const enum PidName token, pipe_descriptor_t* pipe, void* data,
    const uint8_t size)
{
    UPNUM = pipe->id;
    if(IS_CLEAR(UPSTAX, CFGOK))
    {
        // This pipe isn't configured
        return EXIT_FAILURE;
    }
    else if(IS_SET(UPCONX, PFREEZE))
    {
        // This pipe is frozen for some reason
        return EXIT_FAILURE;
    }

    // Override token to specified token
    UPCFG0X &= ~(_BV(PTOKEN1) | _BV(PTOKEN0));
    UPCFG0X |= (token << PTOKEN0);
    
    while(IS_SET(UPINTX, FIFOCON))
    {
        // Wait for current operation to finish
        _delay_ms(1);
    }
    
    // Write the data into the fifo
    uint8_t* bytedata = (uint8_t*)data;
    for(uint16_t i = 0; i < size; ++i)
    {
        UPDATX = bytedata[i];
    }
    
    // Sanity check that all the bytes we read made it into the FIFO count
    uint16_t bytecount = (UPBCHX << 8) | UPBCLX;
    if(bytecount != size)
    {
        return EXIT_FAILURE;
    }
    
    // Send the bank
    UPINTX &= ~_BV(FIFOCON);
    
    return EXIT_SUCCESS;
}

int usb_configure_pipe(pipe_descriptor_t* pipe)
{
    UPNUM = pipe->id;
    
    UPCONX |= PEN;
    
    UPCFG0X = (pipe->type << PTYPE0) | (pipe->token << PTOKEN0) | (pipe->endpoint_target << PEPNUM0);
    
    UPCFG1X = (pipe->size << PSIZE0) | (pipe->num_banks << PBK0) << _BV(ALLOC);
    
    if(IS_SET(UPSTAX, CFGOK))
    {
        UPCFG2X = pipe->int_freq;
    }
    else
    {
        return EXIT_FAILURE;
    }
    
    // pipe now activated and frozen
    return EXIT_SUCCESS;
}

