/*
 * This code borrows heavily from the garminusb2nmea source code and as such
 * retains the following copyright, conditions, and disclaimer:
 * 
    Garmin USB NMEA converter
    Copyright (C) 2004 Manuel Kasper <mk@neon1.net>.
    All rights reserved.
        
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    
    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.
    
    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
    
    THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES,
    INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
    AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
    OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdio.h>

#include "config.h"
#include "conf_usb.h"
#include "lib_mcu/usb/usb_drv.h"
#include "modules/usb/host_chap9/usb_host_task.h"
#include "modules/usb/host_chap9/usb_host_enum.h"

#include "garmin.h"
#include "serial_disp.h"

#define DEBUG 1

const uint8_t GT_PIPE_IN_INT = 0;
const uint8_t GT_PIPE_OUT_BULK = 1;
const uint8_t GT_PIPE_IN_BULK = 2;

uint8_t pipes[3];

uint8_t pktbuf[GARMIN_MAX_PKTSIZE];
Packet_t* glbpkt = (Packet_t*)pktbuf;
uint16_t pktoffset = 0;

void show_error(char* const msg)
{
    serial_clear();
    serial_display("ERROR: ");
    serial_display(msg);
}

static void sanity_check_ep_type(const uint8_t device, const uint8_t interface,
    const uint8_t ep, const uint8_t ep_addr)
{
#ifdef DEBUG
    // Sanity check EP types
    {
        const uint8_t ep_type = usb_tree.device[device].interface[interface].ep[ep].ep_type;
        const uint8_t is_in = Is_ep_addr_in(ep_addr);
        switch(ep)
        {
            case 0:
            {
                if(ep_type != TYPE_INTERRUPT || !is_in)
                {
                    show_error("Endpoint 0 was not of type In/Interrupt");
                }
                break;                            
            }
            case 1:
            {
                if(ep_type != TYPE_BULK || is_in)
                {
                    show_error("Endpoint 1 was not of type Out/Bulk");
                }
                break;                            
            }
            case 2:
            {
                if(ep_type != TYPE_BULK)
                {
                    show_error("Endpoint 2 was not of type In/Bulk");
                }
                break;                            
            }
            default:
            {
                show_error("Unexpected endpoint > 2");
                break;                            
            }
        }
    }
#endif // DEBUG                    
}

static void intialize_pipes()
{
    const uint8_t device = 0;
    const uint8_t interface = 0;

    if(Get_nb_device() > 1)
    {
        show_error("Found more than 1 Device");
    }
    else if(Get_nb_supported_interface() > 1)
    {
        show_error("Found more than 1 Interface");
    }
    else if(Get_nb_ep(interface) != 3)
    {
        show_error("Found number of endpoints != 3 on Interface 0");
    }

    Host_select_device(device);

#ifdef DEBUG
    {
        const uint16_t vid = Get_VID();
        const uint16_t pid = Get_PID();
        char buf[128];
        sprintf(buf, "VID: 0x%04x PID: 0x%04x", vid, pid);
        serial_display(buf);
    }
#endif // DEBUG

    // Connect up the endpoints. We should have three:
    // 1: IN, Interrupt (64 bytes)
    // 2: OUT, Bulk (64 bytes)
    // 3: IN, Bulk (8 bytes)

    uint8_t addrs[3];
    for(uint8_t ep = 0; ep < Get_nb_ep(interface); ++ep)
    {
        addrs[ep] = Get_ep_addr(interface, ep);
        pipes[ep] = host_get_hwd_pipe_nb(addrs[ep]);

        if(ep == 0)
        {
            Host_select_pipe(pipes[ep]);
            Host_continuous_in_mode();
            Host_unfreeze_pipe();
        }

        sanity_check_ep_type(device, interface, ep, addrs[ep]);
    }
}

void garmin_recvpkt(const uint8_t pipe_id)
{
    U16 nbytes = GARMIN_HEADER_SIZE;
    uint8_t status = host_get_data(pipes[pipe_id], &nbytes, (U8*)glbpkt);
    if(nbytes != GARMIN_HEADER_SIZE || status != 0)
    {
        show_error("host_get_data couldn't return GARMIN_HEADER_SIZE bytes");
    }
    
    if(glbpkt->mDataSize > 0)
    {
        nbytes = glbpkt->mDataSize;
        status = host_get_data(pipes[pipe_id], &nbytes, (U8*)(&(glbpkt->mData)));        
        if(nbytes != glbpkt->mDataSize || status != 0)
        {
            show_error("host_get_data couldn't return mDataSize bytes");
        }
    }    
}

void garmin_sendpkt(Packet_t* pkt)
{
    const uint16_t nbytes = GARMIN_HEADER_SIZE + pkt->mDataSize;
    const uint8_t status = host_send_data(pipes[GT_PIPE_OUT_BULK], nbytes, (U8*)glbpkt);
    if(status != 0)
    {
        show_error("Error sending packet");
    }
}

void garmin_recvpkt_bulk(Packet_t* pkt)
{
    
}

void garmin_recvpkt_int(const uint8_t status, const uint16_t nbytes)
{
    if(status != PIPE_GOOD)
    {
        show_error("Bad status in garmin_recvpkt_int");
    }
    if(nbytes >= GARMIN_HEADER_SIZE)
    {
        // Check if we have a well-formed packet
    }
}

void garmin_transmogrifier_task_init(void)
{
    serial_init(umAsync, 9600, csSize8, pNoParity, sbOneStopBit);
}

void garmin_transmogrifier_task(void)
{
    // Is_host_ready will be true if the host is intialized AND a device has
    // been attached and enumerated.
    if(Is_host_ready())
    {
        // This will be true exactly once after the device has connected
        if(Is_new_device_connection_event())
        {
            // setup the pipes array
            intialize_pipes();
        }
    }
    
    if(Is_device_disconnection_event())
    {
        // FIXME: What action do we take? Any?
    }
}
