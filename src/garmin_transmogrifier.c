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
#include <string.h>

#include "config.h"
#include "conf_usb.h"
#include "lib_mcu/usb/usb_drv.h"
#include "modules/usb/host_chap9/usb_host_task.h"
#include "modules/usb/host_chap9/usb_host_enum.h"

#include "garmin.h"
#include "nmeagen.h"
#include "serial_disp.h"

#define DEBUG 1

static const uint8_t GT_PIPE_IN_INT = 0;
static const uint8_t GT_PIPE_OUT_BULK = 1;
static const uint8_t GT_PIPE_IN_BULK = 2;
static uint8_t cur_in_pipe = 0;

static uint8_t pipes[3];

static uint8_t pktbuf[GARMIN_MAX_PKTSIZE];
static Packet_t* gblpkt = (Packet_t*)pktbuf;

static uint16_t _garmin_recvpkt(const bool require_data);

static void show_error(char* const msg)
{
    lcd_clear();
    printf("ERROR: %s\n", msg);
    while(1) {}
}

static void sanity_check_ep_type(const uint8_t device, const uint8_t interface,
    const uint8_t ep, const uint8_t ep_addr)
{
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
}

int initialize_pipes()
{
    const uint8_t device = 0;
    const uint8_t interface = 0;
    cur_in_pipe = GT_PIPE_IN_INT;

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
    printf("VID: 0x%04x PID: 0x%04x\n", Get_VID(), Get_PID());
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

    return 1;
}

static void init_packet(const uint8_t type, const uint8_t id)
{
    memset(pktbuf, 0, GARMIN_MAX_PKTSIZE);
    gblpkt->mPacketType = type;
    gblpkt->mPacketId = id; 
}

uint16_t garmin_recvpkt()
{
    return _garmin_recvpkt(TRUE);
}

static uint16_t _garmin_recvpkt(const bool require_data)
{
    // Zero-out the packet structure
    init_packet(0, 0);
    
    U16 nbytes = GARMIN_MAX_PKTSIZE;
    uint8_t status = host_get_data(pipes[cur_in_pipe], &nbytes, (U8*)gblpkt);

    // Were reading the bulk pipe and are now done?
    if(nbytes == 0 && cur_in_pipe == GT_PIPE_IN_BULK)
    {
        cur_in_pipe = GT_PIPE_IN_INT;
    }

    if(nbytes == 0 && require_data == FALSE)
    {
        return 0;
    }

    if(nbytes < GARMIN_HEADER_SIZE || status != 0)
    {
        show_error("host_get_data couldn't return GARMIN_HEADER_SIZE bytes");            
    }
    
    uint8_t totalbytes = nbytes;
    while(gblpkt->mDataSize > 0 && totalbytes < (GARMIN_HEADER_SIZE + gblpkt->mDataSize))
    {
        nbytes = GARMIN_MAX_PKTSIZE;
        status = host_get_data(pipes[cur_in_pipe], &nbytes, (U8*)(&(gblpkt->mData)));        
        totalbytes += nbytes;

        if(nbytes == 0 && cur_in_pipe == GT_PIPE_IN_BULK)
        {
            cur_in_pipe = GT_PIPE_IN_INT;
        }
    }
    
    return nbytes;
}

void garmin_sendpkt()
{
    const uint16_t nbytes = GARMIN_HEADER_SIZE + gblpkt->mDataSize;
    const uint8_t status = host_send_data(pipes[GT_PIPE_OUT_BULK], nbytes, (U8*)gblpkt);
    if(status != 0)
    {
        show_error("Error sending packet");
    }
}

void garmin_start_session()
{
    init_packet(Prot_USB, Pid_Start_Session);
    garmin_sendpkt();

    do
    {
        // Ignore all packets until we get this, per the Garmin Spec 3.2.3.3
        garmin_recvpkt();        
    } while(gblpkt->mPacketId != Pid_Session_Started);
    
#ifdef DEBUG
    
    // Protocol is little endian and gcc-avr is too, so we can just cast this.
    uint32_t unitID = *((uint32_t*)(&(gblpkt->mData)));
    printf("UnitID: 0x%lx\n", unitID);
#endif // DEBUG
}

bool garmin_check_protocol_support(const uint8_t tag, const uint16_t value)
{
    init_packet(Prot_Application, Pid_Product_Rqst);
    garmin_sendpkt();
    
    // Product_Data_Type
    {
        garmin_recvpkt();
        #ifdef DEBUG
        if(gblpkt->mPacketId != Pid_Product_Data)
        {
            show_error("Non-Product Data Type");
        }
        #endif // DEBUG        
    }

    // Check for zero or more Ext Product packets
    {
        do
        {
            garmin_recvpkt();
        } while(gblpkt->mPacketId == Pid_Ext_Product_Data);        
    }

    // Get Protocol Array
    if(gblpkt->mPacketId == Pid_Protocol_Array)
    {
        const uint16_t num_types = gblpkt->mDataSize / sizeof(Protocol_Data_Type);
        Protocol_Data_Type* cur_type = (Protocol_Data_Type*)(&(gblpkt->mData[0]));
        for(uint16_t i = 0; i < num_types; ++i)
        {
            // Protocol is little endian and gcc-avr is too, so we just compare
            if(cur_type->mTag == tag && cur_type->mData == value)
            {
                // We found a match!
                return TRUE;
            }

            ++cur_type;
        }
    }    
    else
    {
        show_error("Unexpected A000-A001 state");
    }
    
    return FALSE;
}

void garmin_interrupt_recv(const U8 status, const U16 nbytes)
{
    printf("grmn_intr\n");
    if(status == PIPE_GOOD)
    {
        if(nbytes == (GARMIN_HEADER_SIZE + sizeof(D800_Pvt_Data_Type)))
        {
            // Got a good packet
            D800_Pvt_Data_Type* pvt = (D800_Pvt_Data_Type*)(&(gblpkt->mData));
            char nmeabuf[512];
            nmea_gprmc(pvt, nmeabuf);
            printf("BUF: %s\n", nmeabuf);
            
            // Get the next one
            init_packet(0, 0);
            host_get_data_interrupt(pipes[cur_in_pipe],
                GARMIN_HEADER_SIZE + sizeof(D800_Pvt_Data_Type),
                pktbuf, garmin_interrupt_recv);
        }
        else
        {
            printf("rcv %d\n", nbytes);
            while(1){}
            show_error("Bad data size in int_recv\n");
        }
    }
    else
    {
        show_error("status != PIPE_GOOD");
    }
}

void garmin_transmogrifier_task_init(void)
{
}

extern U8 device_state;
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
            initialize_pipes();

            {
                uint32_t private_mode[4];
                private_mode[0] = 0x01106E4B;
                private_mode[1] = 2;
                private_mode[2] = 4;
                private_mode[3] = 0;
                const uint8_t status = host_send_data(pipes[GT_PIPE_OUT_BULK], 16, (U8*)private_mode);
                if(status != 0)
                {
                    show_error("Can't send private data");
                }
            }

            garmin_start_session();
            bool supported = garmin_check_protocol_support(Tag_Appl_Prot_Id, 800);
            supported &= garmin_check_protocol_support(Tag_Data_Type_Id, 800);
            supported &= garmin_check_protocol_support(Tag_Link_Prot_Id, 1);
            if(supported == FALSE)
            {
                show_error("L001/A800/D800 is not supported");
            }
            else
            {
                printf("Enable PVT\n");
                // Initiate PVT transfers from the device
                init_packet(Prot_Application, Pid_Command_Data);
                gblpkt->mDataSize = 2;
                gblpkt->mData[0] = Cmnd_Start_Pvt_Data;
                gblpkt->mData[1] = 0;
                garmin_sendpkt();
            }
        }

        // Constantly check interrupt pipe
        {
            uint16_t bytes = _garmin_recvpkt(FALSE);
            if(bytes > 0)
            {
                switch(gblpkt->mPacketId)
                {
                    case Pid_Pvt_Data:
                    {
                        // Got a good packet
                        D800_Pvt_Data_Type* pvt = (D800_Pvt_Data_Type*)(&(gblpkt->mData));
                        char nmeabuf[512];
                        nmea_gprmc(pvt, nmeabuf);
                        printf("BUF: %s\n", nmeabuf);
                        break;
                    }
                    case Pid_Data_Available:
                    {
                        cur_in_pipe = GT_PIPE_IN_BULK;
                        break;
                    }
                    default:
                    {
#ifdef DEBUG
                        printf("recvPkt(%d)\n", gblpkt->mPacketId);
#endif // DEBUG
                    }
                }
            }
        }
    }
    
    if(Is_device_disconnection_event())
    {
        // FIXME: What action do we take? Any?
    }
}

void garmin_host_usb_error()
{
    show_error("GRMN-USB-ERR\n");
}
