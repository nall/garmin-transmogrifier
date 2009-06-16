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
#include <avr/io.h>
#include <avr/wdt.h>

#include "garmin.h"
#include "garmin_device.h"
#include "nmeagen.h"
#include "serial_disp.h"

#define Is_POR_Reset() ((MCUSR & (_BV(PORF))) ? TRUE : FALSE)
#define Is_EXT_Reset() ((MCUSR & (_BV(EXTRF))) ? TRUE : FALSE)
#define Is_WDT_Reset() ((MCUSR & (_BV(WDRF))) ? TRUE : FALSE)
#define Is_BOD_Reset() ((MCUSR & (_BV(BORF))) ? TRUE : FALSE)

static uint8_t cur_in_pipe = 0;

static uint8_t pipes[3];

static uint8_t pktbuf[GARMIN_MAX_PKTSIZE];
static Packet_t* gblpkt = (Packet_t*)pktbuf;

typedef uint8_t bool;
static const bool FALSE = 0;
static const bool TRUE = 1;

static uint16_t _garmin_recvpkt(const bool require_data);


static void show_error(char* const msg)
{
    lcd_clear();
    printf("ERROR: %s\n", msg);
    while(1) {}
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
    
    uint16_t nbytes = GARMIN_MAX_PKTSIZE;
    
    uint8_t status = 0; // FIXME host_get_data(pipes[cur_in_pipe], &nbytes, (U8*)gblpkt);

    // Were reading the bulk pipe and are now done?
    if(nbytes == 0 && cur_in_pipe == GRMN_DATA_IN_PIPE)
    {
        cur_in_pipe = GRMN_EVENTS_PIPE;
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
        status = 0; // FIXME host_get_data(pipes[cur_in_pipe], &nbytes, (U8*)(&(gblpkt->mData)));        
        totalbytes += nbytes;

        if(nbytes == 0 && cur_in_pipe == GRMN_DATA_IN_PIPE)
        {
            cur_in_pipe = GRMN_EVENTS_PIPE;
        }
    }
    
    return nbytes;
}

void garmin_sendpkt()
{
    // FIXME const uint16_t nbytes = GARMIN_HEADER_SIZE + gblpkt->mDataSize;
    const uint8_t status = 0; // FIXME host_send_data(pipes[GRMN_DATA_OUT_PIPE], nbytes, (U8*)gblpkt);
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
    
#if (DEBUG==1)
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
        #if (DEBUG==1)
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

void garmin_transmogrifier_task(void)
{
    
}

int main(void)
{
    const uint16_t reset_status = MCUSR;

    uart_init(umAsync, 9600, csSize8, pNoParity, sbOneStopBit);
    if(Is_POR_Reset() || Is_EXT_Reset())
    {
        lcd_clear();
    }
#if (DEBUG == 1)
    printf("MCU:%d\n", reset_status);
#endif // DEBUG

    MCUSR = 0;
    wdt_disable();
    while(TRUE)
    {
        
    }
    return 0;
}

