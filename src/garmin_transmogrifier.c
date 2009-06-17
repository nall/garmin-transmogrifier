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

#include "config_descriptor.h"
#include "garmin_transmogrifier.h"
#include "garmin.h"
#include "garmin_device.h"
#include "nmeagen.h"
#include "serial_disp.h"

#define Is_POR_Reset() ((MCUSR & (_BV(PORF))) ? TRUE : FALSE)
#define Is_EXT_Reset() ((MCUSR & (_BV(EXTRF))) ? TRUE : FALSE)
#define Is_WDT_Reset() ((MCUSR & (_BV(WDRF))) ? TRUE : FALSE)
#define Is_BOD_Reset() ((MCUSR & (_BV(BORF))) ? TRUE : FALSE)

static uint8_t cur_in_pipe = 0;

static uint8_t pktbuf[GARMIN_MAX_PKTSIZE];
static Packet_t* gblpkt = (Packet_t*)pktbuf;

static const bool FALSE = 0;
static const bool TRUE = 1;

static uint16_t _garmin_recvpkt(const bool require_data);

TASK_LIST
{
        { .Task = USB_USBTask    , .TaskStatus = TASK_STOP },
        { .Task = USB_Garmin_Host, .TaskStatus = TASK_STOP },
};

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

static uint16_t garmin_recvpkt()
{
    return _garmin_recvpkt(TRUE);
}

static uint16_t _garmin_recvpkt(const bool require_data)
{
    // Zero-out the packet structure
    init_packet(0, 0);
    
    Pipe_SelectPipe(cur_in_pipe);
    Pipe_Unfreeze();

    uint16_t totalbytes = 0;

    if(Pipe_BytesInPipe() > 0 && require_data == FALSE)
    {
        return totalbytes;
    }

    enum Pipe_Stream_RW_ErrorCodes_t rc = Pipe_Read_Stream_LE(gblpkt, GARMIN_HEADER_SIZE);
    if(rc == PIPE_RWSTREAM_NoError)
    {
        totalbytes += GARMIN_HEADER_SIZE;
        rc = Pipe_Read_Stream_LE(gblpkt->mData, gblpkt->mDataSize);
        if(rc != PIPE_RWSTREAM_NoError)
        {
            printf("StreamRWErr1(%d)", rc); while(1) {}
        }
        else
        {
            totalbytes += gblpkt->mDataSize;
        }
    }
    else
    {
        printf("StreamRWErr0(%d)", rc); while(1) {}
    }
    
    Pipe_ClearIN();
    Pipe_Freeze();

    return totalbytes;
}

void garmin_sendpkt()
{
    Pipe_SelectPipe(GRMN_DATA_OUT_PIPE);
    Pipe_Unfreeze();
    
    const uint16_t nbytes = GARMIN_HEADER_SIZE + gblpkt->mDataSize;
    
    Pipe_Write_Stream_LE(gblpkt, nbytes);
    
    Pipe_ClearOUT();
    Pipe_Freeze();
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


/** Event handler for the USB_DeviceAttached event. This indicates that a device
 *  has been attached to the host, and starts the library USB task to begin the
 *  enumeration and USB management process.
 */
void EVENT_USB_DeviceAttached(void)
{
    printf("Device Attached.");
    /* Start USB management task to enumerate the device */
    Scheduler_SetTaskMode(USB_USBTask, TASK_RUN);    
}

/** Event handler for the USB_DeviceUnattached event. This indicates that a
 *  device has been removed from the host, and stops the library USB task
 *  management process.
 */
void EVENT_USB_DeviceUnattached(void)
{
    /* Stop USB management and Still Image tasks */
    Scheduler_SetTaskMode(USB_USBTask, TASK_STOP);
    Scheduler_SetTaskMode(USB_Garmin_Host, TASK_STOP);

    printf("Device Unattached.");
}

/** Event handler for the USB_DeviceEnumerationComplete event. This indicates
 *  that a device has been successfully enumerated by the host and is now ready
 *  to be used by the application.
 */
void EVENT_USB_DeviceEnumerationComplete(void)
{
    /* Once device is fully enumerated, start the Garmin Host task */
    Scheduler_SetTaskMode(USB_Garmin_Host, TASK_RUN);
}

/** Event handler for the USB_HostError event. This indicates that a hardware
 *  error occurred while in host mode.
 */
void EVENT_USB_HostError(const uint8_t ErrorCode)
{
    USB_ShutDown();

#if (DEBUG == 1)
    printf("Host Mode Error(%d)", ErrorCode);
#endif // DEBUG
    for(;;);   
}


/** Event handler for the USB_DeviceEnumerationFailed event. This indicates that
 *  a problem occurred while enumerating an attached USB device.
 */
void EVENT_USB_DeviceEnumerationFailed(const uint8_t ErrorCode, const uint8_t SubErrorCode)
{
#if (DEBUG == 1)
    printf("DevEnumFail(%d, %d, %d)", ErrorCode, SubErrorCode, USB_HostState); while(1) {}
#endif // DEBUG
}

/** Task to set the configuration of the attached device after it has been
 *  enumerated.
 */
void USB_Garmin_Host(void)
{
    uint8_t lastState = 0;
    uint8_t ErrorCode = 0;
    switch(USB_HostState)
    {
        case HOST_STATE_Addressed:
        {
            /* Standard request to set the device configuration to configuration 1 */
            USB_ControlRequest = (USB_Request_Header_t)
                    {
                            .bmRequestType = (REQDIR_HOSTTODEVICE | REQTYPE_STANDARD | REQREC_DEVICE),
                            .bRequest      = REQ_SetConfiguration,
                            .wValue        = 1,
                            .wIndex        = 0,
                            .wLength       = 0,
                    };

            /* Select the control pipe for the request transfer */
            Pipe_SelectPipe(PIPE_CONTROLPIPE);

            /* Send the request, display error and wait for device detach if request fails */
            if (USB_Host_SendControlRequest(NULL) != HOST_SENDCONTROL_Successful)
            {
#if (DEBUG == 1)
                    printf("Control error.");
#endif // DEBUG

                    /* Wait until USB device disconnected */
                    while (USB_IsConnected);
                    break;
            }
                    
            USB_HostState = HOST_STATE_Configured;
            break;
        }
        case HOST_STATE_Configured:
        {
#if (DEBUG == 1)
            printf("Getting Config Data.\r\n");
#endif // DEBUG
    
            /* Get and process the configuration descriptor data */
            if ((ErrorCode = ProcessConfigurationDescriptor()) != SuccessfulConfigRead)
            {
#if (DEBUG == 1)
                printf("ConfigErr(%d)", ErrorCode);
#endif // DEBUG
                    /* Wait until USB device disconnected */
                    while (USB_IsConnected);
                    break;
            }

#if (DEBUG == 1)
            printf("Garmin GPS Enumerated.\r\n");
#endif // DEBUG

            USB_HostState = HOST_STATE_Ready;
            break;
        }
        case HOST_STATE_Ready:
        {
            if(lastState != HOST_STATE_Ready)
            {
                // First time we've been ready -- start application protocol
                garmin_start_session();
                bool supported = garmin_check_protocol_support(Tag_Appl_Prot_Id, 800);
                supported &= garmin_check_protocol_support(Tag_Data_Type_Id, 800);
                supported &= garmin_check_protocol_support(Tag_Link_Prot_Id, 1);
                if(supported == FALSE)
                {
                    show_error("L001/A800/D800 is not supported");
                }
                
                init_packet(Prot_Application, Pid_Command_Data);
                gblpkt->mDataSize = 2;
                gblpkt->mData[0] = Cmnd_Start_Pvt_Data;
                gblpkt->mData[1] = 0;
                garmin_sendpkt();
            }
            else
            {
                // Check Interrupt Pipe and keep on keeping on.
                
                uint16_t bytes = _garmin_recvpkt(FALSE);
                if(bytes > 0)
                {
                    switch(gblpkt->mPacketId)
                    {
                        case Pid_Pvt_Data:
                        {
                            D800_Pvt_Data_Type* pvt = (D800_Pvt_Data_Type*)(&(gblpkt->mData));
                            char nmeabuf[512];
                            nmea_gprmc(pvt, nmeabuf);
                            printf("BUF: %s", nmeabuf);
                            break;
                        }
                        case Pid_Data_Available:
                        {
                            cur_in_pipe = GRMN_DATA_IN_PIPE;
                            break;
                        }
                        default:
                        {
#if (DEBUG == 1)
                            printf("recvPkt(%d)", gblpkt->mPacketId);
#endif // DEBUG
                        }
                    }
                }
            }
        }
    }
    
    lastState = USB_HostState;
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
    printf("MCU2:%d\n", reset_status);
#endif // DEBUG

    MCUSR = 0;
    wdt_disable();
    
    // Disable clock division
    {
        CLKPR = _BV(CLKPCE);
        CLKPR = 0;        
    }
    
    Scheduler_Init();
    
    USB_Init();
    
    Scheduler_Start();
    
    return 0; // never reached
}

