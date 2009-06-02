/*This file has been prepared for Doxygen automatic documentation generation.*/
//! \file *********************************************************************
//!
//! \brief This file contains the function declarations for usb host task functions
//!
//! - Compiler:           IAR EWAVR and GNU GCC for AVR
//! - Supported devices:  AT90USB1287, AT90USB1286, AT90USB647, AT90USB646
//!
//! \author               Atmel Corporation: http://www.atmel.com \n
//!                       Support and FAQ: http://support.atmel.no/
//!
//! ***************************************************************************

/* Copyright (c) 2007, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _USB_HOST_TASK_H_
#define _USB_HOST_TASK_H_

//_____ I N C L U D E S ____________________________________________________

#if (USB_HUB_SUPPORT==ENABLE)
#include "modules/usb/host_chap9/usb_host_hub.h"
#endif

//_____ T Y P E S  _________________________________________________________

typedef struct
{
   bit enable;
   U16 nb_byte_to_process;
   U16 nb_byte_processed;
   U16 nb_byte_on_going;
   U8 *ptr_buf;
   void(*handle)(U8 status, U16 nb_byte);
   U8 status;
   U8 timeout;
   U16 nak_timeout;
} S_pipe_int;



//_____ M A C R O S ________________________________________________________

#define PIPE_GOOD             0
#define PIPE_DATA_TOGGLE   0x01
#define PIPE_DATA_PID      0x02
#define PIPE_PID           0x04
#define PIPE_TIMEOUT       0x08
#define PIPE_CRC16         0x10
#define PIPE_STALL         0x20
#define PIPE_NAK_TIMEOUT   0x40
#define PIPE_DELAY_TIMEOUT 0x80

//! @defgroup usb_host_task USB host task
//! @{

   //! @brief Returns true when device connected and correctly enumerated.
   //! The host high level application should tests this before performing any applicative requests
   //! to the device connected
   #define Is_host_ready()        ((device_state==DEVICE_READY)   ? TRUE : FALSE)

   //! Returns true when the high application should not perform request to the device
   #define Is_host_not_ready()    ((device_state==DEVICE_READY)   ? FALSE :TRUE)

   //! Check if host controller is in suspend mode
   #define Is_host_suspended()    (((device_state==DEVICE_WAIT_RESUME) ||(device_state==DEVICE_SUSPENDED))  ? TRUE : FALSE)

   //! Check if host controller is not suspend mode
   #define Is_host_not_suspended()    (((device_state==DEVICE_WAIT_RESUME) ||(device_state==DEVICE_SUSPENDED))  ? FALSE : TRUE)

   //! Check if there is an attached device connected to the host
   #define Is_host_unattached()   ((device_state==DEVICE_UNATTACHED)   ? TRUE : FALSE)

   //! Check if there is an attached device connected to the host
   #define Is_host_attached()     ((device_state>=DEVICE_UNATTACHED)   ? TRUE : FALSE)

   //! This function should be called to make the host controller enter USB suspend mode
   #define Host_request_suspend()     (device_state=DEVICE_SUSPENDED)

   //! This function should be called to request the host controller to resume the USB bus
   #define Host_request_resume()      (request_resume=TRUE)

   //! Private ack for software event
   #define Host_ack_request_resume()  (request_resume=FALSE)

   //! Force reset and (re)enumeration of the connected device
   #define Host_force_enumeration()    (force_enumeration=TRUE, device_state=DEVICE_ATTACHED, init_usb_tree())

   //! Private check for resume sequence
   #define Is_host_request_resume()   ((request_resume==TRUE)   ? TRUE : FALSE)

   //! Returns true when a new device is enumerated
   #define Is_new_device_connection_event()   (new_device_connected ? TRUE : FALSE)

   //! Returns true when the device disconnects from the host
#if (USB_HUB_SUPPORT==ENABLE)
   #define Is_device_disconnection_event()   ((device_state==DEVICE_DISCONNECTED_ACK || device_state==DEVICE_DISCONNECTED || f_hub_port_disconnect) ? TRUE : FALSE)
#else
   #define Is_device_disconnection_event()   ((device_state==DEVICE_DISCONNECTED_ACK || device_state==DEVICE_DISCONNECTED) ? TRUE : FALSE)
#endif

   //! Stop all interrupt attached to a pipe
   #define Host_stop_pipe_interrupt(i) (\
         Host_disable_transmit_interrupt(), \
         Host_disable_receive_interrupt(), \
         Host_disable_stall_interrupt(), \
         Host_disable_error_interrupt(), \
         Host_disable_nak_interrupt(), \
         Host_reset_pipe(i))

   //! @defgroup device_state_value Host controller states
   //! Defines for device state coding
   //! \image html host_task.gif
   //! @{
   #define DEVICE_UNATTACHED        0
   #define DEVICE_ATTACHED          1
   #define DEVICE_POWERED           2
   #define DEVICE_DEFAULT           3
   #define DEVICE_ADDRESSED         4
   #define DEVICE_CONFIGURED        5
   #define DEVICE_READY             6

   #define DEVICE_ERROR             7

   #define DEVICE_SUSPENDED         8
   #define DEVICE_WAIT_RESUME       9

   #define DEVICE_DISCONNECTED      10
   #define DEVICE_DISCONNECTED_ACK  11

   #define Host_set_device_supported()   (device_status |=  0x01)
   #define Host_clear_device_supported() (device_status &= ~0x01)
   #define Is_host_device_supported()    (device_status &   0x01)

   #define Host_set_device_ready()       (device_status |=  0x02)
   #define Host_clear_device_ready()     (device_status &= ~0x02)
   #define Is_host_device_ready()        (device_status &   0x02)

   #define Host_set_configured()      (device_status |=  0x04)
   #define Host_clear_configured()    (device_status &= ~0x04)
   #define Is_host_configured()       (device_status &   0x04)

   #define Host_clear_device_status()    (device_status =   0x00)
   //! @}



//_____ D E C L A R A T I O N S ____________________________________________

/**
 * @brief This function initializes the USB controller in host mode,
 * the associated variables and interrupts enables.
 *
 * This function enables the USB controller for host mode operation.
 *
 * @param none
 *
 * @return none
 *
 */
void usb_host_task_init     (void);

/**
 * @brief Entry point of the host management
 *
 * The aim is to manage the device target connection and enumeration
 * depending on the device_state, the function performs the required operations
 * to get the device enumerated and configured
 * Once the device is operationnal, the device_state value is DEVICE_READY
 * This state should be tested by the host task application before performing
 * any applicative requests to the device.
 * This function is called from usb_task function depending on the usb operating mode
 * (device or host) currently selected.
 *
 * @param none
 *
 * @return none
 *
 */
void usb_host_task          (void);

/**
  * @brief This function send nb_data pointed with *buf with the pipe number specified
  *
  * @note This function will activate the host sof interrupt to detect timeout. The
  * interrupt enable sof will be restore.
  *
  * @param pipe
  * @param nb_data
  * @param buf
  *
  * @return status (0 is OK)
  */
U8 host_send_data(U8 pipe, U16 nb_data, U8 *buf);

/**
  * @brief This function receives nb_data pointed with *buf with the pipe number specified
  *
  * The nb_data parameter is passed as a U16 pointer, thus the data pointed by this pointer
  * is updated with the final number of data byte received.
  *
  * @note This function will activate the host sof interrupt to detect timeout. The
  * interrupt enable sof will be restore.
  *
  * @param pipe
  * @param nb_data
  * @param buf
  *
  * @return status (0 is OK)
  */
U8 host_get_data(U8 pipe, U16 *nb_data, U8 *buf);

U8 host_get_data_interrupt(U8 pipe, U16 nb_data, U8 *buf, void  (*handle)(U8 status, U16 nb_byte));

U8 host_send_data_interrupt(U8 pipe, U16 nb_data, U8 *buf, void  (*handle)(U8 status, U16 nb_byte));

void reset_it_pipe_str(void);

U8 is_any_interrupt_pipe_active(void);

extern U8 device_state;
extern U8 request_resume;
extern U8 new_device_connected;
extern U8 force_enumeration;


//! @}


#endif /* _USB_HOST_TASK_H_ */

