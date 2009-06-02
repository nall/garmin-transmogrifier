/*This file has been prepared for Doxygen automatic documentation generation.*/
//! \file *********************************************************************
//!
//! \brief This file manages the high level USB Host application.
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

//_____  I N C L U D E S ___________________________________________________

#include "config.h"
#include "conf_usb.h"
#include "host_template_task.h"
#include "modules/usb/host_chap9/usb_host_task.h"
#include "modules/usb/host_chap9/usb_host_enum.h"
#include "lib_mcu/usb/usb_drv.h"


//_____ M A C R O S ________________________________________________________

//_____ D E F I N I T I O N S ______________________________________________

//_____ D E C L A R A T I O N S ____________________________________________

   U8 tab[64];
   U8 busy=FALSE;
   U8 pipe_out;
   U8 pipe_in;
   U8 pipe_interrupt_in;


/**
 * @brief This function initializes the high level host application
 * Here initialize specific hardware ressources requirements.
 *
 * @param none
 *
 * @return none
 *
 */
void host_template_task_init(void)
{
}

//!
//! @brief High level host application task entry point
//! Put here the code of your host application.
//! The sample code just sends and receives 64bytes from
//! IN and OUT pipes
//!
//! @param none
//!
//! @return none
//!
void host_template_task(void)
{
   #if (USB_HOST_PIPE_INTERRUPT_TRANSFER == DISABLE)
   U8 sta;
   #endif
   U16 *ptr_nb;
   U16 nb;
   U8 i;

   ptr_nb=&nb;

   // First check the host controller is in full operating mode with the B device attached
   // and enumerated
   if(Is_host_ready())
   {
      //Put here the code to do in host mode

      // New device connection (executed  only one time after device connection)
      if(Is_new_device_connection_event())
      {
         for(i=0;i<Get_nb_supported_interface();i++)
         {
            // First interface with two bulk IN/OUT pipes
            if(Get_class(i)==0x00 && Get_subclass(i)==0x00 && Get_protocol(i)==0x00)
            {
               //Get correct physical pipes associated to IN/OUT Endpoints
               if(Is_ep_addr_in(Get_ep_addr(i,0)))
               {  //Yes associate it to the IN pipe
                  pipe_in=host_get_hwd_pipe_nb(Get_ep_addr(i,0));
                  pipe_out=host_get_hwd_pipe_nb(Get_ep_addr(i,1));
               }
               else
               {  //No, invert...
                  pipe_in=host_get_hwd_pipe_nb(Get_ep_addr(i,1));
                  pipe_out=host_get_hwd_pipe_nb(Get_ep_addr(i,0));
               }
            }
            // Seconf interface with interrupt IN pipe
            if(Get_class(i)==0x00 && Get_subclass(i)==0x55 && Get_protocol(i)==0xAA)
            {
               pipe_interrupt_in=host_get_hwd_pipe_nb(Get_ep_addr(i,0));
               Host_select_pipe(pipe_interrupt_in);
               Host_continuous_in_mode();
               Host_unfreeze_pipe();
            }
         }
      }


     // Firt interface (bulk IN/OUT) management
     // In polling mode
#if (USB_HOST_PIPE_INTERRUPT_TRANSFER == DISABLE)
      // The sample task sends 64 byte through pipe nb2...
      sta=host_send_data(pipe_out,64,tab);

      // And receives 64bytes from pipe nb 1...
      *ptr_nb=64;
      sta=host_get_data(pipe_in,ptr_nb,tab);
#else
      // similar applicative task under interrupt mode...
      if (busy==FALSE)
      {
         busy=TRUE;
         host_send_data_interrupt(pipe_out,64,tab,call_back_template_1);
      }

#endif

      // Second interface management (USB interrupt IN pipe)
      Host_select_pipe(pipe_interrupt_in);
      if(Is_host_in_received())
      {
         if(Is_host_stall()==FALSE)
         {
            i=Host_read_byte();
            Host_read_byte();
         }
         Host_ack_in_received(); Host_send_in();
      }
   }

   //Device disconnection...
   if(Is_device_disconnection_event())
   {
      //Put here code to be executed upon device disconnection...
   }
}

void call_back_template_1(U8 status, U16 nb_byte)
{
   if (status==PIPE_GOOD)
   {
      host_get_data_interrupt(pipe_in,64,tab,call_back_template_2);
   }

}

void call_back_template_2(U8 status, U16 nb_byte)
{
   if (status==PIPE_GOOD)
   {
      busy=FALSE;
   }
}
