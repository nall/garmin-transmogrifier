/*This file has been prepared for Doxygen automatic documentation generation.*/
//! \file *********************************************************************
//!
//! \brief This file manages the host enumeration process.
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

//_____ I N C L U D E S ____________________________________________________

#include "config.h"
#include "conf_usb.h"
#include "lib_mcu/usb/usb_drv.h"
#include "usb_host_enum.h"
#include "modules/usb/usb_task.h"
#include "usb_host_task.h"

//_____ M A C R O S ________________________________________________________


//_____ D E F I N I T I O N ________________________________________________

//_____ P R I V A T E   D E C L A R A T I O N ______________________________

#if (USB_HOST_FEATURE == DISABLED)
   #warning trying to compile a file used with the USB HOST without USB_HOST_FEATURE enabled
#endif


#ifndef SAVE_INTERRUPT_PIPE_FOR_DMS_INTERFACE
   #define SAVE_INTERRUPT_PIPE_FOR_DMS_INTERFACE   ENABLE
#endif
      
#if (USB_HOST_FEATURE == ENABLED)
extern S_usb_setup_data usb_request;



#ifndef VID_PID_TABLE
   #error VID_PID_TABLE shoud be defined somewhere (conf_usb.h)
  /*   VID_PID_TABLE format definition:
  //   #define VID_PID_TABLE      {VID1, number_of_pid_for_this_VID1, PID11_value,..., PID1X_Value \
  //                              ...\
  //                              ,VIDz, number_of_pid_for_this_VIDz, PIDz1_value,..., PIDzX_Value}
  */
#endif

#ifndef CLASS_SUBCLASS_PROTOCOL
   #error CLASS_SUBCLASS_PROTOCOL shoud be defined somewhere (conf_usb.h)
  /*   CLASS_SUBCLASS_PROTOCOL format definition:
  //   #define CLASS_SUBCLASS_PROTOCOL  {CLASS1, SUB_CLASS1,PROTOCOL1, \
  //                                     ...\
  //                                     CLASSz, SUB_CLASSz,PROTOCOLz}
  */
#endif


//! Const table of known devices (see conf_usb.h for table content)
U16 registered_VID_PID[]   = VID_PID_TABLE;

//! Const table of known class (see conf_usb.h for table content)
U8  registered_class[]     = CLASS_SUBCLASS_PROTOCOL;

//! The main structure that represents the usb tree connected to the host controller
S_usb_tree usb_tree;

#if (USB_HUB_SUPPORT==ENABLE && USER_PERIODIC_PIPE==ENABLE)
U8 user_periodic_pipe=0;
U8 user_periodic_pipe_freeze_state=0;
U8 user_periodic_pipe_device_index=0;
#endif



U8 selected_device=0;



//_____ D E C L A R A T I O N ______________________________________________

/**
 * host_check_VID_PID
 *
 * @brief This function checks if the VID and the PID are supported
 * (if the VID/PID belongs to the VID_PID table)
 *
 * @param none
 *
 * @return status
 */
U8 host_check_VID_PID(void)
{
U8  c,d;

   // Rebuild VID PID from data stage
   LSB(usb_tree.device[selected_device].vid) = data_stage[OFFSET_FIELD_LSB_VID];
   MSB(usb_tree.device[selected_device].vid) = data_stage[OFFSET_FIELD_MSB_VID];
   LSB(usb_tree.device[selected_device].pid) = data_stage[OFFSET_FIELD_LSB_PID];
   MSB(usb_tree.device[selected_device].pid) = data_stage[OFFSET_FIELD_MSB_PID];

   // Compare detected VID PID with supported table
   c=0;
   while (c< sizeof(registered_VID_PID)/2)   // /2 because registered_VID_PID table is U16...
   {
      if (registered_VID_PID[c] == Get_VID())   // VID is correct
      {
         d = (U8)registered_VID_PID[c+1];    // store nb of PID for this VID
         while (d != 0)
         {
            if (registered_VID_PID[c+d+1] == Get_PID())
            {
               return HOST_TRUE;
            }
            d--;
         }
      }
      c+=registered_VID_PID[c+1]+2;
   }
   return HOST_FALSE;
}

/**
 * host_check_class
 *
 * @brief This function checks if the device class is supported.
 * The function looks in all interface declared in the received dewcriptors, if
 * one of them match with the CLASS/SUB_CLASS/PROTOCOL table
 *
 * @param none
 *
 * @return status
 */
U8 host_check_class(void)
{
U8  c;
T_DESC_OFFSET  descriptor_offset;
T_DESC_OFFSET  conf_offset_end;
U16  config_size;
U8  device_class;
U8  device_subclass;
U8  device_protocol;
U8  nb_interface_supported;


   nb_interface_supported=0;   //First asumes ,no interface is supported!
   if (data_stage[OFFSET_FIELD_DESCRIPTOR_TYPE] != DESCRIPTOR_CONFIGURATION)           // check if configuration descriptor
   { return HOST_FALSE;}
   LSB(config_size) = data_stage[OFFSET_FIELD_TOTAL_LENGHT];
   MSB(config_size) = data_stage[OFFSET_FIELD_TOTAL_LENGHT+1];
   usb_tree.device[selected_device].bmattributes = data_stage[OFFSET_FIELD_BMATTRIBUTES];
   usb_tree.device[selected_device].maxpower = data_stage[OFFSET_FIELD_MAXPOWER];
   descriptor_offset = 0;
   conf_offset_end = descriptor_offset + config_size;

   // Look in all interfaces declared in the configuration
   while(descriptor_offset < conf_offset_end)
   {
      // Find next interface descriptor
      while (data_stage[descriptor_offset+OFFSET_FIELD_DESCRIPTOR_TYPE] != DESCRIPTOR_INTERFACE)
      {
         descriptor_offset += data_stage[descriptor_offset];
         if(descriptor_offset >= conf_offset_end)
         {
            if(nb_interface_supported)
            {
               return HOST_TRUE;
            }
            else return HOST_FALSE;
         }
      }
      // Found an interface descriptor
      // Get charateristics of this interface
      device_class    = data_stage[descriptor_offset + OFFSET_FIELD_CLASS];
      device_subclass = data_stage[descriptor_offset + OFFSET_FIELD_SUB_CLASS];
      device_protocol = data_stage[descriptor_offset + OFFSET_FIELD_PROTOCOL];
      // Look in registered class table for match
      c=0;
      while (c< sizeof(registered_class))
      {
         if (registered_class[c] == device_class)                 // class is correct!
         {
            if (registered_class[c+1] == device_subclass)         // sub class is correct!
            {
               if (registered_class[c+2] == device_protocol)      // protocol is correct!
               {
                  // Prepare for another item CLASS/SUB_CLASS/PROTOCOL in table
                  c+=3;
                  // Store this interface as supported interface
                  // Memorize its interface nb
                  usb_tree.device[selected_device].interface[nb_interface_supported].interface_nb=data_stage[descriptor_offset+OFFSET_FIELD_INTERFACE_NB];
                  //          its alternate setting
                  usb_tree.device[selected_device].interface[nb_interface_supported].altset_nb=data_stage[descriptor_offset+OFFSET_FIELD_ALT];
                  //          its USB class
                  usb_tree.device[selected_device].interface[nb_interface_supported].class=device_class;
                  //          its USB subclass
                  usb_tree.device[selected_device].interface[nb_interface_supported].subclass=device_subclass;
                  //          its USB protocol
                  usb_tree.device[selected_device].interface[nb_interface_supported].protocol=device_protocol;
                  //          the number of endpoints associated to this interface
                  //          Note: The associated endpoints addresses are stored during pipe attribution...
                  usb_tree.device[selected_device].interface[nb_interface_supported].nb_ep=data_stage[descriptor_offset+OFFSET_FIELS_NB_OF_EP];
                  // Update the number of interface supported
                  nb_interface_supported++;
                  // Update the number of interface supported for the device
                  usb_tree.device[selected_device].nb_interface++;
                  // Check the maximum number of interfaces we can support
                  if(nb_interface_supported>=MAX_INTERFACE_FOR_DEVICE)
                  {
                     return HOST_TRUE;
                  }
               }
            }
         }
         c+=3; // Check other item CLASS/SUB_CLASS/PROTOCOL in table
      }
      descriptor_offset += data_stage[descriptor_offset]; // Next descriptor
      if(descriptor_offset > SIZEOF_DATA_STAGE)           // Check overflow
      {
         if(nb_interface_supported)
         {
            return HOST_TRUE;
         }
         else return HOST_FALSE;
      }
   }
   if(nb_interface_supported)
   { 
      return HOST_TRUE; 
   }
   else return HOST_FALSE;
}

 /**
 * @brief This function configures the pipe according to the device class of the
 * interface selected.
 * THe function will configure the pipe corresponding to the last device selected
 *
 * @return status
 */
U8 host_auto_configure_endpoint()
{
U8  nb_endpoint_to_configure;
T_DESC_OFFSET  descriptor_offset;
U8  physical_pipe=1;   // =1 cause lookup table assumes that physical pipe 0 is reserved for control
U8 i;
U8 ep_index;
U8 device,interface,alt_interface;
U8 nb_interface;

   // Get the last device number to configure
   device = usb_tree.nb_device-1;
   
   // Get the nulber of interface to configure for the last device connected
   nb_interface = usb_tree.device[device].nb_interface;
   
   // Look for first physical pipe free
   // TODO improve allocation mechanism...
   i = Host_get_selected_pipe();    // Save current selected pipe
   for(physical_pipe=1;physical_pipe<MAX_EP_NB-1;physical_pipe++)
   {
      Host_select_pipe(physical_pipe);
      if(Is_host_pipe_memory_allocated()==FALSE) break; // Pipe already allocated try next one
   }
   Host_select_pipe(i); //Restore previous selected pipe

   // For all interfaces to configure...
   for(i=0;i<nb_interface;i++)
   {
      ep_index=0;
      // Look for the target interface descriptor offset
      interface = usb_tree.device[device].interface[i].interface_nb;
      alt_interface = usb_tree.device[device].interface[i].altset_nb;
      descriptor_offset = get_interface_descriptor_offset(interface,alt_interface);
      // Get the number of endpoint to configure for this interface
      nb_endpoint_to_configure = usb_tree.device[device].interface[i].nb_ep;
      // Get the first Endpoint descriptor offset to configure
      descriptor_offset += data_stage[descriptor_offset+OFFSET_DESCRIPTOR_LENGHT];  // pointing on endpoint descriptor

      // While there is at least one pipe to configure
      while (nb_endpoint_to_configure)
      {
         // Check and look for an Endpoint descriptor
         while (data_stage[descriptor_offset+OFFSET_FIELD_DESCRIPTOR_TYPE] != DESCRIPTOR_ENDPOINT)
         {
            descriptor_offset += data_stage[descriptor_offset];
            if(descriptor_offset > SIZEOF_DATA_STAGE)   // No more endpoint descriptor found -> Errror !
            {  return HOST_FALSE; }
         }
#if (SAVE_INTERRUPT_PIPE_FOR_DMS_INTERFACE==ENABLE)
         // @TODO HUB support & INTERRUPT PIPE No validated
         if(data_stage[descriptor_offset+OFFSET_FIELD_EP_TYPE]==TYPE_INTERRUPT && usb_tree.device[device].interface[i].class==0x08)
         {
            nb_endpoint_to_configure--;
            usb_tree.device[device].interface[i].nb_ep--;
            continue;
         }
#endif         

         // Select the new physical pipe to configure and get ride of any previous configuration for this physical pipe
         Host_select_pipe(physical_pipe);
         Host_disable_pipe();
         Host_unallocate_memory();
         Host_enable_pipe();

         // Build the pipe configuration according to the endpoint descriptors fields received
         //
         // host_configure_pipe(
         //    physical_pipe,                                                                    // pipe nb in USB interface
         //    data_stage[descriptor_offset+OFFSET_FIELD_EP_TYPE],                               // pipe type (interrupt/BULK/ISO)
         //    Get_pipe_token(data_stage[descriptor_offset+OFFSET_FIELD_EP_ADDR]),               // pipe addr
         //    (data_stage[descriptor_offset+2] & MSK_EP_DIR),                                   // pipe dir (IN/OUT)
         //    host_determine_pipe_size((U16)data_stage[descriptor_offset+OFFSET_FIELD_EP_SIZE]),// pipe size
         //    ONE_BANK,                                                                         // bumber of bank to allocate for pipe
         //    data_stage[descriptor_offset+OFFSET_FIELD_EP_INTERVAL]                            // interrupt period (for interrupt pipe)
         //  );

#if (USB_HUB_SUPPORT==ENABLE)
         // For USB hub move from interupt type to bulk type
         if(data_stage[descriptor_offset+OFFSET_FIELD_EP_TYPE]==TYPE_INTERRUPT && usb_tree.device[device].interface[i].class==0x09)
         {
               host_configure_pipe(physical_pipe, \
                             TYPE_BULK, \
                             TOKEN_IN,  \
                             1,   \
                             SIZE_8,      \
                             ONE_BANK,     \
                             0             );  
         }
         else // The device connected is not a hub
         {
            if( nb_hub_present==0) // No hub already exist in the USB tree
            {
               host_configure_pipe(                                                          \
                  physical_pipe,                                                             \
                  data_stage[descriptor_offset+OFFSET_FIELD_EP_TYPE],                        \
                  Get_pipe_token(data_stage[descriptor_offset+OFFSET_FIELD_EP_ADDR]),        \
                  (data_stage[descriptor_offset+OFFSET_FIELD_EP_ADDR] & MSK_EP_DIR),                            \
                  host_determine_pipe_size((U16)data_stage[descriptor_offset+OFFSET_FIELD_EP_SIZE]),\
                  TWO_BANKS,                                                                  \
                  data_stage[descriptor_offset+OFFSET_FIELD_EP_INTERVAL]                     \
               );
            }
            else // At least one hub is present in the usb tree
            {
               if(data_stage[descriptor_offset+OFFSET_FIELD_EP_TYPE]==TYPE_BULK ) // If BULK type : OK
               {
                     host_configure_pipe(                                                          \
                     physical_pipe,                                                             \
                     data_stage[descriptor_offset+OFFSET_FIELD_EP_TYPE],                        \
                     Get_pipe_token(data_stage[descriptor_offset+OFFSET_FIELD_EP_ADDR]),        \
                     (data_stage[descriptor_offset+OFFSET_FIELD_EP_ADDR] & MSK_EP_DIR),                            \
                     host_determine_pipe_size((U16)data_stage[descriptor_offset+OFFSET_FIELD_EP_SIZE]),\
                     TWO_BANKS,                                                                  \
                     data_stage[descriptor_offset+OFFSET_FIELD_EP_INTERVAL]                     \
                  ); 
               }
               #if (USER_PERIODIC_PIPE==ENABLE)
               else
               if(user_periodic_pipe==0 )                  
               {
                  user_periodic_pipe=physical_pipe;
                  user_periodic_pipe_device_index=device;
                  host_configure_pipe(                                                          \
                     physical_pipe,                                                             \
                     data_stage[descriptor_offset+OFFSET_FIELD_EP_TYPE],                        \
                     Get_pipe_token(data_stage[descriptor_offset+OFFSET_FIELD_EP_ADDR]),        \
                     (data_stage[descriptor_offset+OFFSET_FIELD_EP_ADDR] & MSK_EP_DIR),                            \
                     host_determine_pipe_size((U16)data_stage[descriptor_offset+OFFSET_FIELD_EP_SIZE]),\
                     ONE_BANK,                                                                  \
                     data_stage[descriptor_offset+OFFSET_FIELD_EP_INTERVAL]                     \
                  );                  
               }
               else
               {
                  nb_endpoint_to_configure--;
                  usb_tree.device[device].interface[i].nb_ep--;
                  continue;                  
               }
               #endif
            }
         }
#else // NO HUB SUPPORT
         host_configure_pipe(                                                          \
            physical_pipe,                                                             \
            data_stage[descriptor_offset+OFFSET_FIELD_EP_TYPE],                        \
            Get_pipe_token(data_stage[descriptor_offset+OFFSET_FIELD_EP_ADDR]),        \
            (data_stage[descriptor_offset+OFFSET_FIELD_EP_ADDR] & MSK_EP_DIR),                            \
            host_determine_pipe_size((U16)data_stage[descriptor_offset+OFFSET_FIELD_EP_SIZE]),\
            TWO_BANKS,                                                                 \
            data_stage[descriptor_offset+OFFSET_FIELD_EP_INTERVAL]                     \
         );
#endif          
         
         // Update endpoint addr table in supported interface structure
         usb_tree.device[device].interface[i].ep[ep_index].ep_addr = data_stage[descriptor_offset+OFFSET_FIELD_EP_ADDR];
         // Update physical pipe number used for this endpoint
         usb_tree.device[device].interface[i].ep[ep_index].pipe_number = physical_pipe;
         // Update endpoint size in supported interface structure
         usb_tree.device[device].interface[i].ep[ep_index].ep_size = data_stage[descriptor_offset+OFFSET_FIELD_EP_SIZE];
         // Update endpoint type in supported interface structure
         usb_tree.device[device].interface[i].ep[ep_index].ep_type = data_stage[descriptor_offset+OFFSET_FIELD_EP_TYPE];
         // point on next descriptor
         descriptor_offset += data_stage[descriptor_offset];        
         // Use next physical pipe
         if (physical_pipe++>=MAX_EP_NB)
         {
             return HOST_FALSE;
         }
         // To configure next endpoint 
         ep_index++;
         // All target endpoints configured ?
         nb_endpoint_to_configure--;
      } //for(i=0;i<nb_interface;i++)
   }
   Host_set_configured();
   return HOST_TRUE;
}

/**
 * get_interface_descriptor_offset
 *
 * @brief This function returns the offset in data_stage where to find the interface descriptor
 * whose number and alternate setting values are passed as parameters
 *
 * @param interface the interface nb to look for offset descriptor
 * @param alt the interface alt setting number
 *
 * @return T_DESC_OFFSET offset in data_stage[]
 */
T_DESC_OFFSET get_interface_descriptor_offset(U8 interface, U8 alt)
{
   U8 nb_interface;
   T_DESC_OFFSET descriptor_offset;

   nb_interface = data_stage[OFFSET_FIELD_NB_INTERFACE];      // Detects the number of interfaces in this configuration
   descriptor_offset = data_stage[OFFSET_DESCRIPTOR_LENGHT];  // now pointing on next descriptor

   while(descriptor_offset < SIZEOF_DATA_STAGE)            // Look in all interfaces declared in the configuration
   {
      while (data_stage[descriptor_offset+OFFSET_FIELD_DESCRIPTOR_TYPE] != DESCRIPTOR_INTERFACE)
      {
         descriptor_offset += data_stage[descriptor_offset];
         if(descriptor_offset > SIZEOF_DATA_STAGE)
         {  return HOST_FALSE;  }
      }
      if (data_stage[descriptor_offset+OFFSET_FIELD_INTERFACE_NB]==interface
          && data_stage[descriptor_offset+OFFSET_FIELD_ALT]==alt)
      {
        return  descriptor_offset;
      }
      descriptor_offset += data_stage[descriptor_offset];
   }
   return descriptor_offset;
}

/**
 * @brief This function returns the physical pipe number linked to a logical
 * endpoint address.
 *
 * @param ep_addr
 *
 * @return physical_pipe_number
 *
 * @note the function returns 0 if no ep_addr is found in the look up table.
 */
U8 host_get_hwd_pipe_nb(U8 ep_addr)
{
   U8 i,j;
   for(j=0;j<MAX_INTERFACE_FOR_DEVICE;j++)
   {
      for(i=0;i<MAX_EP_PER_INTERFACE;i++)
      {
         if(usb_tree.device[selected_device].interface[j].ep[i].ep_addr==ep_addr)
         { return usb_tree.device[selected_device].interface[j].ep[i].pipe_number; }
      }
   }
   return 0;
}

/**
 * host_send_control.
 *
 * @brief This function is the generic Pipe 0 management function
 * This function is used to send and receive control request over pipe 0
 *
 * @todo Fix all timeout errors and disconnection in active wait loop
 *
 * @param data_pointer
 *
 * @return status
 *
 * @note This function uses the usb_request global structure as parameter.
 * Thus this structure should be filled before calling this function.
 *
 */
U8 host_send_control(U8* data_pointer)
{
U16 data_length;
U8 sav_int_sof_enable;
U8 c;
U8 ep_size_max;

#if (USB_HUB_SUPPORT==ENABLE && USER_PERIODIC_PIPE==ENABLE)
   freeze_user_periodic_pipe();
#endif 

   ep_size_max = usb_tree.device[selected_device].ep_ctrl_size;
   Host_configure_address(usb_tree.device[selected_device].device_address);

   Usb_ack_event(EVT_HOST_SOF);
   sav_int_sof_enable=Is_host_sof_interrupt_enabled();
   Host_enable_sof_interrupt();                   // SOF software detection is in interrupt sub-routine
   while(Is_not_usb_event(EVT_HOST_SOF))          // Wait 1 sof
   {
      if (Is_host_emergency_exit())
      {
         c=CONTROL_TIMEOUT;
         Host_freeze_pipe();
         Host_reset_pipe(0);
         goto host_send_control_end;
      }
   }
   if (sav_int_sof_enable==FALSE)
   {
      Host_disable_sof_interrupt();
   }

   Host_select_pipe(0);
   Host_set_token_setup();
   Host_ack_setup();
   Host_unfreeze_pipe();
  // Build the setup request fields
   Host_write_byte(usb_request.bmRequestType);
   Host_write_byte(usb_request.bRequest);
   Host_write_byte(LSB(usb_request.wValue));
   Host_write_byte(MSB(usb_request.wValue));
   Host_write_byte(LSB(usb_request.wIndex));
   Host_write_byte(MSB(usb_request.wIndex));
   Host_write_byte(LSB(usb_request.wLength));
   Host_write_byte(MSB(usb_request.wLength));

   Host_send_setup();
   while(Is_host_setup_sent() == FALSE)  // wait for SETUP ack
   {
      if (Is_host_emergency_exit())
      {
         c=CONTROL_TIMEOUT;
         Host_freeze_pipe();
         Host_reset_pipe(0);
         goto host_send_control_end;
      }
      if(Is_host_pipe_error())           // Any error ?
      {
         c = Host_error_status();
         Host_ack_all_errors();
         goto host_send_control_end;     // Send error status
      }
   }
  // Setup token sent now send In or OUT token
  // Before just wait one SOF
   Usb_ack_event(EVT_HOST_SOF);
   sav_int_sof_enable=Is_host_sof_interrupt_enabled();
   Host_enable_sof_interrupt();
   Host_freeze_pipe();
   data_length = usb_request.wLength;
   while(Is_not_usb_event(EVT_HOST_SOF))         // Wait 1 sof
   {
      if (Is_host_emergency_exit())
      {
         c=CONTROL_TIMEOUT;
         Host_freeze_pipe();
         Host_reset_pipe(0);
         goto host_send_control_end;
      }
   }
   if (sav_int_sof_enable==FALSE)
   {  Host_disable_sof_interrupt();  }   // Restore SOF interrupt enable

  // IN request management ---------------------------------------------
   if(usb_request.bmRequestType & USB_SETUP_DIR_DEVICE_TO_HOST)
   {
      Host_standard_in_mode();
      Host_set_token_in();
      while(data_length != 0)
      {
         Host_unfreeze_pipe();
         while(!Is_host_control_in_received())
         {
            if (Is_host_emergency_exit())
            {
               c=CONTROL_TIMEOUT;
               Host_freeze_pipe();
               Host_reset_pipe(0);
               goto host_send_control_end;
            }
            if(Is_host_pipe_error())
            {
               c = Host_error_status();
               Host_ack_all_errors();
               goto host_send_control_end;
            }
            if(Is_host_stall())
            {
               c=CONTROL_STALL;
               Host_ack_stall();
               goto host_send_control_end;
            }
         }
         c = Host_data_length_U8();
         if (c == ep_size_max)
         {
            data_length -= c;
            if (usb_request.uncomplete_read == TRUE)           // uncomplete_read
            {
               data_length = 0;
            }
         }
         else
         {
            data_length = 0;
         }
         while (c!=0)
         {
            *data_pointer = Host_read_byte();
            data_pointer++;
            c--;
         }
         Host_freeze_pipe();
         Host_ack_control_in();
         Host_send_control_in();
      }                                // end of IN data stage

      Host_set_token_out();
      Host_unfreeze_pipe();
      Host_ack_control_out();
      Host_send_control_out();
      while(!Is_host_control_out_sent())
      {
         if (Is_host_emergency_exit())
         {
            c=CONTROL_TIMEOUT;
            Host_freeze_pipe();
            Host_reset_pipe(0);
            goto host_send_control_end;
         }
         if(Is_host_pipe_error())
         {
            c = Host_error_status();
            Host_ack_all_errors();
            goto host_send_control_end;
         }
         if(Is_host_stall())
         {
            c=CONTROL_STALL;
            Host_ack_stall();
            goto host_send_control_end;
         }
      }
      Host_ack_control_out();
      c=(CONTROL_GOOD);
      goto host_send_control_end;
   }

  // OUT request management ---------------------------------------------
   else                                 // Data stage OUT (bmRequestType==0)
   {
      Host_set_token_out();
      Host_ack_control_out();
      while(data_length != 0)
      {
         Host_unfreeze_pipe();
         c = ep_size_max;
         if ( ep_size_max > data_length)
         {
            c = (U8)data_length;
            data_length = 0;
         }
         else
         {
            data_length -= c;
         }
         while (c!=0)
         {
            Host_write_byte(*data_pointer);
            data_pointer++;
            c--;
         }
         Host_send_control_out();
         while (!Is_host_control_out_sent())
         {
            if (Is_host_emergency_exit())
            {
               c=CONTROL_TIMEOUT;
               Host_freeze_pipe();
               Host_reset_pipe(0);
               goto host_send_control_end;
            }
            if(Is_host_pipe_error())
            {
               c = Host_error_status();
               Host_ack_all_errors();
               goto host_send_control_end;
            }
            if(Is_host_stall())
            {
               c=CONTROL_STALL;
               Host_ack_stall();
               goto host_send_control_end;
            }
         }
         Host_ack_control_out();
      }                                // end of OUT data stage
      Host_freeze_pipe();
      Host_set_token_in();
      Host_unfreeze_pipe();
      while(!Is_host_control_in_received())
      {
         if (Is_host_emergency_exit())
         {
            c=CONTROL_TIMEOUT;
            Host_freeze_pipe();
            Host_reset_pipe(0);
            goto host_send_control_end;
         }
         if(Is_host_pipe_error())
         {
            c = Host_error_status();
            Host_ack_all_errors();
            goto host_send_control_end;
         }
         if(Is_host_stall())
         {
            c=CONTROL_STALL;
            Host_ack_stall();
            goto host_send_control_end;
         }
      }
      Host_ack_control_in();
      Host_freeze_pipe();
      Host_send_control_in();
      c=(CONTROL_GOOD);
      goto host_send_control_end;
   }
host_send_control_end:
#if(USB_HUB_SUPPORT==ENABLE && USER_PERIODIC_PIPE==ENABLE)
   unfreeze_user_periodic_pipe();
#endif  
   return ((U8)c);
}

/**
 * init_usb_tree
 *
 * @brief This function initializes the usb_tree structure when no device is connected to the root downstream port
 *
 * @return none
 */
void init_usb_tree(void)
{
   U8 i;
  
   // Clear all device entry
   for(i=0;i<MAX_DEVICE_IN_USB_TREE;i++)
   {
      remove_device_entry(i);
   }
   // By default select device 0 (connected to root port)
   Host_select_device(0);

#if (USB_HUB_SUPPORT==ENABLE)
   #if (USER_PERIODIC_PIPE==ENABLE)
   user_periodic_pipe=0;
   user_periodic_pipe_device_index=0;
   #endif
   for(i=0;i<HUB_MAX_NB_PORT;i++)
   hub_init(i);
   for(i=0;i<USB_MAX_HUB_NUMBER;i++)   
   hub_device_address[i]=0;
#endif

   // Clear the number of device in the tree
   usb_tree.nb_device=0;

}

/**
 * remove_device_entry
 *
 * @brief This function reset the device entry required when a device disconnect from the usb tree
 *
 * If the removed device is a hub, the function also removes the devices connected to this hub.
 * The function is also in charge of the USB DPRAM desallocation mechanism. DPRAM is deallocated only if the removed device
 * is the last device that allocate pipes.
 *
 * @param device_index the device numer index in the usb_tree that disconnects
 *
 * @return none
 */
void remove_device_entry(U8 device_index)
{
   U8 i,j,k,m;
   
#if (USB_HUB_SUPPORT==ENABLE)   
   // Check if the disconnected device is a hub
   if(usb_tree.device[device_index].interface[0].class==HUB_CLASS)
   {
      nb_hub_present--; 
      for(i=0;i<USB_MAX_HUB_NUMBER;i++)      // For entire hub table
      {  // Find the hub number that deconnects
         if(hub_device_address[i]==usb_tree.device[device_index].device_address)
         {  // Reset its port state machine
            for(j=0;j<HUB_MAX_NB_PORT;j++)  // For all ports 
            {         
               hub_port_state[i][j]=HUB_DEVICE_POWERED; 
            }
            // Look devices previously connected to this hub and remove ther entries
            for(j=0;j<MAX_DEVICE_IN_USB_TREE;j++)
            {
               if(usb_tree.device[j].parent_hub_number==(i+1))
               {
                  usb_tree.nb_device--;
                  // Caution recursive function !!!!
                  remove_device_entry(j);                  
               }
            }
            hub_device_address[i]=0;
            break;
         }
      }
   }
#endif   

   // Unallocate USB pipe memory only if the device disconnection is the last device 
   // enumerated
   // TODO: Improve it for devices removing from hub.
   // But cannot unallocate USB DPRAM if there is an active pipe number > current pipe for the removed device....
   if(usb_tree.nb_device==device_index+1)
   {
      for(j=0;j<MAX_INTERFACE_FOR_DEVICE;j++)
      {
         for(k=0;k<MAX_EP_PER_INTERFACE;k++)
         {
            m=usb_tree.device[device_index].interface[j].ep[k].pipe_number;
            if(m!=0)
            {
               i = Host_get_selected_pipe();
               Host_select_pipe(m);
               Host_unallocate_memory();
               Host_select_pipe(i);
            }
         }
      }
   }
   // Reset device entry fields ...
   usb_tree.device[device_index].device_address = 0;
   usb_tree.device[device_index].ep_ctrl_size = 0;
   usb_tree.device[device_index].hub_port_nb = 0;
   usb_tree.device[device_index].parent_hub_number = 0;
   usb_tree.device[device_index].nb_interface = 0;
   usb_tree.device[device_index].pid = 0;
   usb_tree.device[device_index].vid = 0;
   usb_tree.device[device_index].bmattributes = 0;   
   usb_tree.device[device_index].maxpower = 0;   
   
   for(j=0;j<MAX_INTERFACE_FOR_DEVICE;j++)
   {
      usb_tree.device[device_index].interface[j].interface_nb = 0;
      usb_tree.device[device_index].interface[j].altset_nb = 0;
      usb_tree.device[device_index].interface[j].class = 0;
      usb_tree.device[device_index].interface[j].subclass = 0;
      usb_tree.device[device_index].interface[j].protocol = 0;
      usb_tree.device[device_index].interface[j].nb_ep = 0;
      for(k=0;k<MAX_EP_PER_INTERFACE;k++)
      {
         usb_tree.device[device_index].interface[j].ep[k].ep_addr = 0;
         usb_tree.device[device_index].interface[j].ep[k].pipe_number = 0;
         usb_tree.device[device_index].interface[j].ep[k].ep_size = 0;
         usb_tree.device[device_index].interface[j].ep[k].ep_type = 0;
      }
   }   
}


// @TODO USER_PERIODIC_PIPE==ENABLE not validated
#if (USB_HUB_SUPPORT==ENABLE && USER_PERIODIC_PIPE==ENABLE)
void freeze_user_periodic_pipe(void)
{
   if(user_periodic_pipe)
   {
      Host_select_pipe(user_periodic_pipe);
      if(Is_host_pipe_freeze()) 
      {
         user_periodic_pipe_freeze_state=0;
      }
      else
      {
         user_periodic_pipe_freeze_state=1;
         Host_freeze_pipe();
      }
   } 
}
#endif

#if (USB_HUB_SUPPORT==ENABLE && USER_PERIODIC_PIPE==ENABLE)
void unfreeze_user_periodic_pipe(void)
{
   if(user_periodic_pipe)
   {
      if(user_periodic_pipe_freeze_state)
      {   
         Host_select_pipe(user_periodic_pipe);
         Host_unfreeze_pipe();
      }
      Host_select_device(user_periodic_pipe_device_index);
   }   
}
#endif

#if (USB_HUB_SUPPORT==ENABLE && USER_PERIODIC_PIPE==ENABLE)
void host_select_device(U8 i)
{
   freeze_user_periodic_pipe();
   selected_device=i;
   Host_configure_address(usb_tree.device[i].device_address);
}
#endif


#endif   //(USB_HOST_FEATURE == ENABLED)

