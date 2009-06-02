/*This file has been prepared for Doxygen automatic documentation generation.*/
//! \file *********************************************************************
//!
//! \brief This file manages the USB Host controller, the host enumeration process
//!
//!         and suspend resume host requests.
//!  This task dos not belongs to the scheduler tasks but is called directly from the general usb_task
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
#include "modules/usb/usb_task.h"
#include "usb_host_task.h"
#include "lib_mcu/usb/usb_drv.h"
#include "lib_mcu/pll/pll_drv.h"
#include "modules/usb/host_chap9/usb_host_enum.h"

#if (USB_HUB_SUPPORT == ENABLE)
#include "modules/usb/host_chap9/usb_host_hub.h"
#endif

#if (USB_HOST_FEATURE == DISABLED)
   #warning trying to compile a file used with the USB HOST without USB_HOST_FEATURE enabled
#endif

#if (USB_HOST_FEATURE == ENABLED)

#ifndef DEVICE_BASE_ADDRESS
   #error DEVICE_BASE_ADDRESS should be defined somewhere in config files (conf_usb.h)
#endif

#ifndef SIZEOF_DATA_STAGE
   #error SIZEOF_DATA_STAGE should be defined in conf_usb.h
#endif

#ifndef HOST_CONTINUOUS_SOF_INTERRUPT
   #error HOST_CONTINUOUS_SOF_INTERRUPT should be defined as ENABLE or DISABLE in conf_usb.h
#endif

#ifndef USB_HOST_PIPE_INTERRUPT_TRANSFER
   #error USB_HOST_PIPE_INTERRUPT_TRANSFER should be defined as ENABLE or DISABLE in conf_usb.h
#endif

#ifndef Usb_id_transition_action
   #define Usb_id_transition_action()
#endif
#ifndef  Host_device_disconnection_action
   #define Host_device_disconnection_action()
#endif
#ifndef  Host_device_connection_action
   #define Host_device_connection_action()
#endif
#ifndef  Host_sof_action
   #define Host_sof_action()
#endif
#ifndef  Host_suspend_action
   #define Host_suspend_action()
#endif
#ifndef  Host_hwup_action
   #define Host_hwup_action()
#endif
#ifndef  Host_device_not_supported_action
   #define Host_device_not_supported_action()
#endif
#ifndef  Host_device_class_not_supported_action
   #define Host_device_class_not_supported_action()
#endif
#ifndef  Host_device_supported_action
   #define Host_device_supported_action()
#endif
#ifndef  Host_device_error_action
   #define Host_device_error_action()
#endif


//_____ M A C R O S ________________________________________________________

#ifndef LOG_STR_CODE
#define LOG_STR_CODE(str)
#else
U8 code log_device_connected[]="Device Connection";
U8 code log_device_enumerated[]="Device Enumerated";
U8 code log_device_unsupported[]="Unsupported Device";
U8 code log_going_to_suspend[]="Usb suspend";
U8 code log_usb_resumed[]="Usb resumed";
#endif

//_____ D E F I N I T I O N S ______________________________________________

//_____ D E C L A R A T I O N S ____________________________________________

#if (USB_HOST_PIPE_INTERRUPT_TRANSFER == ENABLE)
   volatile S_pipe_int   it_pipe_str[MAX_EP_NB];
   volatile U8 pipe_nb_save;
   U8 g_sav_int_sof_enable;
#endif

//!
//! Public : U8 device_state
//! Its value represent the current state of the
//! device connected to the usb host controller
//! Value can be:
//! - DEVICE_ATTACHED
//! - DEVICE_POWERED
//! - DEVICE_SUSPENDED
//! - DEVICE_DEFAULT
//! - DEVICE_BASE_ADDRESSED
//! - DEVICE_CONFIGURED
//! - DEVICE_ERROR
//! - DEVICE_UNATTACHED
//! - DEVICE_READY
//! - DEVICE_WAIT_RESUME
//! - DEVICE_DISCONNECTED
//! - DEVICE_DISCONNECTED_ACK
//!/
U8 device_state;

//! For control requests management over pipe 0
S_usb_setup_data usb_request;

//!
//! Public : U8 data_stage[SIZEOF_DATA_STAGE];
//! Internal RAM buffer for USB data stage content
//! This buffer is required to setup host enumeration process
//! Its contains the device descriptors received.
//! Depending on the device descriptors lenght, its size can be optimized
//! with the SIZEOF_DATA_STAGE define of conf_usb.h file
//!
//!/
U8 data_stage[SIZEOF_DATA_STAGE];

U8 device_status;
U8 request_resume = FALSE;
U8 force_enumeration = FALSE;
U8 new_device_connected = 0;


static U16  c;                // As internal host start of frame counter


#if (USB_HUB_SUPPORT == ENABLE)
   static U8 i,j;
   volatile U8 hub_interrupt_sof=0;
   U8 saved_device;
#endif

/**
 * @brief This function initializes the USB controller in host mode and
 *        the associated variables.
 *
 * This function enables the USB controller for host mode operation.
 *
 * @param none
 *
 * @return none
 *
 */
void usb_host_task_init(void)
{
   Pll_start_auto();
   Wait_pll_ready();
   Usb_disable();
   Usb_enable();
   Usb_unfreeze_clock();
   Usb_attach();
   Usb_enable_uvcon_pin();
   Usb_select_host();
   Usb_disable_vbus_hw_control();   // Force Vbus generation without timeout
   Host_enable_device_disconnection_interrupt();
   device_state=DEVICE_UNATTACHED;
   init_usb_tree();
}

/**
 * @brief Entry point of the USB host management
 *
 * The aim is to manage the device target connection and enumeration
 * depending on the device_state, the function performs the required operations
 * to get the device enumerated and configured
 * Once the device is operationnal, the device_state value is DEVICE_READY
 * This state should be tested by the host task application before performing
 * any applicative requests to the device.
 *
 * @param none
 *
 * @return none
 *
 * \image html host_task.gif
 */
void usb_host_task(void)
{

   switch (device_state)
   {
     //------------------------------------------------------
     //   DEVICE_UNATTACHED state
     //
     //   - Default init state
     //   - Try to give device power supply
     //
      case DEVICE_UNATTACHED:
         Host_clear_device_supported();        // Reset Device status
         Host_clear_configured();
         Host_clear_device_ready();
         Usb_clear_all_event();                // Clear all software events
         new_device_connected=0;
         selected_device=0;
         
#if (USB_HUB_SUPPORT==ENABLE)
         nb_hub_present = 0;
#endif         
         
#if (SOFTWARE_VBUS_CTRL==ENABLE)
         if( Is_usb_bconnection_error_interrupt()||Is_usb_vbus_error_interrupt())
         {
            Usb_ack_bconnection_error_interrupt();
            Usb_ack_vbus_error_interrupt();
            Host_clear_vbus_request();
         }
         Usb_disable_vbus_pad();
         Usb_enable_manual_vbus();
         if(Is_usb_srp_interrupt())
         {
            Usb_ack_srp_interrupt();
            Usb_enable_vbus_pad();
            Usb_enable_vbus();
            device_state=DEVICE_ATTACHED;
         }
#else
         Usb_enable_vbus();                    // Give at least device power supply!!!
         if(Is_usb_vbus_high())
         { device_state=DEVICE_ATTACHED; }     // If VBUS ok goto to device connection expectation
#endif
      break;

     //------------------------------------------------------
     //   DEVICE_ATTACHED state
     //
     //   - Vbus is on
     //   - Try to detected device connection
     //
      case DEVICE_ATTACHED :
         if (Is_device_connection() || (force_enumeration==TRUE))     // Device pull-up detected
         {
            Host_ack_device_connection();
            Host_clear_device_supported();        // Reset Device status
            Host_clear_configured();
            Host_clear_device_ready();
            Usb_clear_all_event();                // Clear all software events
            new_device_connected=0;
            force_enumeration=FALSE;

           // Now device is connected, enable disconnection interrupt
            Host_enable_device_disconnection_interrupt();
            Enable_interrupt();
           // Reset device status
            Host_clear_device_supported();
            Host_clear_configured();
            Host_clear_device_ready();
            Host_enable_sof();            // Start Start Of Frame generation
            Host_enable_sof_interrupt();  // SOF will be detected under interrupt
            c = 0;
            while (c<100)               // wait 100ms before USB reset
            {
               if (Is_usb_event(EVT_HOST_SOF)) { Usb_ack_event(EVT_HOST_SOF); c++; }// Count Start Of frame
               if (Is_host_emergency_exit() || Is_usb_bconnection_error_interrupt()) {goto device_attached_error;}
            }
            Host_disable_device_disconnection_interrupt();
            Host_send_reset();          // First USB reset
            Usb_ack_event(EVT_HOST_SOF);
            while (Is_host_reset());    // Active wait of end of reset send
            Host_ack_reset();
            //Workaround for some bugly devices with powerless pull up
            //usually low speed where data line rise slowly and can be interpretaded as disconnection
            for(c=0;c!=0xFFFF;c++)    // Basic Timeout counter
            {
               if(Is_usb_event(EVT_HOST_SOF))   //If we detect SOF, device is still alive and connected, just clear false disconnect flag
               {
                  if(Is_device_disconnection())
                  {
                      Host_ack_device_connection();
                      Host_ack_device_disconnection();
                      break;
                  }
               }
            }
            Host_enable_device_disconnection_interrupt();
            // All USB pipes must be reconfigured after a USB reset generation
            host_configure_pipe(PIPE_CONTROL, \
                                            TYPE_CONTROL, \
                                            TOKEN_SETUP,  \
                                            EP_CONTROL,   \
                                            SIZE_64,      \
                                            ONE_BANK,     \
                                            0             );            
            c = 0;
            while (c<100)               // wait 100ms after USB reset
            {
               if (Is_usb_event(EVT_HOST_SOF)) { Usb_ack_event(EVT_HOST_SOF); c++; }// Count Start Of frame
               if (Is_host_emergency_exit() || Is_usb_bconnection_error_interrupt()) {goto device_attached_error;}
            }
            device_state = DEVICE_POWERED;
            c=0;
         }
         device_attached_error:
        // Device connection error, or vbus pb -> Retry the connection process from the begining
         if( Is_usb_bconnection_error_interrupt()||Is_usb_vbus_error_interrupt()||Is_usb_vbus_low())
         {
            Usb_ack_bconnection_error_interrupt();
            Usb_enable_vbus_hw_control();
            device_state=DEVICE_UNATTACHED;
            Usb_disable_vbus();
            Usb_disable_vbus_pad();
            Usb_enable_vbus_pad();
            Usb_ack_vbus_error_interrupt();
            Usb_enable_vbus();
            Usb_disable_vbus_hw_control();
            Host_disable_sof();
         }
         break;

     //------------------------------------------------------
     //   DEVICE_POWERED state
     //
     //   - Device connection (attach) as been detected,
     //   - Wait 100ms and configure default control pipe
     //
      case DEVICE_POWERED :
         LOG_STR_CODE(log_device_connected);
         Host_device_connection_action();
         if (Is_usb_event(EVT_HOST_SOF))
         {
            Usb_ack_event(EVT_HOST_SOF);
            if (c++ >= 100)                          // Wait 100ms
            {
               device_state = DEVICE_DEFAULT;
            }
         }
         break;

     //------------------------------------------------------
     //   DEVICE_DEFAULT state
     //
     //   - Get device descriptor
     //   - Reconfigure Pipe 0 according to Device EP0
     //   - Attribute device address
     //
      case DEVICE_DEFAULT :
        // Get first device descriptor
         Host_select_device(0);
         usb_tree.device[0].ep_ctrl_size=8;
         if( CONTROL_GOOD == host_get_device_descriptor_uncomplete())
         {
            c = 0;
            while(c<20)           // wait 20ms before USB reset (special buggly devices...)
            {
               if (Is_usb_event(EVT_HOST_SOF)) { Usb_ack_event(EVT_HOST_SOF); c++; }
               if (Is_host_emergency_exit() || Is_usb_bconnection_error_interrupt())  {break;}
            }
            Host_disable_device_disconnection_interrupt();
            Host_send_reset();          // First USB reset
            Usb_ack_event(EVT_HOST_SOF);
            while (Is_host_reset());    // Active wait of end of reset send
            Host_ack_reset();
            //Workaround for some bugly devices with powerless pull up
            //usually low speed where data line rise slowly and can be interpretaded as disconnection
            for(c=0;c!=0xFFFF;c++)    // Basic Timeout counter
            {
               if(Is_usb_event(EVT_HOST_SOF))   //If we detect SOF, device is still alive and connected, just clear false disconnect flag
               {
                  if(Is_device_disconnection())
                  {
                      Host_ack_device_connection();
                      Host_ack_device_disconnection();
                      break;
                  }
               }
            }
            Host_enable_device_disconnection_interrupt();
            c = 0;
            host_configure_pipe(PIPE_CONTROL, \
                                            TYPE_CONTROL, \
                                            TOKEN_SETUP,  \
                                            EP_CONTROL,   \
                                            SIZE_64,      \
                                            ONE_BANK,     \
                                            0             );            
            while(c<200)           // wait 200ms after USB reset
            {
               if (Is_usb_event(EVT_HOST_SOF)) { Usb_ack_event(EVT_HOST_SOF); c++; }
               if (Is_host_emergency_exit() || Is_usb_bconnection_error_interrupt())  {break;}
            }
            usb_tree.device[0].ep_ctrl_size=data_stage[OFFSET_FIELD_MAXPACKETSIZE];
            
            // Give an absolute device address
            host_set_address(DEVICE_BASE_ADDRESS);
            usb_tree.device[0].device_address=DEVICE_BASE_ADDRESS;
            device_state = DEVICE_ADDRESSED;
         }
         else
         {  device_state = DEVICE_ERROR; }
         break;

     //------------------------------------------------------
     //   DEVICE_BASE_ADDRESSED state
     //
     //   - Check if VID PID is in supported list
     //
      case DEVICE_ADDRESSED :
         if (CONTROL_GOOD == host_get_device_descriptor())
         {
           // Detect if the device connected belongs to the supported devices table
            if (HOST_TRUE == host_check_VID_PID())
            {
               Host_set_device_supported();
               Host_device_supported_action();
               device_state = DEVICE_CONFIGURED;
            }
            else
            {
               #if (HOST_STRICT_VID_PID_TABLE==ENABLE)
                  Host_device_not_supported_action();
                  device_state = DEVICE_ERROR;
               #else
                  device_state = DEVICE_CONFIGURED;
               #endif
            }
         }
         else // Can not get device descriptor
         {  device_state = DEVICE_ERROR; }
         break;

     //------------------------------------------------------
     //   DEVICE_CONFIGURED state
     //
     //   - Configure pipes for the supported interface
     //   - Send Set_configuration() request
     //   - Goto full operating mode (device ready)
     //
      case DEVICE_CONFIGURED :
         if (CONTROL_GOOD == host_get_configuration_descriptor())
         {
            if (HOST_FALSE != host_check_class()) // Class support OK?
            {
               usb_tree.nb_device++;
            #if (HOST_AUTO_CFG_ENDPOINT==ENABLE)
               if(host_auto_configure_endpoint())
            #else
               Host_set_configured();     // Assumes config is OK with user config
               if(User_configure_endpoint()) // User call here instead of autoconfig
            #endif
               {
                  if (CONTROL_GOOD== host_set_configuration(1))  // Send Set_configuration
                  {
                     //host_set_interface(interface_bound,interface_bound_alt_set);
                     // device and host are now fully configured
                     // goto DEVICE READY normal operation
                      device_state = DEVICE_READY;
                     // monitor device disconnection under interrupt
                      Host_enable_device_disconnection_interrupt();
                     // If user host application requires SOF interrupt event
                     // Keep SOF interrupt enable otherwize, disable this interrupt
                  #if (HOST_CONTINUOUS_SOF_INTERRUPT==DISABLE && USB_HUB_SUPPORT==DISABLE)
                      Host_disable_sof_interrupt();
                  #endif
                  #if (USB_HUB_SUPPORT==ENABLE)
                      // Check if the connected device is a hub
                      if(Get_class(0)==HUB_CLASS && Get_subclass(0)==0x00 && Get_protocol(0)==0x00)
                      {
                           // Get hub descriptor
                           if( Get_hub_descriptor()==CONTROL_GOOD)
                           {
                              // Power each port of the hub
                              i=data_stage[NB_PORT_OFFSET];
                              for(c=1;c<=i;c++)
                              {
                                 Set_port_feature(PORT_POWER,c);
                              }
                              nb_hub_present = 1;
                              hub_device_address[0]=DEVICE_BASE_ADDRESS;
                              hub_init(nb_hub_present-1);    
                           }
                      }
                      else
                      {
                         nb_hub_present = 0;
                         new_device_connected=TRUE;
                      }
                  #else
                      new_device_connected=TRUE;
                  #endif
                      
                      Enable_interrupt();
                      LOG_STR_CODE(log_device_enumerated);
                  }
                  else// Problem during Set_configuration request...
                  {   device_state = DEVICE_ERROR;  }
               }
            }
            else // device class not supported...
            {
                device_state = DEVICE_ERROR;
                LOG_STR_CODE(log_device_unsupported);
                Host_device_class_not_supported_action();
            }
         }
         else // Can not get configuration descriptors...
         {  device_state = DEVICE_ERROR; }
         break;

     //------------------------------------------------------
     //   DEVICE_READY state
     //
     //   - Full std operatinf mode
     //   - Nothing to do...
     //
      case DEVICE_READY:     // Host full std operating mode!
         new_device_connected=FALSE;
         
      #if (USB_HUB_SUPPORT==ENABLE)
         f_hub_port_disconnect=FALSE;
         // If one hub is present in the USB tree and the period interval
         // for the interrupt hub endpoint occurs
         if(nb_hub_present && hub_interrupt_sof==0)
         {
            saved_device=selected_device;      // Backup user selected device
            for(j=1;j<=nb_hub_present;j++)
            {
               for(i=0;i<MAX_DEVICE_IN_USB_TREE;i++)
               {
                  if(usb_tree.device[i].device_address==hub_device_address[j-1]) break;
               }
               Host_select_device(i);
               Host_select_pipe(usb_tree.device[i].interface[0].ep[0].pipe_number);
               Host_ack_nak_received();
               Host_ack_in_received();
               Host_unfreeze_pipe();
               Host_send_in();
               while(1)
               {
                   if(Is_host_nak_received())   break;
                   if(Is_host_emergency_exit()) break;
                   if(Is_host_in_received())    break;
               }
               Host_freeze_pipe();
               if(Is_host_nak_received())
               {
                  Host_ack_nak_received();
               }
               if(Is_host_in_received())
               {
                  if(Is_host_stall()==FALSE)
                  {
                     c=Host_read_byte();
                  }
                  Host_ack_in_received();
                  hub_manage_port_change_status(c,j);
               }
            } // for all hub 
            Host_select_device(saved_device);  // Restore user selected device
            #if (USER_PERIODIC_PIPE==ENABLE)
            unfreeze_user_periodic_pipe();
            #endif
         }
      #endif
         break;

     //------------------------------------------------------
     //   DEVICE_ERROR state
     //
     //   - Error state
     //   - Do custom action call (probably go to default mode...)
     //
      case DEVICE_ERROR :    // TODO !!!!
      #if (HOST_ERROR_RESTART==ENABLE)
         device_state=DEVICE_UNATTACHED;
      #endif
         Host_device_error_action();
         break;

     //------------------------------------------------------
     //   DEVICE_SUSPENDED state
     //
     //   - Host application request to suspend the device activity
     //   - State machine comes here thanks to Host_request_suspend()
     //
      case DEVICE_SUSPENDED :
         if(Is_device_supports_remote_wakeup()) // If the connected device supports remote wake up
         {
           host_set_feature_remote_wakeup();    // Enable this feature...
         }
         LOG_STR_CODE(log_going_to_suspend);
         c = Is_host_sof_interrupt_enabled(); //Save current sof interrupt enable state
         Host_disable_sof_interrupt();
         Host_ack_sof();
         Host_disable_sof();           // Stop start of frame generation, this generates the suspend state
         Host_ack_hwup();
         Host_enable_hwup_interrupt(); // Enable host wake-up interrupt
                                       // (this is the unique USB interrupt able to wake up the CPU core from power-down mode)
         Usb_freeze_clock();
         Stop_pll();
         Host_suspend_action();              // Custom action here! (for example go to power-save mode...)
         device_state=DEVICE_WAIT_RESUME;    // wait for device resume event
         break;

     //------------------------------------------------------
     //   DEVICE_WAIT_RESUME state
     //
     //   - Wait in this state till the host receives an upstream resume from the device
     //   - or the host software request the device to resume
     //
      case DEVICE_WAIT_RESUME :
         if(Is_usb_event(EVT_HOST_HWUP)|| Is_host_request_resume())// Remote wake up has been detected
                                                                 // or Local resume request has been received
         {
            if(Is_host_request_resume())       // Not a remote wakeup, but an host application request
            {
               Host_disable_hwup_interrupt();  // Wake up interrupt should be disable host is now wake up !
              // CAUTION HWUP can be cleared only when USB clock is active
               Pll_start_auto();               // First Restart the PLL for USB operation
               Wait_pll_ready();               // Get sure pll is lock
               Usb_unfreeze_clock();           // Enable clock on USB interface
               Host_ack_hwup();                // Clear HWUP interrupt flag
            }
            Host_enable_sof();
            Host_send_resume();                            // Send down stream resume
            while (Is_host_down_stream_resume()==FALSE);   // Wait Down stream resume sent
            Host_ack_remote_wakeup();        // Ack remote wake-up reception
            Host_ack_request_resume();       // Ack software request
            Host_ack_down_stream_resume();   // Ack down stream resume sent
            Usb_ack_event(EVT_HOST_HWUP);    // Ack software event
            if(c) { Host_enable_sof_interrupt(); } // Restore SOF interrupt enable state before suspend
            device_state=DEVICE_READY;       // Come back to full operating mode
            LOG_STR_CODE(log_usb_resumed);
         }
         break;

     //------------------------------------------------------
     //   DEVICE_DISCONNECTED state
     //
     //   - Device disconnection has been detected
     //   - Run scheduler in this state at least two times to get sure event is detected by all host application tasks
     //   - Go to DEVICE_DISCONNECTED_ACK state before DEVICE_UNATTACHED, to get sure scheduler calls all app tasks...
     //
      case DEVICE_DISCONNECTED :
         device_state = DEVICE_DISCONNECTED_ACK;
         break;

     //------------------------------------------------------
     //   DEVICE_DISCONNECTED_ACK state
     //
     //   - Device disconnection has been detected and managed bu applicatives tasks
     //   - Go to DEVICE_UNATTACHED state
     //
      case DEVICE_DISCONNECTED_ACK :
         device_state = DEVICE_UNATTACHED;
         break;

     //------------------------------------------------------
     //   default state
     //
     //   - Default case: ERROR
     //   - Goto no device state
     //
      default :
         device_state = DEVICE_UNATTACHED;
         break;
      }
}

//___ F U N C T I O N S   F O R   P O L L I N G   M A N A G E D   D A T A  F L O W S  _________________________

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
  * @return status
  */
U8 host_send_data(U8 pipe, U16 nb_data, U8 *buf)
{
   U8 c;
   U8 status=PIPE_GOOD;
   U8 sav_int_sof_enable;
   U8 nak_timeout;
   U16 cpt_nak;
   U8 nb_data_loaded;
   U8 cpt_err_timeout=0;

#if (USER_PERIODIC_PIPE==ENABLE)
   freeze_user_periodic_pipe();
#endif
   sav_int_sof_enable=Is_host_sof_interrupt_enabled();  // Save state of enable sof interrupt
   Host_enable_sof_interrupt();
   Host_select_pipe(pipe);
   Host_set_token_out();
   Host_ack_out_sent();
   Host_unfreeze_pipe();
   
   while (nb_data != 0)         // While there is something to send...
   {
     // Prepare data to be sent
      c = Host_get_pipe_length();
      if ( (U16)c > nb_data)
      {
         nb_data_loaded = (U8)nb_data;
         c = nb_data;
      }
      else
      {  nb_data_loaded = c; }
      while (c!=0)              // Load Pipe buffer
      {
         Host_write_byte(*buf++);
         c--;
      }
      private_sof_counter=0;    // Reset the counter in SOF detection sub-routine
      cpt_nak=0;
      nak_timeout=0;
      Host_ack_out_sent();
      Host_send_out();
      while (!Is_host_out_sent())
      {
         if (Is_host_emergency_exit())// Async disconnection or role change detected under interrupt
         {
            status=PIPE_DELAY_TIMEOUT;
            Host_reset_pipe(pipe);
            goto host_send_data_end;
         }
         #if (TIMEOUT_DELAY_ENABLE==ENABLE)
         if (private_sof_counter>=250)            // Count 250ms (250sof)
         {
            private_sof_counter=0;
            if (nak_timeout++>=TIMEOUT_DELAY) // Inc timeout and check for overflow
            {
               status=PIPE_DELAY_TIMEOUT;
               Host_reset_pipe(pipe);
               goto host_send_data_end;
            }
         }
         #endif
         if (Is_host_pipe_error()) // Any error ?
         {
            status = Host_error_status();
            Host_ack_all_errors();
            if(status == PIPE_TIMEOUT)
            {
               if(cpt_err_timeout++>100)
               {
                  goto host_send_data_end;
               }
               else
               {
                  c=0;
                  while(c<2)           // wait  2 ms
                  {
                     if (Is_usb_event(EVT_HOST_SOF)) { Usb_ack_event(EVT_HOST_SOF); c++; }
                     if (Is_host_emergency_exit() )  {break;}
                  }

                  Host_unfreeze_pipe();
               }
            }
         }
         if (Is_host_stall())      // Stall management
         {
            status =PIPE_STALL;
            Host_ack_stall();
            goto host_send_data_end;
         }
         #if (NAK_TIMEOUT_ENABLE==ENABLE)
         if(Is_host_nak_received())  //NAK received
         {
            Host_ack_nak_received();
            if (cpt_nak++>NAK_SEND_TIMEOUT)
            {
               status = PIPE_NAK_TIMEOUT;
               Host_reset_pipe(pipe);
               goto host_send_data_end;
            }
         }
         #endif
      }
      // Here OUT sent
      nb_data -= nb_data_loaded;
      status=PIPE_GOOD;         // Frame correctly sent
      Host_ack_out_sent();
   }
   while(0!=Host_number_of_busy_bank());
   
host_send_data_end:
   Host_freeze_pipe();
  // Restore sof interrupt enable state
   if (sav_int_sof_enable==FALSE)   {Host_disable_sof_interrupt();}
   #if (USER_PERIODIC_PIPE==ENABLE)
   unfreeze_user_periodic_pipe();
   #endif   
  // And return...
   return ((U8)status);
}



/**
  * @brief This function receives nb_data pointed with *buf with the pipe number specified
  *
  * The nb_data parameter is passed as a U16 pointer, thus the data pointed by this pointer
  * is updated with the final number of data byte received.
  *
  * @param pipe
  * @param nb_data
  * @param buf
  *
  * @return status
  */
U8 host_get_data(U8 pipe, U16 *nb_data, U8 *buf)
{
   U8 status=PIPE_GOOD;
   U8 sav_int_sof_enable;
   U8 nak_timeout;
   U16 n,i;
   U16 cpt_nak;
   
   #if (USER_PERIODIC_PIPE==ENABLE)
   freeze_user_periodic_pipe();   
   #endif
   n=*nb_data;
   *nb_data=0;
   sav_int_sof_enable=Is_host_sof_interrupt_enabled();
   Host_enable_sof_interrupt();
   Host_select_pipe(pipe);
   Host_continuous_in_mode();
   Host_set_token_in();
   Host_ack_in_received();
   while (n)              // While missing data...
   {
      Host_unfreeze_pipe();
      Host_send_in();
      private_sof_counter=0; // Reset the counter in SOF detection sub-routine
      nak_timeout=0;
      cpt_nak=0;
      while (!Is_host_in_received())
      {
         if (Is_host_emergency_exit())   // Async disconnection or role change detected under interrupt
         {
            status=PIPE_DELAY_TIMEOUT;
            Host_reset_pipe(pipe);
            goto host_get_data_end;
         }
         #if (TIMEOUT_DELAY_ENABLE==ENABLE)
         if (private_sof_counter>=250)   // Timeout management
         {
            private_sof_counter=0;       // Done in host SOF interrupt
            if (nak_timeout++>=TIMEOUT_DELAY)// Check for local timeout
            {
               status=PIPE_DELAY_TIMEOUT;
               Host_reset_pipe(pipe);
               goto host_get_data_end;
            }
         }
         #endif
         if(Is_host_pipe_error())        // Error management
         {
            status = Host_error_status();
            Host_ack_all_errors();
            goto host_get_data_end;
         }
         if(Is_host_stall())             // STALL management
         {
            status =PIPE_STALL;
            Host_reset_pipe(pipe);
            Host_ack_stall();
            goto host_get_data_end;
         }
         #if (NAK_TIMEOUT_ENABLE==ENABLE)
         if(Is_host_nak_received())  //NAK received
         {
            Host_ack_nak_received();
            if (cpt_nak++>NAK_RECEIVE_TIMEOUT)
            {
               status = PIPE_NAK_TIMEOUT;
               Host_reset_pipe(pipe);
               goto host_get_data_end;
            }
         }
         #endif
      }
      status=PIPE_GOOD;
      Host_freeze_pipe();
      if (Host_byte_counter()<=n)
      {
         if ((Host_byte_counter() < n)&&(Host_byte_counter()<Host_get_pipe_length()))
         { n=0;}
         else
         { n-=Host_byte_counter();}
         (*nb_data)+=Host_byte_counter();  // Update nb of byte received
         for (i=Host_byte_counter();i;i--)
         { *buf=Host_read_byte(); buf++;}
      }
      else  // more bytes received than expected
      {     // TODO error code management
         *nb_data+=n;
         for (i=n;i;i--)                  // Byte number limited to the initial request (limit tab over pb)
         {  *buf=Host_read_byte(); buf++; }
         n=0;
      }
      Host_ack_in_received();
   }
   Host_freeze_pipe();
host_get_data_end:
   if (sav_int_sof_enable==FALSE)
   {
      Host_disable_sof_interrupt();
   }
   #if (USER_PERIODIC_PIPE==ENABLE)
   unfreeze_user_periodic_pipe();
   #endif   
   return ((U8)status);
}


//___ F U N C T I O N S   F O R   I N T E R R U P T   M A N A G E D   D A T A   F L O W S  _________________________

#if (USB_HOST_PIPE_INTERRUPT_TRANSFER == ENABLE)

void reset_it_pipe_str(void)
{
   U8 i;
   for(i=0;i<MAX_EP_NB;i++)
   {
      it_pipe_str[i].enable=DISABLE;
      it_pipe_str[i].timeout=0;
   }
}

U8 is_any_interrupt_pipe_active(void)
{
   U8 i;
   for(i=0;i<MAX_EP_NB;i++)
   {
      if(it_pipe_str[i].enable==ENABLE) return TRUE;
   }
   return FALSE;
}

/**
  * @brief This function receives nb_data pointed with *buf with the pipe number specified
  *
  * The nb_data parameter is passed as a U16 pointer, thus the data pointed by this pointer
  * is updated with the final number of data byte received.
  *
  * @param pipe
  * @param nb_data
  * @param buf
  * @param handle call back function pointer
  *
  * @return status
  */
U8 host_get_data_interrupt(U8 pipe, U16 nb_data, U8 *buf,void(*handle)(U8 status, U16 nb_byte))
{
   Host_select_pipe(pipe);
   if(it_pipe_str[pipe].enable==ENABLE)
   {
      return HOST_FALSE;
   }
   else
   {
      if(is_any_interrupt_pipe_active()==FALSE)
      {
         g_sav_int_sof_enable=Is_host_sof_interrupt_enabled();
         Host_enable_sof_interrupt();
      }
      it_pipe_str[pipe].enable=ENABLE;
      it_pipe_str[pipe].nb_byte_to_process=nb_data;
      it_pipe_str[pipe].nb_byte_processed=0;
      it_pipe_str[pipe].ptr_buf=buf;
      it_pipe_str[pipe].handle=handle;
      it_pipe_str[pipe].timeout=0;
      it_pipe_str[pipe].nak_timeout=NAK_RECEIVE_TIMEOUT;

      private_sof_counter=0;           // Reset the counter in SOF detection sub-routine
      Host_reset_pipe(pipe);
      Host_enable_stall_interrupt();
      #if (NAK_TIMEOUT_ENABLE==ENABLE)
      Host_enable_nak_interrupt();
      #endif
      Host_enable_error_interrupt();
      Host_enable_receive_interrupt();
      Host_ack_stall();
      Host_ack_nak_received();

      Host_continuous_in_mode();
      Host_set_token_in();
      Host_unfreeze_pipe();
      return HOST_TRUE;
   }
}

/**
  * @brief This function send nb_data pointed with *buf with the pipe number specified
  *
  *
  * @param pipe
  * @param nb_data
  * @param buf
  * @param handle call back function pointer
  *
  * @return status
  */
U8 host_send_data_interrupt(U8 pipe, U16 nb_data, U8 *buf, void(*handle)(U8 status, U16 nb_byte))
{
   U8 i;
   U8 *ptr_buf=buf;

   Host_select_pipe(pipe);
   if(it_pipe_str[pipe].enable==ENABLE)
   {
      return HOST_FALSE;
   }
   else
   {
      if(is_any_interrupt_pipe_active()==FALSE)
      {
         g_sav_int_sof_enable=Is_host_sof_interrupt_enabled();
         Host_enable_sof_interrupt();
      }
      it_pipe_str[pipe].enable=ENABLE;
      it_pipe_str[pipe].nb_byte_to_process=nb_data;
      it_pipe_str[pipe].nb_byte_processed=0;
      it_pipe_str[pipe].ptr_buf=buf;
      it_pipe_str[pipe].handle=handle;
      it_pipe_str[pipe].timeout=0;
      it_pipe_str[pipe].nak_timeout=NAK_SEND_TIMEOUT;
      it_pipe_str[pipe].nb_byte_on_going=0;

      Host_reset_pipe(pipe);
      Host_unfreeze_pipe();
      // Prepare data to be sent
      i = Host_get_pipe_length();
      if ( i > nb_data)                // Pipe size> remaining data
      {
         i = nb_data;
         nb_data = 0;
      }
      else                             // Pipe size < remaining data
      {
         nb_data -= i;
      }
      it_pipe_str[pipe].nb_byte_on_going+=i;   // Update nb data processed
      while (i!=0)                    // Load Pipe buffer
      {  Host_write_byte(*ptr_buf++); i--;
      }
      private_sof_counter=0;          // Reset the counter in SOF detection sub-routine
      it_pipe_str[pipe].timeout=0;    // Refresh timeout counter
      Host_ack_out_sent();
      Host_ack_stall();
      Host_ack_nak_received();

      Host_enable_stall_interrupt();
      Host_enable_error_interrupt();
      #if (NAK_TIMEOUT_ENABLE==ENABLE)
      Host_enable_nak_interrupt();
      #endif
      Host_enable_transmit_interrupt();
      Host_send_out();                // Send the USB frame
      return HOST_TRUE;
   }
}

//! @brief USB pipe interrupt subroutine
//!
//! @param none
//!
//! @return none
#ifdef __GNUC__
 ISR(USB_COM_vect)
#else
#pragma vector = USB_Endpoint_Pipe_vect
__interrupt void usb_pipe_interrupt()
#endif
{
   U8 pipe_nb;
   U8 *ptr_buf;
   void  (*fct_handle)(U8 status,U16 nb_byte);
   U16 n;
   U8 i;
   U8 do_call_back=FALSE;

   pipe_nb_save = Host_get_selected_pipe();       // Important! Save here working pipe number
   pipe_nb=usb_get_nb_pipe_interrupt();  // work with the correct pipe number that generates the interrupt
   Host_select_pipe(pipe_nb);                        // Select this pipe
   fct_handle=*(it_pipe_str[pipe_nb].handle);

   // Now try to detect what event generate an interrupt...

   if (Is_host_pipe_error())             // Any error ?
   {
      it_pipe_str[pipe_nb].status = Host_error_status();
      it_pipe_str[pipe_nb].enable=DISABLE;
      Host_stop_pipe_interrupt(pipe_nb);
      Host_ack_all_errors();
      do_call_back=TRUE;
      goto usb_pipe_interrupt_end;
   }

   if (Is_host_stall())                  // Stall handshake received ?
   {
      it_pipe_str[pipe_nb].status=PIPE_STALL;
      it_pipe_str[pipe_nb].enable=DISABLE;
      Host_stop_pipe_interrupt(pipe_nb);
      do_call_back=TRUE;
      goto usb_pipe_interrupt_end;
   }

   #if (NAK_TIMEOUT_ENABLE==ENABLE)
   if (Is_host_nak_received())           // NAK ?
   {
      Host_ack_nak_received();
      // check if number of NAK timeout error occurs (not for interrupt type pipe)
      if((--it_pipe_str[pipe_nb].nak_timeout==0) && (Host_get_pipe_type()!=TYPE_INTERRUPT))
      {
         it_pipe_str[pipe_nb].status=PIPE_NAK_TIMEOUT;
         it_pipe_str[pipe_nb].enable=DISABLE;
         Host_stop_pipe_interrupt(pipe_nb);
         do_call_back=TRUE;
         goto usb_pipe_interrupt_end;
      }
   }
   #endif

   if (Is_host_in_received())            // Pipe IN reception ?
   {
      ptr_buf=it_pipe_str[pipe_nb].ptr_buf+it_pipe_str[pipe_nb].nb_byte_processed;       // Build pointer to data buffer
      n=it_pipe_str[pipe_nb].nb_byte_to_process-it_pipe_str[pipe_nb].nb_byte_processed;  // Remaining data bytes
      Host_freeze_pipe();
      if (Host_byte_counter()<=n)
      {
         if ((Host_byte_counter() < n)&&(Host_byte_counter()<Host_get_pipe_length())) //Received less than remaining, but less than pipe capacity
                                                                                      //TODO: error code
         {
            n=0;
         }
         else
         {
            n-=Host_byte_counter();
         }
         it_pipe_str[pipe_nb].nb_byte_processed+=Host_byte_counter();  // Update nb of byte received
         for (i=Host_byte_counter();i;i--)
         { *ptr_buf=Host_read_byte(); ptr_buf++;}
      }
      else  // more bytes received than expected
      {     // TODO error code management
         it_pipe_str[pipe_nb].nb_byte_processed+=n;
         for (i=n;i;i--)                  // Byte number limited to the initial request (limit tab over pb)
         { *ptr_buf=Host_read_byte(); ptr_buf++;}
         n=0;
      }
      Host_ack_in_received();
      if(n>0) //still something to process
      {
         Host_unfreeze_pipe();            // Request another IN transfer
         Host_send_in();
         private_sof_counter=0;           // Reset the counter in SOF detection sub-routine
         it_pipe_str[pipe_nb].timeout=0;  // Reset timeout
         it_pipe_str[pipe_nb].nak_timeout=NAK_RECEIVE_TIMEOUT;

      }
      else //end of transfer
      {
         it_pipe_str[pipe_nb].enable=DISABLE;
         it_pipe_str[pipe_nb].status=PIPE_GOOD;
         Host_stop_pipe_interrupt(pipe_nb);
         do_call_back=TRUE;
      }
   }

   if(Is_host_out_sent())                  // Pipe OUT sent ?
   {
      Host_ack_out_sent();
      it_pipe_str[pipe_nb].nb_byte_processed+=it_pipe_str[pipe_nb].nb_byte_on_going;
      it_pipe_str[pipe_nb].nb_byte_on_going=0;
      ptr_buf=it_pipe_str[pipe_nb].ptr_buf+it_pipe_str[pipe_nb].nb_byte_processed;       // Build pointer to data buffer
      n=it_pipe_str[pipe_nb].nb_byte_to_process-it_pipe_str[pipe_nb].nb_byte_processed;  // Remaining data bytes
      if(n>0)   // Still data to process...
      {
         Host_unfreeze_pipe();
        // Prepare data to be sent
         i = Host_get_pipe_length();
         if ( i > n)     // Pipe size> remaining data
         {
            i = n;
            n = 0;
         }
         else                // Pipe size < remaining data
         {  n -= i; }
         it_pipe_str[pipe_nb].nb_byte_on_going+=i;   // Update nb data processed
         while (i!=0)                     // Load Pipe buffer
         {
            Host_write_byte(*ptr_buf++); i--;
         }
         private_sof_counter=0;           // Reset the counter in SOF detection sub-routine
         it_pipe_str[pipe_nb].timeout=0;  // Refresh timeout counter
         it_pipe_str[pipe_nb].nak_timeout=NAK_SEND_TIMEOUT;
         Host_send_out();                 // Send the USB frame
      }
      else                                //n==0 Transfer is finished
      {
         it_pipe_str[pipe_nb].enable=DISABLE;    // Tranfer end
         it_pipe_str[pipe_nb].status=PIPE_GOOD;  // Status OK
         Host_stop_pipe_interrupt(pipe_nb);
         do_call_back=TRUE;
      }
   }

usb_pipe_interrupt_end:
   Host_select_pipe(pipe_nb_save);   // Restore pipe number !!!!
   if (is_any_interrupt_pipe_active()==FALSE)    // If no more transfer is armed
   {
      if (g_sav_int_sof_enable==FALSE)
      {
         Host_disable_sof_interrupt();
      }
   }
   if(do_call_back)      // Any callback functions to perform ?
   {
      fct_handle(it_pipe_str[pipe_nb].status,it_pipe_str[pipe_nb].nb_byte_processed);
   }
}
#endif


#endif // USB_HOST_FEATURE ENABLE

