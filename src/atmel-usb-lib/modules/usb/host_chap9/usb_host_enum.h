/*This file has been prepared for Doxygen automatic documentation generation.*/
//! \file *********************************************************************
//!
//! \brief USB host enumeration process header file
//!
//!  This file contains the USB pipe 0 management routines corresponding to
//!  the standard enumeration process (refer to chapter 9 of the USB
//!  specification.
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

#ifndef _USB_HOST_ENUM_H_
#define _USB_HOST_ENUM_H_

//_____ I N C L U D E S ____________________________________________________


#include "modules/usb/usb_task.h"

//_____ M A C R O S ________________________________________________________

#ifndef SIZEOF_DATA_STAGE
   #error SIZEOF_DATA_STAGE should be defined in conf_usb.h
#endif

#if (SIZEOF_DATA_STAGE<0xFF)     //! Optimize descriptor offset index according to data_stage[] size
   #define T_DESC_OFFSET   U8    //! U8 is enought and faster
#else
   #define T_DESC_OFFSET   U16   //! U16 required !
#endif

#ifndef MAX_EP_PER_INTERFACE
   #define MAX_EP_PER_INTERFACE 4
#endif


//_____ S T A N D A R D    D E F I N I T I O N S ___________________________

//! @defgroup host_enum USB host enumeration functions
//! @{

//! Usb Setup Data
typedef struct
{
   U8      bmRequestType;        //!< Characteristics of the request
   U8      bRequest;             //!< Specific request
   U16     wValue;               //!< field that varies according to request
   U16     wIndex;               //!< field that varies according to request
   U16     wLength;              //!< Number of bytes to transfer if Data
   U8      uncomplete_read;      //!< 1 = only one read
}  S_usb_setup_data;

typedef struct
{
   U8 ep_addr;
   U8 pipe_number;
   U8 ep_size;
   U8 ep_type;
} S_usb_endpoint;

typedef struct
{
   U8  interface_nb;
   U8  altset_nb;
   U16 class;
   U16 subclass;
   U16 protocol;
   U8  nb_ep;
   S_usb_endpoint ep[MAX_EP_PER_INTERFACE];
} S_usb_interface;

typedef struct
{
   U8 device_address;
   U8 ep_ctrl_size;
   U8 hub_port_nb;
   U8 parent_hub_number;
   U8 nb_interface;
   U16 pid;
   U16 vid;
   U8 bmattributes;
   U8 maxpower;
   S_usb_interface interface[MAX_INTERFACE_FOR_DEVICE];
} S_usb_device;

typedef struct
{
   U8 nb_device;
   S_usb_device device[MAX_DEVICE_IN_USB_TREE];
} S_usb_tree;


#if (USB_HUB_SUPPORT==ENABLE)
extern U8 user_periodic_pipe;
#endif
extern  S_usb_tree usb_tree;
extern  S_usb_setup_data usb_request;
extern  U8 data_stage[SIZEOF_DATA_STAGE];
extern  U8 device_status;

#define REQUEST_TYPE_POS         0
#define REQUEST_POS              1
#define VALUE_HIGH_POS           2
#define VALUE_LOW_POS            3
#define INDEX_HIGH_POS           4
#define INDEX_LOW_POS            5
#define LENGTH_HIGH_POS          6
#define LENGTH_LOW_POS           7
#define UNCOMPLETE_READ_POS      8
#define DATA_ADDR_HIGH_POS       9
#define DATA_ADDR_LOW_POS       10

#define CONTROL_GOOD             0
#define CONTROL_DATA_TOGGLE   0x01
#define CONTROL_DATA_PID      0x02
#define CONTROL_PID           0x04
#define CONTROL_TIMEOUT       0x08
#define CONTROL_CRC16         0x10
#define CONTROL_STALL         0x20
#define CONTROL_NO_DEVICE     0x40


//!< Set of defines for offset in data stage
#define OFFSET_FIELD_MAXPACKETSIZE     7
#define OFFSET_FIELD_MSB_VID           9
#define OFFSET_FIELD_LSB_VID           8
#define OFFSET_FIELD_MSB_PID           11
#define OFFSET_FIELD_LSB_PID           10

#define OFFSET_DESCRIPTOR_LENGHT       0
#define OFFSET_FIELD_DESCRIPTOR_TYPE   1
#define OFFSET_FIELD_TOTAL_LENGTH      2
#define OFFSET_FIELD_BMATTRIBUTES      7
#define OFFSET_FIELD_MAXPOWER          8



//! OFFSET for INTERFACE DESCRIPTORS
#define OFFSET_FIELD_NB_INTERFACE      4
#define OFFSET_FIELD_CLASS             5
#define OFFSET_FIELD_SUB_CLASS         6
#define OFFSET_FIELD_PROTOCOL          7

#define OFFSET_FIELD_INTERFACE_NB      2
#define OFFSET_FIELD_ALT               3
#define OFFSET_FIELS_NB_OF_EP          4

#define OFFSET_FIELD_EP_ADDR           2
#define OFFSET_FIELD_EP_TYPE           3
#define OFFSET_FIELD_EP_SIZE           4
#define OFFSET_FIELD_EP_INTERVAL       6

#define HOST_FALSE                     0
#define HOST_TRUE                      1

U8 host_send_control(U8*);

/**
 * host_clear_endpoint_feature
 *
 * @brief this function send a clear endpoint request
 *
 * @param U8 ep (the target endpoint nb)
 *
 * @return status
 */
#define host_clear_endpoint_feature(ep)   (usb_request.bmRequestType = USB_SETUP_SET_STAND_ENDPOINT,\
                                           usb_request.bRequest      = SETUP_CLEAR_FEATURE,\
                                           usb_request.wValue        = FEATURE_ENDPOINT_HALT << 8,\
                                           usb_request.wIndex        = ep,\
                                           usb_request.wLength       = 0,\
                                           usb_request.uncomplete_read = FALSE,\
                                           host_send_control(data_stage))
/**
 * host_get_configuration
 *
 * @brief this function send a get configuration request
 *
 * @param none
 *
 * @return status
 */
#define host_get_configuration()          (usb_request.bmRequestType = USB_SETUP_GET_STAND_DEVICE,\
                                           usb_request.bRequest      = SETUP_GET_CONFIGURATION,\
                                           usb_request.wValue        = 0,\
                                           usb_request.wIndex        = 0,\
                                           usb_request.wLength       = 1,\
                                           usb_request.uncomplete_read = FALSE,\
                                           host_send_control(data_stage))
/**
 * host_set_configuration
 *
 * @brief this function send a set configuration request
 *
 * @param U8 configuration numer to activate
 *
 * @return status
 */
#define host_set_configuration(cfg_nb)    (usb_request.bmRequestType = USB_SETUP_SET_STAND_DEVICE,\
                                           usb_request.bRequest      = SETUP_SET_CONFIGURATION,\
                                           usb_request.wValue        = cfg_nb,\
                                           usb_request.wIndex        = 0,\
                                           usb_request.wLength       = 0,\
                                           usb_request.uncomplete_read = FALSE,\
                                           host_send_control(data_stage))
/**
 * host_set_interface
 *
 * @brief this function send a set interface request
 * to specify a specific alt setting for an interface
 *
 * @param U8 interface_nb (the interface)
 *        U8 alt_setting (the alternate setting to activate)
 *
 * @return status
 */
#define host_set_interface(interface_nb,alt_setting)        (usb_request.bmRequestType = USB_SETUP_SET_STAND_DEVICE,\
                                           usb_request.bRequest      = USB_SETUP_SET_STAND_INTERFACE,\
                                           usb_request.wValue        = alt_setting,\
                                           usb_request.wIndex        = interface_nb,\
                                           usb_request.wLength       = 0,\
                                           usb_request.uncomplete_read = FALSE,\
                                           host_send_control(data_stage))

/**
 * host_get_device_descriptor_uncomplete
 *
 * @brief this function send a get device desriptor request.
 * The descriptor table received is stored in data_stage array.
 * The received descriptors is limited to the control pipe lenght
 *
 *
 * @param none
 *
 *
 * @return status
 */
#define host_get_device_descriptor_uncomplete()  (usb_request.bmRequestType = USB_SETUP_GET_STAND_DEVICE,\
                                           usb_request.bRequest      = SETUP_GET_DESCRIPTOR,\
                                           usb_request.wValue        = DESCRIPTOR_DEVICE << 8,\
                                           usb_request.wIndex        = 0,\
                                           usb_request.wLength       = 64,\
                                           usb_request.uncomplete_read = TRUE,\
                                           host_send_control(data_stage))

/**
 * host_get_device_descriptor
 *
 * @brief this function send a get device desriptor request.
 * The descriptor table received is stored in data_stage array.
 *
 *
 * @param none
 *
 *
 * @return status
 */
#define host_get_device_descriptor()      (usb_request.bmRequestType = USB_SETUP_GET_STAND_DEVICE,\
                                           usb_request.bRequest      = SETUP_GET_DESCRIPTOR,\
                                           usb_request.wValue        = DESCRIPTOR_DEVICE << 8,\
                                           usb_request.wIndex        = 0,\
                                           usb_request.wLength       = 18,\
                                           usb_request.uncomplete_read = FALSE,\
                                           host_send_control(data_stage))
/**
 * host_get_configuration_descriptor
 *
 * @brief this function send a get device configuration request.
 * The configuration descriptor table received is stored in data_stage array.
 *
 *
 * @param none
 *
 *
 * @return status
 */
#define host_get_configuration_descriptor()  (usb_request.bmRequestType = USB_SETUP_GET_STAND_DEVICE,\
                                           usb_request.bRequest      = SETUP_GET_DESCRIPTOR,\
                                           usb_request.wValue        = DESCRIPTOR_CONFIGURATION << 8,\
                                           usb_request.wIndex        = 0,\
                                           usb_request.wLength       = 255,\
                                           usb_request.uncomplete_read = FALSE,\
                                           host_send_control(data_stage))

#define host_get_descriptor_uncomplete()  (usb_request.bmRequestType = USB_SETUP_GET_STAND_DEVICE,\
                                           usb_request.bRequest      = SETUP_GET_DESCRIPTOR,\
                                           usb_request.wValue        = 0,\
                                           usb_request.wIndex        = 0,\
                                           usb_request.wLength       = 64,\
                                           usb_request.uncomplete_read = FALSE,\
                                           host_send_control(data_stage))
/**
 * host_set_address
 *
 * @brief this function send a set address request.
 *
 *
 * @param U8 address (the addr attributed to the device)
 *
 *
 * @return status
 */
#define host_set_address(addr)            (usb_request.bmRequestType = USB_SETUP_SET_STAND_DEVICE,\
                                           usb_request.bRequest      = SETUP_SET_ADDRESS,\
                                           usb_request.wValue        = (U16)addr,\
                                           usb_request.wIndex        = 0,\
                                           usb_request.wLength       = 0,\
                                           usb_request.uncomplete_read = FALSE,\
                                           host_send_control(data_stage))

/**
 * host_set_feature_remote_wakeup
 *
 * @brief this function send a set feature device remote wakeup
 *
 * @param none
 *
 * @return status
 */
#define host_set_feature_remote_wakeup()   (usb_request.bmRequestType = USB_SETUP_SET_STAND_DEVICE,\
                                           usb_request.bRequest      = SETUP_SET_FEATURE,\
                                           usb_request.wValue        = 1,\
                                           usb_request.wIndex        = 1,\
                                           usb_request.wLength       = 0,\
                                           usb_request.uncomplete_read = FALSE,\
                                           host_send_control(data_stage))

/**
 * host_ms_get_max_lun
 *
 * @brief this function send the mass storage specific request "get max lun"
 *
 *
 * @param none
 *
 *
 * @return status
 */
#define host_ms_get_max_lun()             (usb_request.bmRequestType = USB_SETUP_GET_CLASS_INTER,\
                                           usb_request.bRequest      = SETUP_MASS_STORAGE_GET_MAX_LUN,\
                                           usb_request.wValue        = 0,\
                                           usb_request.wIndex        = 0,\
                                           usb_request.wLength       = 1,\
                                           usb_request.uncomplete_read = FALSE,\
                                           host_send_control(data_stage))

/**
 * Get_VID
 *
 * @brief this function returns the VID of the device connected
 *
 * @param none
 *
 *
 * @return U16 (VID value)
 */
#define Get_VID()      (usb_tree.device[selected_device].vid)

/**
 * Get_PID
 *
 * @brief this function returns the PID of the device connected
 *
 * @param none
 *
 *
 * @return U16 (PID value)
 */
#define Get_PID()      (usb_tree.device[selected_device].pid)

/**
 * Get_ep0_size
 *
 * @brief this function returns the size of the control endpoint for the selected device
 *
 * @param none
 *
 *
 * @return U8 (EP0 size)
 */
#define Get_ep0_size()      (usb_tree.device[selected_device].ep_ctrl_size)

/**
 * Get_maxpower
 *
 * @brief this function returns the maximum power consumption ot the connected device (unit is 2mA)
 *
 * @param none
 *
 *
 * @return U8 (maxpower value)
 */
#define Get_maxpower()      (usb_tree.device[selected_device].maxpower)

/**
 * @brief this function returns the USB class associated to the specified interface
 *
 * @param U8 s_interface: the supported interface number
 *
 * @return U16 (CLASS code)
 */
#define Get_class(s_interface)      (usb_tree.device[selected_device].interface[s_interface].class)

/**
 * @brief this function returns the USB subclass associated to the specified interface
 *
 * @param U8 s_interface: the supported interface number
 *
 * @return U16 (SUBCLASS code)
 */
#define Get_subclass(s_interface)      (usb_tree.device[selected_device].interface[s_interface].subclass)

/**
 * @brief this function returns the USB protocol associated to the specified interface
 *
 * @param U8 s_interface: the supported interface number
 *
 * @return U16 (protocol code)
 */
#define Get_protocol(s_interface)      (usb_tree.device[selected_device].interface[s_interface].protocol)

/**
 * @brief this function returns endpoint address associated to the specified interface and
 * endpoint number in this interface.
 *
 * @param U8 s_interface: the supported interface number
 * @param U8 n_ep: the endpoint number in this interface
 *
 * @return U8 (endpoint address)
 */
#define Get_ep_addr(s_interface,n_ep)      (usb_tree.device[selected_device].interface[s_interface].ep[n_ep].ep_addr)

/**
 * @brief this function returns number of endpoints associated to
 * a supported interface.
 *
 * @param U8 s_interface: the supported interface number
 *
 * @return U8 (number of enpoints)
 */
#define Get_nb_ep(s_interface)      (usb_tree.device[selected_device].interface[s_interface].nb_ep)

/**
 * @brief this function returns number of the alternate setting field associated to
 * a supported interface.
 *
 * @param U8 s_interface: the supported interface number
 *
 * @return U8 (number of alt setting value)
 */
#define Get_alts_s(s_interface)      (usb_tree.device[selected_device].interface[s_interface].altset_nb)

/**
 * @brief this function returns number of the interface number associated to
 * a supported interface.
 *
 * @param U8 s_interface: the supported interface number
 *
 * @return U8 (number of the interface)
 */
#define Get_interface_number(s_interface)      (usb_tree.device[selected_device].interface[s_interface].interface_nb)

/**
 * @brief this function returns the number of interface supported in the device connected
 *
 * @param none
 *
 * @return U8 : The number of interface
 */
#define Get_nb_supported_interface()      (usb_tree.device[selected_device].nb_interface)

/**
 * @brief this function returns true if the device connected is self powered
 *
 * @param none
 *
 * @return U8 : The number of interface
 */
#define Is_device_self_powered()      ((usb_tree.device[selected_device].bmattributes & USB_CONFIG_ATTRIBUTES_SELFPOWERED) ? TRUE : FALSE)

/**
 * @brief this function returns the number of devices connected in the USB tree
 *
 * @param none
 *
 * @return U8 : The number of device
 */
#define Get_nb_device()      (usb_tree.nb_device)

/**
 * @brief this function returns true if the device supports remote wake_up
 *
 * @param none
 *
 * @return U8 : The number of interface
 */
#define Is_device_supports_remote_wakeup()  ((usb_tree.device[selected_device].bmattributes & USB_CONFIG_ATTRIBUTES_REMOTEWAKEUP) ? TRUE : FALSE)

#if (USB_HUB_SUPPORT==ENABLE && USER_PERIODIC_PIPE==ENABLE)
   #define Host_select_device(i)   (host_select_device(i))
#else
   #define Host_select_device(i)   (selected_device=i, Host_configure_address(usb_tree.device[i].device_address))
#endif

#define Host_get_nb_device()    ((U16)(usb_tree.nb_device))


extern U8 selected_device;
extern U8 ctrl_pipe_size;

U8 host_check_VID_PID(void);
U8 host_check_class  (void);
U8 host_auto_configure_endpoint();
T_DESC_OFFSET get_interface_descriptor_offset(U8 interface, U8 alt);
U8 host_get_hwd_pipe_nb(U8 ep_addr);
void init_usb_tree(void);
void remove_device_entry(U8 device_index);
void host_select_device(U8 i);

#if (USB_HUB_SUPPORT==ENABLE && USER_PERIODIC_PIPE==ENABLE)
void freeze_user_periodic_pipe(void);
void unfreeze_user_periodic_pipe(void);
#endif

//! @}

#endif  // _USB_HOST_ENUM_H_

