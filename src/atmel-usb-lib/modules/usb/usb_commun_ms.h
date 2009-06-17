/*This file has been prepared for Doxygen automatic documentation generation.*/
//! \file *********************************************************************
//!
//! \brief This file contains the usb MassStorage definition constant parameters
//!
//! - Compiler:           IAR EWAVR and GNU GCC for AVR
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

#ifndef _USB_COMMUN_MS_H_
#define _USB_COMMUN_MS_H_
                             
//! \name Global Class, SubClass & Protocol constants for Mass Storage
//! @{
#define  MS_CLASS                            0x08  //!< Mass Storage Class
#define  MS_SUB_CLASS2                       0x02  //!< ATAPI Subclass
#define  MS_SUB_CLASS5                       0x05  //!< SCSI transparent Command Set Class 5
#define  MS_SUB_CLASS6                       0x06  //!< SCSI transparent Command Set Class 6
#define  MS_PROTOCOL                         0x50  //!< Bulk-Only Transport
//! @}

//! \name Specific setup requests from Mass Storage
//! @{
#define  SETUP_MASS_STORAGE_GET_MAX_LUN      0xFE
#define  SETUP_MASS_STORAGE_RESET            0xFF
//! @}

#endif   // _USB_COMMUN_MS_H_

