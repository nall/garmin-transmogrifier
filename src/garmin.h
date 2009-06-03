/*
    Garmin protocol to NMEA 0183 converter
    Copyright (C) 2004 Manuel Kasper <mk@neon1.net>.
    All rights reserved.

    Input:
        - D800_Pvt_Data_Type (PID 51)
        - satellite data record (PID 114)

    Available output sentences:
        GPGGA, GPRMC, GPGLL, GPGSA, GPGSV

    Known caveats:
        - DOP (Dilution of Precision) information not available
          (Garmin protocol includes EPE only)
        - DGPS information in GPGGA sentence not returned
        - speed and course over ground are calculated from the
          north/east velocity and may not be accurate
        - magnetic variation information not available
        - Garmin 16-bit SNR scale unknown

    ---------------------------------------------------------------------------
    
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
#ifndef GARMIN_H
#define GARMIN_H

#include <stdint.h>
typedef uint8_t u_int8_t;
typedef uint16_t u_int16_t;
typedef uint32_t u_int32_t;

#define GARMIN_HEADER_SIZE  12
#define GARMIN_MAX_PKTSIZE  512

typedef struct {
    u_int8_t    mPacketType;
    u_int8_t    mReserved1;
    u_int16_t   mReserved2;
    u_int16_t   mPacketId;
    u_int16_t   mReserved3;
    u_int32_t   mDataSize;
    u_int8_t    mData[1];
} Packet_t;

typedef struct {
    u_int8_t    mTag;
    u_int16_t   mData;
} Protocol_Data_Type;

typedef struct {
    float   alt;
    float   epe;
    float   eph;
    float   epv;
    int16_t fix;
    double  tow;
    double  lat;
    double  lon;
    float   east;
    float   north;
    float   up;
    float   msl_hght;
    int16_t leap_scnds;
    int32_t wn_days;
} D800_Pvt_Data_Type;

enum {
    Pid_Command_Data = 10,
    Pid_Xfer_Cmplt = 12,
    Pid_Date_Time_Data = 14,
    Pid_Position_Data = 17,
    Pid_Prx_Wpt_Data = 19,
    Pid_Records = 27,
    Pid_Rte_Hdr = 29,
    Pid_Rte_Wpt_Data = 30,
    Pid_Almanac_Data = 31,
    Pid_Trk_Data = 34,
    Pid_Wpt_Data = 35,
    Pid_Pvt_Data = 51,
    Pid_RMR_Data = 52,
    Pid_Rte_Link_Data = 98,
    Pid_Trk_Hdr = 99,
    Pid_SatData_Record = 114,
    Pid_FlightBook_Record = 134,
    Pid_Lap = 149
};

enum {
    Cmnd_Abort_Transfer = 0, /* abort current transfer */
    Cmnd_Transfer_Alm = 1, /* transfer almanac */
    Cmnd_Transfer_Posn = 2, /* transfer position */
    Cmnd_Transfer_Prx = 3, /* transfer proximity waypoints */
    Cmnd_Transfer_Rte = 4, /* transfer routes */
    Cmnd_Transfer_Time = 5, /* transfer time */
    Cmnd_Transfer_Trk = 6, /* transfer track log */
    Cmnd_Transfer_Wpt = 7, /* transfer waypoints */
    Cmnd_Turn_Off_Pwr = 8, /* turn off power */
    Cmnd_Start_Pvt_Data = 49, /* start transmitting PVT data */
    Cmnd_Stop_Pvt_Data = 50, /* stop transmitting PVT data */
    Cmnd_FlightBook_Transfer = 92, /* start transferring flight records */
    Cmnd_Start_RMR = 110, /* start transmitting Receiver Measurement Records */
    Cmnd_Stop_RMR = 111, /* start transmitting Receiver Measurement Records */
    Cmnd_Transfer_Laps = 117 /* transfer laps */
};

#endif
