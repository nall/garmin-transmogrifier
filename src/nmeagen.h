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
#ifndef NMEAGEN_H
#define NMEAGEN_H
#include "garmin.h"

#define KNOTS_TO_KMH    1.852
#define G_PI            3.14159265358979324
#define rad2deg(x)      ((x) * 180.0 / G_PI)

void nmea_getutc(D800_Pvt_Data_Type *pvt, char *utctime, char *utcdate);
void nmea_fmtlat(double lat, char *latstr);
void nmea_fmtlon(double lon, char *lonstr);
int nmea_gprmc(D800_Pvt_Data_Type *pvt, char *nmeastc);
int nmea_gpgll(D800_Pvt_Data_Type *pvt, char *nmeastc);
unsigned char nmea_cksum(char *str);

#endif
