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

#include "garmin.h"
#include "nmeagen.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define NMEA_BUF_SIZE       256
#define NMEA_LATLON_SIZE    16
#define NMEA_UTC_SIZE       16

double g_lastcourse = -1;

void nmea_getutc(D800_Pvt_Data_Type *pvt, char *utctime, char *utcdate) {
    double  tmp;
    int     daysdiff;

    /* UTC time of position fix */
    tmp = pvt->tow - pvt->leap_scnds;
    if (tmp < 0.0)
        tmp += 86400.0;
    daysdiff = 0;
    while (tmp > 86400.0) {
        tmp -= 86400.0;
        daysdiff++;
    }

    if (utctime) {
        int h, m, s;
        h = ((int)tmp) / 3600;
        m = ((int)tmp - h*3600) / 60;
        s = ((int)tmp - h*3600 - m*60);
        sprintf(utctime, "%02d%02d%02d", h, m, s);
    }

    if (utcdate) {
        /* Garmin format: number of days since December 31, 1989 */
        unsigned long jd = pvt->wn_days + daysdiff + 2447892;
        unsigned long w, x, a, b, c, d, e, f;
        unsigned long day, month, year;

        w = (unsigned long)((jd - 1867216.25)/36524.25);
        x = w/4;
        a = jd + 1 + w - x;
        b = a + 1524;
        c = (unsigned long)((b - 122.1)/365.25);
        d = (unsigned long)(365.25 * c);
        e = (unsigned long)((b-d)/30.6001);
        f = (unsigned long)(30.6001 * e);

        day = b - d - f;
        month = e - 1;
        if (month > 12)
            month -= 12;
        year = c - 4716;
        if (month == 1 || month == 2)
            year++;

        year -= 2000;

        sprintf(utcdate, "%02ld%02ld%02ld", day, month, year);
    }
}

void nmea_fmtlat(double lat, char *latstr) {
    double  latdeg, tmp;
    latdeg = rad2deg(fabs(lat));
    tmp = floor(latdeg);
    sprintf(latstr, "%02d%07.4f,%c", (int)tmp, (latdeg - tmp) * 60,
        (lat >= 0) ? 'N' : 'S');
}

void nmea_fmtlon(double lon, char *lonstr) {
    double  londeg, tmp;
    londeg = rad2deg(fabs(lon));
    tmp = floor(londeg);
    sprintf(lonstr, "%03d%07.4f,%c", (int)tmp, (londeg - tmp) * 60,
        (lon >= 0) ? 'E' : 'W');
}

/*
    nmea_gprmc()
    NMEA Recommended Minimum Specific GPS/TRANSIT Data (RMC)
    
    Caveats:
        - Speed and course over ground are calculated from
            the north/east velocity and may not be accurate
        - Magnetic variation not available
*/
int nmea_gprmc(D800_Pvt_Data_Type *pvt, char *nmeastc) {
    
    char    buf[NMEA_BUF_SIZE];
    char    slat[NMEA_LATLON_SIZE], slon[NMEA_LATLON_SIZE];
    char    utctime[NMEA_UTC_SIZE], utcdate[NMEA_UTC_SIZE];
    double  speed, course;
    unsigned char   cksum;

    nmea_getutc(pvt, utctime, utcdate);

    /* latitude */
    nmea_fmtlat(pvt->lat, slat);

    /* longitude */
    nmea_fmtlon(pvt->lon, slon);
    
    /* speed over ground */
    speed = sqrt(pvt->east*pvt->east + pvt->north*pvt->north) * 3.6 / KNOTS_TO_KMH;

    /* course */
    if (speed < 1.0) {
        if (g_lastcourse >= 0)
            course = g_lastcourse;
        else
            course = 0; /* too low to determine course */
    } else {
        course = atan2(pvt->east, pvt->north);
        if (course < 0)
            course += 2*G_PI;
        course = rad2deg(course);
        g_lastcourse = course;  /* remember for later */
    }

    sprintf(buf, "GPRMC,%s,%c,%s,%s,%05.1f,%05.1f,%s,,", utctime,
        (pvt->fix >= 2 && pvt->fix <= 5) ? 'A' : 'V',
        slat, slon, speed, course, utcdate);

    cksum = nmea_cksum(buf);

    sprintf(nmeastc, "$%s*%02X\r\n", buf, cksum);

    return 0;
}

/*
    nmea_gpgll()
    NMEA Geographic Position (GLL)
*/
int nmea_gpgll(D800_Pvt_Data_Type *pvt, char *nmeastc) {
    
    char    buf[NMEA_BUF_SIZE];
    char    slat[NMEA_LATLON_SIZE], slon[NMEA_LATLON_SIZE];
    char    utctime[NMEA_UTC_SIZE];
    unsigned char   cksum;

    nmea_getutc(pvt, utctime, NULL);

    /* latitude */
    nmea_fmtlat(pvt->lat, slat);

    /* longitude */
    nmea_fmtlon(pvt->lon, slon);

    sprintf(buf, "GPGLL,%s,%s,%s,%c", slat, slon, utctime,
        (pvt->fix >= 2 && pvt->fix <= 5) ? 'A' : 'V');

    cksum = nmea_cksum(buf);

    sprintf(nmeastc, "$%s*%02X\r\n", buf, cksum);

    return 0;
}

unsigned char nmea_cksum(char *str) {

    int i;
    unsigned char cksum = 0;

    for (i = 0; str[i]; i++) {
        cksum ^= (unsigned char)str[i];
    }

    return cksum;
}
