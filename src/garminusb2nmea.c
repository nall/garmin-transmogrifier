/*
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
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include "garmin.h"
#include "nmeagen.h"

#define GARMIN_HEADER_SIZE  12
#define GARMIN_MAX_PKTSIZE  512
#define PKTBUF_SIZE         4097

#pragma pack(push, 1)
typedef struct {
    u_int8_t    mPacketType;
    u_int8_t    mReserved1;
    u_int16_t   mReserved2;
    u_int16_t   mPacketId;
    u_int16_t   mReserved3;
    u_int32_t   mDataSize;
    u_int8_t    mData[1];
} Packet_t;
#pragma pack(pop)

int gps_fd;
char pktbuf[PKTBUF_SIZE];
int pktbuf_head = 0, pktbuf_tail = 0;

D800_Pvt_Data_Type  lastpvt;
cpo_sat_data        lastsatdata[12];
int satdata_valid = 0;

int pktbuf_size() {
    return (PKTBUF_SIZE - pktbuf_head + pktbuf_tail) % PKTBUF_SIZE;
}

int pktbuf_peek(char *buf, int n) {
    int i;
    int mypktbuf_head = pktbuf_head;
    for (i = 0; (i < n) && (mypktbuf_head != pktbuf_tail); i++) {
        buf[i] = pktbuf[mypktbuf_head];
        mypktbuf_head++;
        if (mypktbuf_head == PKTBUF_SIZE)
            mypktbuf_head = 0;
    }
    return i;
}

int pktbuf_deq(char *buf, int n) {
    int i;
    for (i = 0; (i < n) && (pktbuf_head != pktbuf_tail); i++) {
        buf[i] = pktbuf[pktbuf_head];
        pktbuf_head++;
        if (pktbuf_head == PKTBUF_SIZE)
            pktbuf_head = 0;
    }
    return i;
}

int pktbuf_enq(char *buf, int n) {
    
    int i;
    
    if (pktbuf_size() + n >= PKTBUF_SIZE)
        return 0;
    
    for (i = 0; i < n; i++) {
        pktbuf[pktbuf_tail] = buf[i];
        pktbuf_tail++;
        if (pktbuf_tail == PKTBUF_SIZE)
            pktbuf_tail = 0;
    }
    
    return i;
}

int sendpacket(Packet_t *pack) {
    int nwr;
    nwr = write(gps_fd, pack, GARMIN_HEADER_SIZE + pack->mDataSize);
    if (nwr == -1) {
        perror("GPS write error");
        return 1;
    }
    return 0;
}

Packet_t* recvpacket(void) {
    Packet_t *pkt;
    char tmp[64];
    int nr;
    
    pkt = (Packet_t*)malloc(GARMIN_MAX_PKTSIZE);
    if (pkt == NULL) {
        perror("malloc failed");
        return NULL;
    }

chkbuf: 
    /* complete packet in buffer? */
    if (pktbuf_size() >= GARMIN_HEADER_SIZE) {
        Packet_t bufpkt;
        pktbuf_peek((char*)&bufpkt, GARMIN_HEADER_SIZE);
        int pktlen = GARMIN_HEADER_SIZE + bufpkt.mDataSize;
        if (pktbuf_size() >= pktlen) {
            pktbuf_deq((char*)pkt, pktlen);
            return pkt;
        }
    }
    
    /* not enough data - read some */
    nr = read(gps_fd, tmp, 64);
    if (nr == -1) {
        perror("GPS read error");
        free(pkt);
        return NULL;
    }
    if (pktbuf_enq(tmp, nr) == 0)
        fprintf(stderr, "Input buffer full!");
    
    goto chkbuf;
}

/*
    garmin_pvton()
    turn on position records
  
    receiver measurement records could also be enabled with 
    command 110 (instead of 49), but we don't need them at present
*/
void garmin_pvton(void) {
    Packet_t *pvtpack = (Packet_t*)malloc(14);

    pvtpack->mPacketType = 20;
    pvtpack->mReserved1 = 0;
    pvtpack->mReserved2 = 0;
    pvtpack->mPacketId = 10;
    pvtpack->mReserved3 = 0;
    pvtpack->mDataSize = 2;
    pvtpack->mData[0] = 49;
    pvtpack->mData[1] = 0;
    
    sendpacket(pvtpack);
}

int main(int argc, char *argv[]) {
    
    char nmeabuf[256];
    u_int32_t privcmd[4];
    FILE *nmeaout;
    struct termios termio;

    if (argc < 2) {
        printf("Usage: %s gpsdev [nmeaoutdev]\n", argv[0]);
        return 1;
    }

    gps_fd = open(argv[1], O_RDWR);
    if (gps_fd == -1) {
        perror("Cannot open GPS device");
        return 1;
    }
    
    tcgetattr(gps_fd, &termio);
    cfmakeraw(&termio);
    tcsetattr(gps_fd, TCIOFLUSH, &termio);
    
    privcmd[0] = 0x01106E4B;
    privcmd[1] = 2;
    privcmd[2] = 4;
    privcmd[3] = 0;
    write(gps_fd, privcmd, 16);
    
    if (argc == 3) {
        nmeaout = fopen(argv[2], "w");
        if (!nmeaout) {
            perror("Cannot open output file");
            return 1;
        }
    } else
        nmeaout = stdout;

    garmin_pvton();
    
    while (1) {
        Packet_t *pkt = recvpacket();
        printf("Packet ID: %d\n", pkt->mPacketId);
        if (pkt->mPacketId == Pid_Pvt_Data) {
            memcpy(&lastpvt, pkt->mData, sizeof(lastpvt));
            
            if (nmea_gpgga(&lastpvt, satdata_valid ? lastsatdata : NULL, nmeabuf) == 0)
                fprintf(nmeaout, "%s", nmeabuf);
            if (nmea_gprmc(&lastpvt, nmeabuf) == 0)
                fprintf(nmeaout, "%s", nmeabuf);
            if (nmea_gpgll(&lastpvt, nmeabuf) == 0)
                fprintf(nmeaout, "%s", nmeabuf);
            if (nmea_gpgsa(&lastpvt, satdata_valid ? lastsatdata : NULL, nmeabuf) == 0)
                fprintf(nmeaout, "%s", nmeabuf);
            
            fflush(nmeaout);
            
        } else if (pkt->mPacketId == Pid_SatData_Record) {
            memcpy(lastsatdata, pkt->mData, sizeof(lastsatdata));
            satdata_valid = 1;
            if (nmea_gpgsv(lastsatdata, nmeabuf) == 0) {
                fprintf(nmeaout, "%s", nmeabuf);
                fflush(nmeaout);
            }
        }
        free(pkt);
    }
    
    close(gps_fd);
}
