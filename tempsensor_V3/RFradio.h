//file: RFradio.h
/*---------------------------------------------------------------------------------

   RFM95 LoRa RF radio declarations for tempsensor/temprcvr

 ----------------------------------------------------------------------------------
   Copyright (C) 2018, Len Shustek
   The MIT License (MIT): Permission is hereby granted, free of charge, to any
   person obtaining a copy of this software and associated documentation files
   (the "Software"), to deal in the Software without restriction, including
   without limitation the rights to use, copy, modify, merge, publish, distribute,
   sublicense, and/or sell copies of the Software, and to permit persons to whom
   the Software is furnished to do so, subject to the following conditions:
   The above copyright notice and this permission notice shall be
   included in all copies or substantial portions of the Software.
   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
   FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
   COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
   IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
   CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
   ------------------------------------------------------------------------------*/

struct datetime_t {  // this matches Teensy's time_t, coincidentally
   byte sec /*0..59*/, min /*0.59*/, hour /*0-23*/,
        wday /*1..7*/, date /*1..31*/, month /*1..12*/, year /*0..99 from 1970*/; };

enum ptype_t {PT_BAD, PT_UNUSED, PT_TEMP, PT_RESP }; // packet types

struct pkt_temp_t {  // the temperature reporting packet
   byte addr;         // node address, 0...3, mapped to "zones" 1..4
   byte type;         // this packet type, PT_TEMP
   byte temperature;  // temperature in degrees F, 1 to 120
   byte humidity;     // humidity in percent, 0..100
   uint32_t sequence; // packet sequence number (4-byte aligned!)
   #define RF_MAX_SEQUENCE (uint32_t)0xffffffffL
   byte battery;      // battery voltage*10
   byte sw_version;   // software version number x 10
};
#define RF_TEMP_PKT_SIZE 10 // on Teensy, structures are rounded up to a multiple of 4!

struct pkt_resp_t {  // the response packet
   byte addr;         // node address, 0...3, mapped to "zones" 1..4
   byte type;         // this packet type, TP_RESP
   struct datetime_t datetime; // 7 bytes of date and time
};
#define RF_RESP_PKT_SIZE 9 // on Teensy, structures are rounded up to a multiple of 4!

enum RFERR  // the various RF radio transient packet errors
{RF_OK, RF_RXTIMEOUT, RF_BADHDR, RF_TOOSMALL, RF_TOOBIG, RF_BADCRC, RF_NOPKT, RF_UNKNOWN };
// See the corresponding list of strings in RFradio.cpp!
extern const char* const RF_errors[] PROGMEM;

enum FATAL_ERRORS // the various merged fatal errors
// this is for: tempsensor, temprcvr, temprcvr_tester, RFradio.cpp, and display.cpp
{FATAL_NONE, 
FATAL_NO_RADIO, FATAL_BAD_GOTPKT, FATAL_BAD_GETPKT, // 1-3 for RFradio
FATAL_MSG_BIG, FATAL_BEEPTIMER, // 4-5 for temprcvr 
FATAL_NO_DISP_RESPONSE,  // 6 for display.cpp
FATAL_NO_TEMPSENSOR // 7 for tempsensor
 } ;

extern byte power_level; 
byte RFradio_init(byte cspin, byte resetpin);
void RFradio_printRegisters(const char *title);
void RFradio_sleep(void);
void RFradio_standby(void);
void RFradio_receive(void);
bool RFradio_gotpkt(void);
enum RFERR RFradio_getpkt (byte *ptr, byte length);
void RFRadio_pktstats(int8_t *rssi, int8_t *snr);
void RFradio_transmit(byte *ptr, int length);

void fatal(const char *, const byte num);