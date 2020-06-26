// file: display.c
/***************************************************************************************************

   E-paper display routines for the
   battery-powered remote wireless temperature sensor, version 2

   These drive the Waveshare 1" x 1" 200 x 200 pixel e-Paper display
   https://www.waveshare.com/wiki/1.54inch_e-Paper_Module

   The screen is 200 by 200 pixels, with X-axis horizontal, the Y-axis
   vertical, and the origin at the top left corner when held with the
   24-pin connector for the display itself at the bottom, and the
   8-pin system connector at the top:

        |--------O-O-O-O-O-O-O-O--------|
   000  |     ----> access order        |
        |                               |
        |              *                |
   Y    |             * *               |
        |            *****              |
        |           *     *             |
   199  |                               |
        |-------------------------------|
        000             X              199

   Memory access for a subsection is X first, then Y.
   The X coordinate must be a multiple of 8, because
   transfers are in byte-size chunks!

   About our fonts:

   We use only fixed-width fonts whose width is a multiple of 8 bits.

   The 24-raster-high "font24" is a modification of the sample font provided
   by Waveshare, and contains all the ASCII characters.

   The 34- and 67-raster high fonts were created in two steps from the
   Windows 10 "Arial Black" TrueType font, and contain only the numerical
   digits and the degree symbol replacing the ASCII accent grave, 96.
   The process was as follows:

    1. Use the free Bitmap Font Generator from AngelCode.com,
       http://www.angelcode.com/products/bmfont/
       to create fixed-width glpyh characters of 64 or 128 pixels
       for the selected characters, without font smoothing,
       and save as XML to .fnt and .pmg files.
       The configuration file is saved as bmfontgen.bmfc.

    2. Run the free Python program "bmtfontgen"
       https://larsee.com/blog/2014/05/converting-fonts-to-c-source-using-bmfont2c/
       which reads a configuration file bmfont2c.cfg, which points to the .fnt
       file, which in turn points to the .pmg file. It creates .h and .c files of
       bitmap character initialization, which can be tweaked by hand and/or
       by changing parameters in the bmfont2c.cfg file

   ------------------------------------------------------------------------------
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
#include <Arduino.h>
#include <SPI.h>
#include "display.h"
#include <RFradio.h> // only for global FATAL error codes

// EPD1IN54 commands
#define DRIVER_OUTPUT_CONTROL                       0x01
#define BOOSTER_SOFT_START_CONTROL                  0x0C
#define GATE_SCAN_START_POSITION                    0x0F
#define DEEP_SLEEP_MODE                             0x10
#define DATA_ENTRY_MODE_SETTING                     0x11
#define SW_RESET                                    0x12
#define TEMPERATURE_SENSOR_CONTROL                  0x1A
#define MASTER_ACTIVATION                           0x20
#define DISPLAY_UPDATE_CONTROL_1                    0x21
#define DISPLAY_UPDATE_CONTROL_2                    0x22
#define WRITE_RAM                                   0x24
#define WRITE_VCOM_REGISTER                         0x2C
#define WRITE_LUT_REGISTER                          0x32
#define SET_DUMMY_LINE_PERIOD                       0x3A
#define SET_GATE_TIME                               0x3B
#define BORDER_WAVEFORM_CONTROL                     0x3C
#define SET_RAM_X_ADDRESS_START_END_POSITION        0x44
#define SET_RAM_Y_ADDRESS_START_END_POSITION        0x45
#define SET_RAM_X_ADDRESS_COUNTER                   0x4E
#define SET_RAM_Y_ADDRESS_COUNTER                   0x4F
#define TERMINATE_FRAME_READ_WRITE                  0xFF


static const unsigned char lut_full_update[] = {
   0x02, 0x02, 0x01, 0x11, 0x12, 0x12, 0x22, 0x22,
   0x66, 0x69, 0x69, 0x59, 0x58, 0x99, 0x99, 0x88,
   0x00, 0x00, 0x00, 0x00, 0xF8, 0xB4, 0x13, 0x51,
   0x35, 0x51, 0x51, 0x19, 0x01, 0x00 };

static const unsigned char lut_partial_update[] = {
   0x10, 0x18, 0x18, 0x08, 0x18, 0x18, 0x08, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x13, 0x14, 0x44, 0x12,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

#define FASTIO 1
#if FASTIO
   // These are much faster than digitalWrite: about 4 usec instead of 80!
   #define setbit(pin,reg,bitnum) __asm__ ("sbi %0,%1\n"::"I"(reg),"I"(bitnum))
   #define clrbit(pin,reg,bitnum) __asm__ ("cbi %0,%1\n"::"I"(reg),"I"(bitnum))
#else
   #define setbit(pin,reg,bitnum) digitalWrite(pin, HIGH)
   #define clrbit(pin,reg,bitnum) digitalWrite(pin, LOW)
#endif

inline void dsp_sendcmd(byte dat) __attribute__((always_inline));
inline void dsp_sendcmd(byte cmd) {
   clrbit(DSP_DC_PIN, DSP_DC_PORT, DSP_DC_BIT); // "command"
   clrbit(DSP_CS_PIN, DSP_CS_PORT, DSP_CS_BIT); // "chip select"
   SPI.transfer(cmd);
   setbit(DSP_CS_PIN, DSP_CS_PORT, DSP_CS_BIT); }

inline void dsp_senddata(byte dat) __attribute__((always_inline));
inline void dsp_senddata(byte dat) {
   setbit(DSP_DC_PIN, DSP_DC_PORT, DSP_DC_BIT); // "data"
   clrbit(DSP_CS_PIN, DSP_CS_PORT, DSP_CS_BIT); // "chip select"
   SPI.transfer(dat);
   setbit(DSP_CS_PIN, DSP_CS_PORT, DSP_CS_BIT); }

void dsp_wait(void) {
   if (digitalRead(DSP_BUSY_PIN) == LOW) return; // quick exit if already done
   unsigned long startwait = millis(); // otherwise start timing
   while (digitalRead(DSP_BUSY_PIN) == HIGH) // fatal error if timeout (2 seconds)
      if (millis() - startwait > 2000) fatal("No display", FATAL_NO_DISP_RESPONSE); }

bool dsp_busy(void) {
   return (digitalRead(DSP_BUSY_PIN) == HIGH); }

void dsp_sleep (void) {
   dsp_sendcmd(DEEP_SLEEP_MODE);
   dsp_senddata(0x01); // "enter Deep Sleep Mode"
}

void dsp_start(bool full_update) { // start, or restart after deep sleep
   delay(1);
   digitalWrite(DSP_RESET_PIN, LOW);
   delay(1);
   digitalWrite(DSP_RESET_PIN, HIGH);
   dsp_wait();
   dsp_sendcmd(DRIVER_OUTPUT_CONTROL);
   dsp_senddata((EPD_HEIGHT - 1) & 0xFF);
   dsp_senddata(((EPD_HEIGHT - 1) >> 8) & 0xFF);
   dsp_senddata(0x00);                     // GD = 0; SM = 0; TB = 0;
   dsp_sendcmd(BOOSTER_SOFT_START_CONTROL);
   dsp_senddata(0xD7);
   dsp_senddata(0xD6);
   dsp_senddata(0x9D);
   dsp_sendcmd(WRITE_VCOM_REGISTER);
   dsp_senddata(0xA8);                     // VCOM 7C
   dsp_sendcmd(SET_DUMMY_LINE_PERIOD);
   dsp_senddata(0x1A);                     // 4 dummy lines per gate
   dsp_sendcmd(SET_GATE_TIME);
   dsp_senddata(0x08);                     // 2us per line
   dsp_sendcmd(DATA_ENTRY_MODE_SETTING);
   dsp_senddata(0x03);                     // X increment; Y increment
   dsp_sendcmd(WRITE_LUT_REGISTER);        // write the lookup table
   byte const *lutptr = full_update ? lut_full_update : lut_partial_update;
   for (int i = 0; i < 30; i++) {
      dsp_senddata(*lutptr++); } }

bool dsp_init (void) {  // one-time initialization
   digitalWrite(DSP_CS_PIN, HIGH);
   digitalWrite(DSP_RESET_PIN, HIGH);
   pinMode(DSP_CS_PIN, OUTPUT);
   pinMode(DSP_DC_PIN, OUTPUT);
   pinMode(DSP_RESET_PIN, OUTPUT);
   pinMode(DSP_BUSY_PIN, INPUT);
   SPI.begin();
   SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
   // That 2 Mhz is aspirational; with a 1 Mhz processor system clock,
   // SPI will be downgraded to 500 Khz.
   return true; }

void dsp_setmemarea(int x_start, int y_start, int x_end, int y_end) {
   dsp_sendcmd(SET_RAM_X_ADDRESS_START_END_POSITION);
   /* x point must be the multiple of 8 or the last 3 bits will be ignored */
   dsp_senddata((x_start >> 3) & 0xFF);
   dsp_senddata((x_end >> 3) & 0xFF);
   dsp_sendcmd(SET_RAM_Y_ADDRESS_START_END_POSITION);
   dsp_senddata(y_start & 0xFF);
   dsp_senddata((y_start >> 8) & 0xFF);
   dsp_senddata(y_end & 0xFF);
   dsp_senddata((y_end >> 8) & 0xFF); }

void dsp_setmemptr (int x, int y) {
   dsp_sendcmd(SET_RAM_X_ADDRESS_COUNTER);
   /* x point must be the multiple of 8 or the last 3 bits will be ignored */
   dsp_senddata((x >> 3) & 0xFF);
   dsp_sendcmd(SET_RAM_Y_ADDRESS_COUNTER);
   dsp_senddata(y & 0xFF);
   dsp_senddata((y >> 8) & 0xFF);
   dsp_wait(); }

void dsp_clearmem(void) {
   dsp_setmemarea(0, 0, EPD_WIDTH - 1, EPD_HEIGHT - 1);
   dsp_setmemptr(0, 0);
   dsp_sendcmd(WRITE_RAM);
   for (int y = 0; y < EPD_WIDTH; ++y)
      for (int x = 0; x < EPD_HEIGHT / 8; ++x)
         dsp_senddata(0xff); // set to white
}
void dsp_displaymem(bool wait) {
   dsp_sendcmd(DISPLAY_UPDATE_CONTROL_2);
   dsp_senddata(0xc4);
   dsp_sendcmd(MASTER_ACTIVATION);
   dsp_sendcmd(TERMINATE_FRAME_READ_WRITE);
   if (wait) dsp_wait();
   // Doing the wait with SPI off only changes a typical writestr/displaymem
   // loop from 4.58 mA to 4.42 mA, so it really isn't worth it.
   // (Table 31-13 on page 473 says SPI takes 43 uA at 4 Mhz and 3V.)
}
void dsp_senddata(byte * ptr, int cnt) {
   dsp_sendcmd(WRITE_RAM);
   while (cnt--) dsp_senddata(~*ptr++); }

void dsp_writestr_mag(const struct font_t *font, int x, int y, int magnification, char *msg) {
   // Display the message in the specified font with a magnification from 1 to 5.
   // The x position must be a multiple of 8.
   byte charw = pgm_read_byte(&font->charw); // character width in bits (multiple of 8!)
   byte charh = pgm_read_byte(&font->charh); // character height in bits (arbitrary)
   byte charwbytes = charw >> 3; // character width in bytes
   int bytes_per_char = charwbytes * charh;
   byte *fontdataptr = pgm_read_word(&font->fontdata); // start of font character bitmaps
   //Serial.print("writing: "); Serial.println(msg);
   //Serial.print("charw="); Serial.print(charw); Serial.print(", charh="); Serial.println(charh);
   int length = strlen(msg);
   // define an area within the screen memory
   dsp_setmemarea(x, y, x + length * charw * magnification - 1, y + charh * magnification - 1);
   // start the pointer in the upper left corner; scan is horizontal first
   dsp_setmemptr(x, y);
   dsp_sendcmd(WRITE_RAM); // issue the "screen memory write" command
   byte bits_ready = 0; // how many bits are ready to go out
   byte dupmask;  // the byte where we accumulate them
   int rowoffset = 0;
   for (byte row = 0; row < charh; ++row) { // for each row
      // There are various optimizations we just put into dsp_writestr() that we could use here too,
      // but we're not using this routine at the moment.
      for (byte rowdup = 0; rowdup < magnification; ++rowdup) { // for each duplicate of the row
         for (byte chnum = 0; chnum < length; ++chnum) { // for each character in the string
            int ndx; // pointer to start of bitmap data for the character
            if (msg[chnum] < DSP_MIN_ASCII || msg[chnum] > DSP_MAX_ASCII) ndx = -1; // invalid character unless ' ' through '~'
            else {
               byte mapped_char = pgm_read_byte(&font->charmap[0] + msg[chnum] - DSP_MIN_ASCII);
               //Serial.print("char "); Serial.print(msg[chnum]); Serial.print(" mapped to "); Serial.println(mapped_char);
               ndx = mapped_char == XX
                     ? -1 // the font doesn't have that character
                     : mapped_char * bytes_per_char + rowoffset; // start of the character bitmap
            }
            for (byte chbyte = 0; chbyte < charwbytes; ++chbyte) { // for each bitmap byte of that character
               byte mask = ndx == -1
                           ? 0x00  // all black for unknown character
                           : ~pgm_read_byte(fontdataptr + ndx + chbyte);
               for (byte bitt = 0; bitt < 8; ++bitt) { // for each bit of the bitmap
                  for (byte dup = 0; dup < magnification; ++dup) { // for each duplicate of that bit
                     dupmask = (dupmask << 1) | (mask >> 7); // make a copy of it
                     if (++bits_ready == 8) {
                        dsp_senddata(dupmask); // we've accumulated a full byte, so write it
                        bits_ready = 0; } }
                  mask <<= 1; // go to the next bitmap bit
               } } } }
      rowoffset += charwbytes; }
   //caller must eventually do this: dsp_displaymem();
}

void dsp_writestr(const struct font_t *font, int x, int y, char *msg) {
   // Display the message in the specified font without magnification.
   // The x position must be a multiple of 8.
   byte charw = pgm_read_byte(&font->charw); // character width in bits (multiple of 8!)
   byte charh = pgm_read_byte(&font->charh); // character height in bits (arbitrary)
   byte charwbytes = charw >> 3; // character width in bytes
   int bytes_per_char = charwbytes * charh;
   byte *fontdataptr = pgm_read_word(&font->fontdata); // start of font character bitmaps
   //Serial.print("writing: "); Serial.println(msg);
   //Serial.print("charw="); Serial.print(charw); Serial.print(", charh="); Serial.println(charh);
   int length = strlen(msg);
   // define an area within the screen memory
   dsp_setmemarea(x, y, x + length * charw - 1, y + charh - 1);
   // start the pointer in the upper left corner; scan is horizontal first
   dsp_setmemptr(x, y);
   dsp_sendcmd(WRITE_RAM); // issue the "screen memory write" command
   byte rowcnt = charh; do { // for each row
      for (byte chnum = 0; chnum < length; ++chnum) { // for each character in the string
         byte mapped_char;
         if (msg[chnum] < DSP_MIN_ASCII || msg[chnum] > DSP_MAX_ASCII) mapped_char = XX; // invalid character unless ' ' through '~'
         else mapped_char = pgm_read_byte(&font->charmap[0] + msg[chnum] - DSP_MIN_ASCII); // could still be XX
         byte chbytecnt = charwbytes;
         if (mapped_char == XX) {
            do {
               dsp_senddata(0x00); }
            while (--chbytecnt); }
         else {
            byte *chardataptr = // pointer to start of this row of the bitmap data for the character
               fontdataptr + mapped_char * bytes_per_char;
            do {
               dsp_senddata(~pgm_read_byte(chardataptr++)); }
            while (--chbytecnt); } //
      } // next char
      fontdataptr += charwbytes; }
   while (--rowcnt); // next row
   //caller must eventually do this: dsp_displaymem();
}
//* display.cpp
