/***************************************************************************************************

    Battery-powered remote wireless temperature sensor, version 2

   This is the software for a small (3" x 3" x 1") battery-powered wireless sensor
   that measures temperature and humidity, displays the temperature, date, and time
   on a 1" x 1" e-paper display, and periodically sends data to a receiver that
   digitally simulates the resistance of thermistors inside up to four conventional
   (Mitsubishi) wired thermostats.

   This is version 2, reimplemented with parts optimized for low power consumption,
   because the batteries in the first version only lasted two to three months.
   The objective is to the have 3 AAA batteries (1000 mAH) last a year or more.
   That requires a time-averaged current draw of less than 0.11 mA (110 uA), and
   we achieve about 80-90 uA.

   The custom-built sensor hardware consist of:
   - Atmel ATmega328P processor, with 32K of Flash ROM and 2K bytes of RAM
     https://www.microchip.com/wwwproducts/en/ATmega328P
     We use the internal 8Mhz RC oscillator and 8x prescaler, for a 1Mhz clock speed.
   - HopeRF RFM95 LoRa 868/915 Mhz spread-spectrum radio
     http://www.hoperf.com/rf_transceiver/lora/RFM95W.html
   - Waveshare 1" x 1" 200x200 pixel e-Paper display
     https://www.waveshare.com/wiki/1.54inch_e-Paper_Module
     (The old version only! See below.)
   - Silicon Labs Si7021 I2C digital temperature/humidity sensor
     https://www.silabs.com/documents/public/data-sheets/Si7021-A20.pdf
     https://www.sparkfun.com/products/13763
   - Microchip MCP1711 150 mA ultra-low quiescent current LDO 3.3V regulator
     http://ww1.microchip.com/downloads/en/DeviceDoc/20005415D.pdf
   - 3-cell AAA battery holder
     https://www.digikey.com/product-detail/en/2479/36-2479-ND/303824
   - 2-position slide switch to set the sensor address and mode
     https://www.digikey.com/product-detail/en/1825360-3/450-1781-ND/3283586
   - a 2.1" x 2.7" 2-sided custom printed circuit board, made for $10 each by
     https://oshpark.com/
   - 3" x 3" x 1.2" vented plastic enclosure
     https://www.ebay.com/itm/Z-123-WH-Enclosure-multipurpose-X75-8mm-Y78-8mm-Z30-2mm-vented-ABS-Z123BABS/401475260109

   The design of the hardware and software for the sensors and the corresponding
   receiver is open-source.  See https://github.com/LenShustek/tempsensor.

   Our measured current draw for various phases is as follows:

    sleeping:            13 uA
    get temp/humidity:  4.7 mA for 12 msec
    read battery:       4.5 mA for 0.9 msec
    send packet, 15 dBm: 90 mA for 42 msec
    receive packet:      12 mA for 45 msec
    write date:         4.7 mA for 36 msec (once a day)
    write time:         4.7 mA for 36 msec (once a minute)
    write temp:         4.7 mA for 87 msec (only when it changes)
    update display:     8.2 mA for 360 msec (when any of the above happened)

   The long-term average is about 88 uA if we send a packet about
   every 2 minutes and change the time display every minute.
   So the batteries should last almost 18 months!

   Note that the quiescent current of 13 uA is not just the processor, which
   is about 7 uA with the watchdog timer running. It also includes the sleep
   currents of the e-paper display, RF radio, temperature/humidity sensor,
   battery test voltage divider, and voltage regulator.

   WARNING: The latest version of the Waveshare 1" x 1" display module includes
   a voltage regulator and level shifter so that it works on 5V systems. But
   that makes it useless for this application, since the quiescent current goes
   from less than 5 uA to more than 150 uA! I was able to get them to send me
   enough of the old version, but it is now out of stock. If you have high-level
   soldering skills, you can use a hot-air rework gun to remove the regulator and
   level-shifter from the new version, and then solder 7 tiny wires to bridge
   the signals. That works, but it's hard to do. It also has a different connector
   (2mm spacing instead of 2.5mm spacing) compared to the old version.
   An alternative would be to buy just the raw e-paper display and add the 19
   passive components described in the datasheet.

   Tips for programming the "bare" ATMega328P on our PC board:
      - A 2x3 pin header mates to the Adafruit USBTinyISP programmer,
        with the "power board" jumper removed.
      - Select the "USBTiny" programmer in the Aduino IDE tools/programmer menu.
         Then "Upload using programmer". May have to retry and/or power cycle
         the board and/or the USBTinyISP; it's finicky. Problems might be that the
         ePaper display and/or RFradio might load the MISO and SCLK lines if they
         are not initialized, or that you can't program when the CPU is in deep
         sleep. Try programming immediately after resetting the board
         but before it starts to go into deep sleep. Try having the batteries
         installed, or not.
      - Also can use AVRDUDE to check and modify fuse settings to change the
        clock, or to make the clock appear on the CLKO pin.
         avrdude -c usbtiny -p m328
      - The board selection is "Arduino/Genuino Uno", but the boards.txt file
        has to be changed to allow the frequency to be set to 1 MHz, which is
        the default with the internal 8Mhz oscillator and 8x precaling.
          #uno.build.f_cpu=1000000L
          ...
          menu.speed=CPU Speed
          uno.menu.speed.1=1 MHz
          uno.menu.speed.2=2 MHz
          uno.menu.speed.4=4 MHz
          uno.menu.speed.8=8 MHz
          uno.menu.speed.16=16 MHz
          uno.menu.speed.1.build.f_cpu=1000000L
          uno.menu.speed.2.build.f_cpu=2000000L
          uno.menu.speed.4.build.f_cpu=4000000L
          uno.menu.speed.8.build.f_cpu=8000000L
          uno.menu.speed.16.build.f_cpu=16000000L
       (See https://tttapa.github.io/Pages/Arduino/Bootloaders/ATmega328P-custom-frequency.html)

   The ATMega328p comes with fuses preprogrammed for the internal RC 8 Mhz
   ocillator divided by 8, for a system clock of 1 Mhz. We leave it at that.

   To output the clock to CLKO (pin 14 on the DIP, pin 12 on the TQFP) to
   see with a scope how far off the clock really is from 8 Mhz, change the
   lower fuse byte from the default of 0x62 to 0x22 as follows:
     avrdude -c usbtiny -p m328p -U lfuse:w:0x22:m
   You can then tweak OSCCAL (as we do in init()) to get closer to 8 Mhz.
   Not that it matters much...

   It's harder to figure out what the real frequency of the watchdog timer
   oscillator is, but we did that by looking at how good our timekeeping is
   when we're not getting response packets from the receiver. It's not that
   critical though; it only affects the accuracy of the time we display
   when out of communication with the central receiver station.

   Tips for debugging:
     A 6-pin header mates to the Taylor Roco CP2102 USB-to-TTL converter.
     Connect that to a Windows PC and run the Termite terminal emulator.
     Then all the Serial.print output will appear.

   By the way: AVR 8-bit processors like the ATMega328P have no divide
   instruction, so minimize division in places where time matters!

   ------------------------------------------------------------------------------
   Copyright (C) 2015,2018, Len Shustek

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
   ------------------------------------------------------------------------------
   Change log

   11 Nov 2015, V1.0, L. Shustek, First version.
   30 Nov 2015, V1.1, L. Shustek, Add retry for HC-12 errors.
   19 Dec 2015, V1.2, L. Shustek, Use zone 1..4 in displays.
                                  Read addr switches before going into sleep mode.
   05 Jul 2018, V2.0, L. Shustek, Reimplemented with completely different hardware for lower power.
   12 Oct 2018, V2.1, L. Shustek, Test code for diagnosing possible battery read glitches.
                                  Ignore the first A-to-D conversion after coming out of sleep,
                                  because "the first conversion might give a wrong value" (p250)
                                  might apply to that situation too.
   01 Nov 2018, V2.2, L. Shustek, Well, that doesn't solve the problem; about once every 10 times
                                  we get 3.5-3.7 volts from the A-to-D converter, and I don't know
                                  why. So hack: use the highest of the last 3 values.
   04 Feb 2019, V2.3, L. Shustek, Fix bug: must increment wday when we increment date on our own.

*****************************************************************************************************/
#define VERSION 23

#define TIME_BETWEEN_SENDS 2*60*1000UL // Nominally 2 minutes. Should it depend on whether temp changed?
//#define TIME_BETWEEN_SENDS 5*1000UL  // every 5 seconds, for antenna testing

#define DEBUG 0         // output monitor window debugging?
#define MARKTIMING 0    // generate logic analyzer timing pulse output?

#define RADIO  1        // switches to disable code for power consumption and other testing
#define EPAPER 1
#define TEMPSENSOR 1

#define AREF_MV 3300    // analog reference voltage is VDD = 3.3V

#include <Arduino.h>
#include <Wire.h>       // I2C for Si7021 temperature/humidity sensor
#include <SPI.h>        // SPI for ePaper display, and the RF Radio
#include <RFradio.h>    // our RFM95 LoRa radio library, in the Arduino library directory
#include "display.h"    // e-paper display

// pin assignments

// The #define numbers are the Arduino "fake" pin numbers
#define RF_CS_PIN A2    // RF radio chip select, PC2, PDIP pin 25, TQFP pin 25
#define RF_RESET_PIN A3 // RF radio reset, PC2, PDIP pin 26, TQFP pin 26
#define BATTERY_PIN A0  // battery voltage/2, PC0, PDIP pin 23, TQFP pin 23
#define SW1 6           // node address MSB, PD6, PDIP pin 12, TQFP pin 10
#define SW2 7           // node address LSB, PD7, PDIP pin 13, TQFP pin 11
#define LED 0           // LED, active low, PD0, PDIP pin 2, TQFP pin 30
// The pin assignments for the e-paper display are in display.h!

#define TIME_MARKER_PIN 9   // PB1, PDIP pin 15, TQFP pin 13
#define TIME_MARKER_PORT 0x05  // r/w data port B, 0-based address for SBI/CBI instructions
#define TIME_MARKER_BIT 1      // bit number (not mask)

// implicit pins used are:
//    MOSI: arduino pin 11, PB3, PDIP pin 17, TQFP pin 15
//    MISO: arduino pin 12, PB4, PDIP pin 18, TQFP pin 16
//    SCK:  arduino pin 13, PB5, PDIP pin 19, TQFP pin 17

byte node_addr;            // 0..3
byte current_temp = 0;     // degrees F
byte current_humid = 0;    // percent, 0..100
struct pkt_temp_t xmt_pkt; // the packet we send: temp, humidity, battery voltage
struct pkt_resp_t rcv_pkt; // the packet we receive: date, time
static bool display_working = false;
byte batt_prev = 0, batt_prevprev = 0;

//-----------------------------------------------------------------------------------------------------------------
//  Temperature/humidity sensor routines
//  The Si7021 communicates using the I2C protocol (SCL/SDA)
//-----------------------------------------------------------------------------------------------------------------

#define Si7021_I2CADDR       0x40   // I2C address
#define Si7021_WRITEREG      0xe6   // write User Register 1
#define Si7021_TEMP          0xf3   // measure temperature, no hold master mode
#define Si7021_HUMID         0xf5   // measure humidity, no hold master mode
#define Si7021_TEMP_PREV     0xe0   // return temp measured during humidity measurement

void Si7021_writereg(byte val) {  // write User Register 1
   Wire.beginTransmission(Si7021_I2CADDR);
   Wire.write(Si7021_WRITEREG);
   Wire.write(val);
   Wire.endTransmission(); }

void Si7021_init (void) { // initialize
   Wire.beginTransmission(Si7021_I2CADDR);
   Wire.write(0xfc);  // request 2nd access of serial number
   Wire.write(0xc9);
   Wire.endTransmission();
   Wire.requestFrom(Si7021_I2CADDR, 1);
   byte ID = Wire.read();  // SNB3 field should be 0x15 for Si7021
   if (ID != 0x15) {
      Serial.println("No temp sensor");
      fatal("No temp sensor", FATAL_NO_TEMPSENSOR); }
   Si7021_writereg(0x01);  // select 8 bit humidity, 12 bit temp
}

uint16_t Si7021_measure(byte cmd, byte delay_msec) {
   Wire.beginTransmission(Si7021_I2CADDR);
   Wire.write(cmd);
   Wire.endTransmission();
   delay(delay_msec);
   Wire.requestFrom(Si7021_I2CADDR, 2);
   byte msb = Wire.read();
   byte lsb = Wire.read() & 0xfc;
   return msb << 8 | lsb; }

void Si7021_gettemphum(void) {
   uint16_t raw_humid = Si7021_measure(Si7021_HUMID, 7);   // 8 bit humid takes 3.1 msec max, plus 3.8 for temp
   uint16_t raw_temp = Si7021_measure(Si7021_TEMP_PREV, 1); // 12 bit temp as measured already
   if (0 && DEBUG) {
      Serial.print("raw temp, humid: ");
      Serial.print(raw_temp); Serial.print(", ");
      Serial.print(raw_humid); Serial.println(); }
   // convert raw T to degrees F: ((175.72*raw/2^16)-46.85) * 1.8 + 32
   // or F = (316.296*raw - 52.33*2^16) / 2^16
   // To calculate with 32-bit integers, we scale up by a factor of 2^6 = 64
   // F = (20242*raw - 219487928 + 2^21 /*rounding*/) / 2^22
   // F = (20242*raw - 217390776) >> 22
   current_temp = (byte)(((uint32_t)raw_temp * 20242UL - 217390776UL) >> 22);
   // convert raw H to relative humidity in %: RH = (125*raw/2^16) - 6;
   if (raw_humid < 3146) raw_humid = 3146;  // prevent <0 or >100
   if (raw_humid > 55575) raw_humid = 55575;
   current_humid = (byte) ((((uint32_t)raw_humid * 125) >> 16) - 6);
   if (DEBUG) {
      Serial.print("converted temp, humid: ");
      Serial.print(current_temp); Serial.print(", ");
      Serial.print(current_humid); Serial.println(); } }

//-----------------------------------------------------------------------------------------------------------------
//    sleep routines
//-----------------------------------------------------------------------------------------------------------------

ISR (WDT_vect) { // watchdog timer interrupt
   __asm__ __volatile__ ("wdr");  // reset the watchdog timer
   MCUSR &= ~bit(WDRF); // reset WD system reset flag
   // Keep old prescaler setting to prevent unintentional time-out
   WDTCSR |= (1 << WDCE) | (1 << WDE); // enable changes
   WDTCSR = 0x00; // Turn off WDT
}
void sleep (byte sleep_code) {  // some of this is from http://gammon.com.au/power
   noInterrupts (); {
      MCUSR = 0; // clear various "reset" flags
      WDTCSR = bit (WDCE) | bit (WDE);   // allow changes to watchdog register, disable reset
      WDTCSR = bit (WDIE) | sleep_code;  // set interrupt mode, and the delay
      __asm__ __volatile__ ("wdr");      // reset and start the watchdog timer
      SMCR = 0x05; // set POWER_DOWN_SLEEP_MODE and SLEEP_ENABLE_MASK
      MCUCR = bit (BODS) | bit (BODSE);  // turn off brown-out detect during sleep
      MCUCR = bit (BODS); }
   interrupts ();   // guarantees the next instruction is executed
   __asm__ __volatile__ ( "sleep" );   // enter sleep mode
   SMCR = 0;  // disable sleep mode
}
#define turnon_SPI(x) PRR &= ~bit(PRSPI)
#define turnoff_SPI(x) PRR |= bit(PRSPI)

#define turnon_TWI(x) PRR &= ~bit(PRTWI)
#define turnoff_TWI(x) PRR |= bit(PRTWI)

#define turnon_ADC(x) {PRR &= ~bit(PRADC); ACSR &= ~bit(ACD); ADCSRA |= bit(ADEN);}
#define turnoff_ADC(x) {ADCSRA &= ~bit(ADEN); ACSR |= bit(ACD); PRR |= bit(PRADC);}

//-----------------------------------------------------------------------------------------------------------------
//    display and utility routines
//-----------------------------------------------------------------------------------------------------------------

void writedate(struct datetime_t *p) {
   static const char *days[] =
   {"???", "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };
   static const char *months[] =
   {"???", "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };
   char msg[20];
   if (p->wday > 7) p->wday = 0;
   if (p->month > 12) p->month = 0;
   sprintf_P(msg, PSTR("%s %s %d "), days[p->wday], months[p->month], p->date);
   dsp_writestr(&font24, 24, 125, msg); }

void writetime(struct datetime_t *p) {
   char msg[20];
   char ampm = 'a';
   byte hour = p->hour;
   if (hour > 11) {
      hour -= 12; ampm = 'p'; }
   if (hour == 0) hour = 12;
   sprintf_P(msg, PSTR("%2d:%02d %cm"), hour, p->min, ampm);
   dsp_writestr(&font24, 32, 150, msg); }

void fatal(char const * msg, const byte num) {
   if (DEBUG) {
      Serial.print("ERROR: ");
      Serial.println(msg); }
   if (display_working) {
      turnon_SPI();
      dsp_start(true); // full update mode
      dsp_writestr(&font24, 0, 125, (char *)"FATAL ERROR:");
      char buf[40];
      strcpy(buf, msg); // copy from FLASH
      buf[12] = 0;  // max 12 chars
      dsp_writestr(&font24, 0, 150, buf);
      dsp_displaymem(true);
      dsp_sleep(); // put the display into deep sleep
      turnoff_SPI(); }
   while (1) { // flash light as the last resort
      for (byte cnt = 0; cnt < 10; ++cnt) { // flutter to indicate "fatal error"
         digitalWrite(LED, LOW);
         delay(25);
         digitalWrite(LED, HIGH);
         delay(25); }
      delay(500);
      for (byte cnt = 0; cnt < num; ++cnt) { // count out the error number
         digitalWrite(LED, LOW);
         delay(200);
         digitalWrite(LED, HIGH);
         delay(400); }
      delay(500); } }

#if MARKTIMING
void MARKTIME(byte cnt) { // coded pin output to use the logic analyzer for timing
   do {
      __asm__ ("sbi %0,%1\n"::"I"(TIME_MARKER_PORT), "I"(TIME_MARKER_BIT));
      __asm__ ("nop\n");
      __asm__ ("cbi %0,%1\n"::"I"(TIME_MARKER_PORT), "I"(TIME_MARKER_BIT)); }
   while (--cnt); }
#else
#define MARKTIME(x)
#endif

//-----------------------------------------------------------------------------------------------------------------
//    Initialization
//-----------------------------------------------------------------------------------------------------------------

uint32_t time_between_sends = 0;  // the constant above, plus some fuzz based on our address
uint32_t last_send_time = 0;      // the millis() when we last sent a packet
byte displayed_min = 0xff;

struct datetime_t datetime_now = {        // the date/time now
   0, 0, 0, 1, 29, 7, 48 };               // (starting default is midnight July 29, 2018)
uint32_t datetime_now_millis = 0;         // the milliseconds now
uint32_t datetime_now_update_time = 0;    // when we last updated "now"

void setup() {
   if (DEBUG) {
      Serial.begin(9600);
      delay(1000);
      Serial.println("Sensor started.\n"); }
   else delay(1000);

   pinMode(SW1, INPUT_PULLUP); // configure and read node address switches
   pinMode(SW2, INPUT_PULLUP);
   node_addr = ( (digitalRead(SW1) << 1) | digitalRead(SW2)) ^ 0x03; // 0..3
   // After reading, set the switch inputs to output 0 instead.
   // Otherwise the 30 Kohm internal pullup uses 100 uA for each closed switch!
   digitalWrite(SW1, 0);
   digitalWrite(SW2, 0);
   pinMode(SW1, OUTPUT);
   pinMode(SW2, OUTPUT);

   digitalWrite(LED, HIGH);  // indicator light is active low
   pinMode(LED, OUTPUT);
   digitalWrite(TIME_MARKER_PIN, LOW); // logic analyzer time marker
   pinMode(TIME_MARKER_PIN, OUTPUT);

   for (int i = 0; i < 3; ++i) { // announce our presence
      digitalWrite(LED, LOW);
      delay(250);
      digitalWrite(LED, HIGH);
      delay(250); }

   if (DEBUG) { // display the switches and our node address
      Serial.print("switches: ");
      Serial.print(digitalRead(SW1) ? '1' : '0');
      Serial.print(digitalRead(SW2) ? '1' : '0');
      Serial.print("node addr: ");
      Serial.println(node_addr); }

   // Our power-management strategy is to keep everything turned off until we need to use it.
   // (Should we also disable brown-out detection via the fuse?)

   ADCSRA &= ~bit(ADEN);  // turn off ADCfirst; p.46: "The ADC must be disabled before shut down"
   ACSR |= bit(ACD);      // turn off analog comparator
   DIDR1 = bit(AIN1D) | bit(AIN0D); // disable analog comparator input buffers
   PRR =
      bit(PRTWI)      // turn off 2-wire interface (needed by the temp/humidity sensor)
      | bit(PRSPI)    // turn off SPI (needed by the radio and the display)
      | bit(PRADC)    // turn off the A-to-D converter (need by the battery monitor)
      | (DEBUG ? 0 : bit(PRUSART0))  // turn of the USART if not debugging (used by the serial monitor)
      | bit(PRTIM1)   // turn off timer/counter 1 (not used)
      | bit(PRTIM2);  // turn off timer/counter 2 (not used)
   // We leave out PRTIM0 to keep timer/counter 0 running for timekeeping: millis(), delay()
   TCCR1B = 0;  // turn off 16-bit timer/counter 1
   TCCR2B = 0;  // turn off 16-bit timer/counter 2

   // We use tables 35-38 and 35-39 on page 607 to compensate for what we see as the
   // typical internal RC oscillator frequency of 8.1 Mhz instead of 8.0 Mhz at 3.3V
   // and 20C, or 1.2% fast. This isn't as important as our correction to the watchdog
   // timer oscillator correction that comes later, but it can't hurt.
   // (The notes at the beginning explain how to program CPU fuses so you can measure
   //  the actual oscillator frequency with an oscilloscope.)
   //   OSCCAL = 125; // 10.77 Mhz
   //   OSCCAL = 86;  //  8.01 Mhz
   //   OSCCAL = 85;  //  7.96 Mhz
   //   OSCCAL = 83;  //  7.84 Mhz
   //   OSCCAL = 80;  //  7.66 Mhz
   //   OSCCAL = 64;  //  6.95 Mhz
   // Note that 0 to 127 is the low range, and 128 to 255 is the high range.
   // Why is the default 128 (0x80), the first point of the higher range?
   // Shouldn't that always result in a frequency *less* than 8 Mhz?

   OSCCAL = 86;  // 8.008 Mhz, typically

   if (EPAPER) {
      if (DEBUG) Serial.println("starting display");
      turnon_SPI();
      dsp_init(); // initialize display
      dsp_start(true); // start in full update mode
      dsp_clearmem();  // clear first page
      dsp_displaymem(true);
      dsp_clearmem();  // clear second page
      dsp_displaymem(true);
      if (DEBUG) Serial.println("display initialized");
      dsp_start(false); // restart in partial update mode

      char msg[25];   // show some configuration info

      sprintf_P(msg, PSTR("vers %d.%01d"), VERSION / 10, VERSION % 10);
      dsp_writestr(&font24, 24, 125, msg);
      dsp_displaymem(true);
      delay(1000);

      dsp_writestr(&font24, 24, 125, (char *)__DATE__); // compile date
      dsp_displaymem(true);
      delay(1000);

      turnon_ADC(); // current battery voltage
      analogReference(DEFAULT);  // use the 3.3V reference for battery voltage check
      uint16_t rawv = analogRead(BATTERY_PIN);
      turnoff_ADC();
      byte vx10 = (uint32_t)rawv * AREF_MV / (1023UL * 50); // voltage times 10 from VCC/2 network
      if (DEBUG) {
         Serial.print("raw battery voltage: "); Serial.println(rawv); }
      sprintf_P(msg, PSTR("%d.%dV       "), vx10 / 10, vx10 % 10);
      dsp_writestr(&font24, 24, 125, msg);
      dsp_displaymem(true);
      delay(1000);

      sprintf_P(msg, PSTR("zone %d     "), node_addr + 1); // our zone number
      dsp_writestr(&font24, 24, 125, msg);
      dsp_displaymem(true);
      delay(1000);

      dsp_sleep(); // put the display into deep sleep
      turnoff_SPI();
      display_working = true;
      if (DEBUG) Serial.println("display init done"); }

   if (TEMPSENSOR) {
      turnon_TWI();
      Si7021_init();                   // initialize the temp sensor
      if (DEBUG) Si7021_gettemphum();
      turnoff_TWI(); }

   if (RADIO) {
      turnon_SPI();
      RFradio_init(RF_CS_PIN, RF_RESET_PIN); // initialize the radio
      turnoff_SPI();
      if (DEBUG) Serial.println("radio initialized");
      // dither the time between sends to avoid consistent collisions with other nodes
      time_between_sends = TIME_BETWEEN_SENDS +  ((uint32_t)node_addr << 10); } }

//-----------------------------------------------------------------------------------------------------------------
//    The main loop
//-----------------------------------------------------------------------------------------------------------------

void update_time(void) { // update datetime_now, based on elapsed time
   uint32_t rightnow = millis();
   datetime_now_millis += rightnow - datetime_now_update_time; // the msec deficit
   // This should only loop at most 8 times, for an 8-second sleep, but it's faster than division
   while (datetime_now_millis >= 1000) {
      datetime_now_millis -= 1000;
      if (++datetime_now.sec >= 60) {
         datetime_now.sec -= 60;
         if (++datetime_now.min >= 60) {
            datetime_now.min -= 60;
            if (++datetime_now.hour >= 24) {
               datetime_now.hour -= 24;
               if (++datetime_now.wday > 7) datetime_now.wday = 1;
               ++datetime_now.date;
               // Ignore the complicated rollover to the next month;
               // the next packet we receive will fix it anyway.
            } } } }
   datetime_now_update_time = rightnow; }

void loop (void) {

   #if 0
   // TEMP TEST CODE FOR BATTERY READ FAILURE
   int row = 0;
   unsigned long counter = 1;
   while (1) {
      turnon_ADC();
      analogReference(DEFAULT);  // use the 3.3V reference for battery voltage check
      uint16_t rawv = analogRead(BATTERY_PIN);
      turnoff_ADC();
      byte volts = (uint32_t)rawv * AREF_MV / (1023UL * 50); // VCC/2 resistor network!
      if (volts < 40 || counter < 10) { // display a few, then only if battery voltage is too low
         char msg[25];
         turnon_SPI(); dsp_start(false); // partial update mode
         sprintf_P(msg, PSTR("%5lu %d.%1dV"), counter % 100000L, volts / 10, volts % 10);
         dsp_writestr(&font24, 8, row, msg); dsp_displaymem(true);
         dsp_writestr(&font24, 8, row, msg); dsp_displaymem(true);
         row += 24; if (row > 175) row = 0;
         dsp_sleep(); turnoff_SPI(); // put the display into deep sleep
      }
      sleep(0x03); // 125 msec
      if (++counter % (5000 / 125) == 0) { // flash LED every 5 seconds
         digitalWrite(LED, LOW); delay(200); digitalWrite(LED, HIGH); } }
   #endif

   static bool neversent = true;

   static byte displayed_temp = 0;
   static byte tempchange_countdown = 2;  // make sure temp is changed on both display pages

   static byte displayed_date = 0;
   static byte datechange_countdown = 2;  // make sure date is changed on both display pages

   // check if it is time to read the temperature, send a packet, and get a response

   if (neversent || millis() - last_send_time >= time_between_sends) { // (works ok at overflow)
      xmt_pkt.addr = node_addr;
      xmt_pkt.type = PT_TEMP;
      xmt_pkt.sw_version = VERSION;
      ++xmt_pkt.sequence;

      MARKTIME(1);  //*** read the temperature and humidity
      turnon_TWI();
      Si7021_gettemphum();
      xmt_pkt.temperature = current_temp;
      xmt_pkt.humidity = current_humid;
      turnoff_TWI();

      MARKTIME(2);  //*** read the battery voltage
      turnon_ADC();
      analogReference(DEFAULT);  // use the 3.3V reference for battery voltage check
      uint16_t rawv = analogRead(BATTERY_PIN);
      turnoff_ADC();

      byte batt = (uint32_t)rawv * AREF_MV / (1023UL * 50); // VCC/2 resistor network!
      xmt_pkt.battery = batt; // use maximum of last three values, b/c occsionally it's bogus
      if (batt_prev > xmt_pkt.battery) xmt_pkt.battery = batt_prev;
      if (batt_prevprev > xmt_pkt.battery) xmt_pkt.battery = batt_prevprev;
      batt_prevprev = batt_prev;
      batt_prev = batt;

      MARKTIME(3); //*** send the packet
      turnon_SPI();
      RFradio_transmit((byte *)&xmt_pkt, RF_TEMP_PKT_SIZE);

      MARKTIME(4); //*** receive the response
      RFradio_receive();
      uint32_t starttime = millis();
      enum RFERR error = RF_UNKNOWN;
      while (1) {  // wait for the response packet
         if (RFradio_gotpkt()) {
            MARKTIME(5); //*** got the response
            error = RFradio_getpkt((byte *)&rcv_pkt, RF_RESP_PKT_SIZE);
            if (error == RF_OK) {
               datetime_now = rcv_pkt.datetime;  // copy the packet's datetime
               datetime_now_update_time = millis();
               datetime_now_millis = 0; }
            else {
               MARKTIME(5);
               MARKTIME(error);
               // do something to display the receive packet error?
            }
            break; }
         if (millis() - starttime > 75) { // msec to wait for packet
            MARKTIME(5);
            MARKTIME(RF_NOPKT);
            // do something to display lack of response packet?
            break; } //
      }
      RFradio_sleep();
      turnoff_SPI();
      last_send_time = millis();
      neversent = false; //
   } // send

   // check if we need to update the display because the minute changed

   update_time();
   if (datetime_now.min != displayed_min) {
      displayed_min = datetime_now.min; // the minute has changed
      MARKTIME(6);  //*** write updated date/time/temp
      turnon_SPI();
      dsp_start(false); // partial update mode
      if (datetime_now.date != displayed_date) // only write date if changed
         datechange_countdown = 2;
      if (datechange_countdown) { // but do it twice, for both display pages
         writedate(&datetime_now); // write day and date
         displayed_date = datetime_now.date;
         --datechange_countdown; }
      writetime(&datetime_now); // always write time
      if (current_temp != displayed_temp) // only write temp if changed
         tempchange_countdown = 2;
      if (tempchange_countdown) { // but do it twice, for both display pages
         char msg[25];
         sprintf_P(msg, PSTR("%2d`"), min(current_temp, 99));
         dsp_writestr(&font67, 16, 40, msg);  // write temperature
         displayed_temp = current_temp;
         --tempchange_countdown; }
      MARKTIME(7); //*** update the display
      dsp_displaymem(true);
      dsp_sleep(); // put the display into deep sleep
      turnoff_SPI(); }

   // figure out how long we should sleep for

   uint32_t time_to_next_minute_change = (60 - datetime_now.sec) * 1000;

   uint32_t time_since_last_send = millis() - last_send_time;
   // Think twice if you have a plan for how to simplify this calculation!
   uint32_t time_to_next_send;
   if (time_since_last_send >= time_between_sends)
      time_to_next_send = 0;
   else time_to_next_send = time_between_sends - time_since_last_send;

   uint16_t sleep_time = min(time_to_next_send, time_to_next_minute_change);

   // The watchdog oscillator tends to run slow! Table 35-10 on page 605, and
   // our measurements, show that at 20C and 3.3V it typically is 118-124 Khz
   // instead of 128 Khz, or 6-7% slow. That matters when we're not getting
   // frequent clock updates from the receiver, since we have to adjust for
   // the time during sleep when TIMER0 is not running. We thus adjust our
   // sleep time numbers with the WD macro so that we're closer to real time.

#define WDSPEED 122  // typical speed of watchdog oscillator in Khz
#define WD(x) (uint16_t)((uint32_t)x*128/WDSPEED)

   byte sleep_code; // WDP3, WDP2, WDP1, WDP0, but not in consecutive bits!
   if (sleep_time >= WD(8000)) {
      sleep_code = 0x21;  // 8 seconds is the most the processor can sleep
      sleep_time = WD(8000); }
   else if (sleep_time >= WD(4000)) {
      sleep_code = 0x20;  // 4 seconds
      sleep_time = WD(4000); }
   else if (sleep_time >= WD(2000)) {
      sleep_code = 0x07;  // 2 seconds
      sleep_time = WD(2000); }
   else if (sleep_time >= WD(1000)) {
      sleep_code = 0x06;  // 1 seconds
      sleep_time = WD(1000); }
   else if (sleep_time >= WD(500)) {
      sleep_code = 0x05;  // 0.5 seconds
      sleep_time = WD(500); }
   else if (sleep_time >= WD(250)) {
      sleep_code = 0x04;  // 0.25 seconds
      sleep_time = WD(250); }
   else if (sleep_time >= WD(125)) {
      sleep_code = 0x03;   // 0.125 sec
      sleep_time = WD(125); }
   else if (sleep_time >= WD(64)) {
      sleep_code = 0x02;   // 64 msec
      sleep_time = WD(64); }
   else if (sleep_time >= WD(32)) {
      sleep_code = 0x01;   // 32 msec
      sleep_time = WD(32); }
   else
      sleep_code = 0;   // don't sleep for less than that; just delay

   MARKTIME(8); //*** sleep
   if (sleep_code == 0)
      delay(sleep_time);  // delay instead of sleeping
   else {
      sleep(sleep_code);  // "deep sleep" hibernation; only the watchdog timer runs
      noInterrupts (); {
         extern volatile uint32_t timer0_millis;
         timer0_millis += sleep_time; // approximate adjustment to the clock, which was stopped
         interrupts(); } }

   MARKTIME(9); //*** wake up

   //while(1) sleep(0x21); // for quiescent current measurement

}

//* tempsensor.ino

