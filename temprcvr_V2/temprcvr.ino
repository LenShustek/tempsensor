/**************************************************************************************

    remote temperature receiver

   This software is inside a box that receives wireless data from four remote
   temperature sensors. We replaces the thermistors in our Mitsubishi PAR-21MAA
   thermostats with a digitally-generated resistance that reflects the
   temperature at the remote locations.

   The custom-built hardware for the receiver consists of the following:
   - Teensy 3.1 microcontroller from http://www.pjrc.com
   - HopeRF RFM95 LoRa 868/915 Mhz spread-spectrum radio
     http://www.hoperf.com/rf_transceiver/lora/RFM95W.html
   - four Microchip MCP4251-series 50K digital potentiometers
     https://www.microchip.com/wwwproducts/en/MCP4251
   - 4-line by 20-character LCD display
     https://www.adafruit.com/product/198
   - Chronodot realtime clock with DS3231 and CR1632 3V battery
     http://docs.macetech.com/doku.php/chronodot_v2.0
   - a piezo-electric buzzer for making low-battery alarm chirps
   - two pushbutton switches
   - three mode-setting slide switches (currently unused)

   The four lines of the display are used as follows:
    line 1:  titles of the four zones
    line 2:  current zone temperatures
    line 3:  current humidity, or remote sensor battery status "LoBat"
    line 4:  the current date and time, alternating with various
             statistical and debugging information

   The design of the hardware and software for this receiver and the corresponding
   sensors is open-source.  See https://github.com/LenShustek/tempsensor.

   -------------------------------------------------------------------------------
    (C) Copyright 2015,2018 Len Shustek
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
   -------------------------------------------------------------------------------
   Change log

   11 Nov 2015, V1.0, L. Shustek; first version
   29 Nov 2015, V1.1, L. Shustek; add pushbutton commands
   14 Dec 2015, V1.2, L. Shustek; change zone number displays to be 1..4
    8 Jan 2016, V1.3, L. Shustek; expect transmissions only every 30 seconds
    9 Jan 2016, V1.4. L. Shustek; use packet sequence numbers to tally lost packets
   30 Jan 2016, V1.5, L. Shustek; better condensed packet count display
   29 Feb 2016, v1.6, L. Shustek; add humidity display, and button for holding display
   15 Oct 2016, V1.7, L. Shustek; expect and display sensors' software version number;
                                  longer comm delay threshold for new temp send 1/minute
   8 Aug 2018,  V2.0, L. Shustek; change to use LoRa radio for the new temperature
                                  sensors, and add signal strength display in dBm
*/

#define VERSION "2.0"
#define BATTERY_THRESHOLD 38     // minimum good battery level in tenths of a volt
#define DEBUG 0
#define TESTS 0

#define LOST_COMM_TIME (4*60+20) // 4+ minutes before declaring a zone "lost"

#include <arduino.h>
#include <LiquidCrystal.h>  // for LCD display
#include <SPI.h>            // for the digital pots and the RF LoRa radio using SPI
#include <Wire.h>           // for the realtime clock using I2C
#include <RFradio.h>        // our RFM95 LoRa radio library
#include "IntervalTimer.h"  // for timing

// pin assignments

#define SPEAKER_POS 20    // bipolar speaker pin
#define SPEAKER_NEG 21    // bipolar speaker pin
#define RF_CS 4           // RF radio chip select pin
#define RF_RESET 22       // RF radio reset pin
#define MODESW_1 23       // mode slide switch pin
#define MODESW_2 16       // mode slide switch pin
#define MODESW_3 17       // mode slide switch pin
#define BUTTON_1 2        // action pushbutton pin, which is normally closed
#define BUTTON_2 3        // action pushbutton pin, which is normally closed
#define BUTTON_PUSHED 1
#define BUTTON_NOT_PUSHED 0
#define SPI_MOSI 11       // default SPI pins for pots and RF radio
#define SPI_MISO 12       // default SPI pins for pots and RF radio
#define SPI_CLK 13        // default SPI pins for pots and RF radio
#define LED 13            // (SPI_CLK is also connected to the onboard LED)
#define SPI_CS1 14        // select pin for the first two pots, IC U$2
#define SPI_CS2 15        // select pin for second two pots, IC U$4
LiquidCrystal lcd(/*RS*/ 5, /*E*/ 6, /*D4-D7*/ 7, 8, 9, 10);  // LCD pins

unsigned long malformed_packets = 0;

struct { // current information about each zone
   const char *name;
   byte temperature;
   byte battery;
   byte humidity;
   bool lost_comm; // did we lose communication?
   uint32_t last_sequence;
   unsigned long good_packets;
   unsigned long bad_packets;
   unsigned long missed_packets;
   unsigned long msec_since_packet;
   int8_t rssi; // last signal strength
   int8_t snr;  // last signal-to-noise ratio
   byte sw_version; }
zone[4];

bool refresh_zoneinfo = false;    // do we need to refresh zone info display on rows 1 and 2?

struct datetime_t
   timenow, // current time
   clock_init = { // when we first wrote the code
   50, 10, 8, 4, 8, 8, 48 };

static const char *months[] = {
   // index 1..12 from realtime clock
   "xxx", "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
static const byte days_in_month []  = {
   // index 1..12 from realtime clock
   99, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

#define PACKETLOG_SIZE 100
struct { // the log of incoming packets we record for debugging
   byte addr; // 0..3
   byte temperature;
   byte battery;
   struct datetime_t timestamp; } packet_log[PACKETLOG_SIZE];
int packetlog_count = 0; // number of events stored
int packetlog_index = 0; // next log array location to write

#define ERRORLOG_SIZE 25
struct { // the log of errors we record for debugging
   byte addr;   // currently we only store "disconnect" events
   struct datetime_t timestamp; } error_log[ERRORLOG_SIZE];
int errorlog_count = 0; // number of errors stored
int errorlog_index = 0; // next log array location to write

//-----------------------------------------------------------------------------------------------------------------
// display routines
//-----------------------------------------------------------------------------------------------------------------

void fatal(const char *msg, const byte num) {
   lcd.clear();
   lcd.print("** INTERNAL ERROR **");
   lcd.setCursor(0, 1);
   lcd.print("assertion failed : ");
   lcd.setCursor(0, 2);
   lcd.print(msg);
   while (true) ; }

void assert (boolean test, const char *msg, byte num) {
   if (!test) fatal(msg, num); }

void center_message (byte row, const char *msg) {
   byte len, nblanks;
   len = strlen(msg);
   assert (len <= 20, "big center_message", FATAL_MSG_BIG);
   nblanks = (20 - len) >> 1;
   lcd.setCursor(0, row);
   for (byte i = 0; i < nblanks; ++i) lcd.print(" ");
   lcd.print(msg);
   nblanks = (20 - nblanks) - len;
   for (byte i = 0; i < nblanks; ++i) lcd.print(" "); }

//-----------------------------------------------------------------------------------------------------------------
//  Realtime clock routines
//-----------------------------------------------------------------------------------------------------------------

#define DS1307_CTRL_ID B1101000   //DS1307 realtime clock id

byte bcd2bin (byte val) {
   return val - 6 * (val >> 4); }

byte bin2bcd (byte val) {
   return val + 6 * (val / 10); }

void rtc_read(struct datetime_t *dt) { // Read realtime clock data, in 24-hour mode
   Wire.beginTransmission(DS1307_CTRL_ID);
   Wire.write(0x00);
   Wire.endTransmission();
   // request the 7 bytes of data  (secs, min, hr, wday, date. mth, yr)
   Wire.requestFrom(DS1307_CTRL_ID, 7);
   dt->sec = bcd2bin(Wire.read());
   dt->min = bcd2bin(Wire.read());
   dt->hour = bcd2bin(Wire.read());
   dt->wday = bcd2bin(Wire.read());
   dt->date = bcd2bin(Wire.read());
   dt->month = bcd2bin(Wire.read());
   if (dt->month > 12) dt->month = 12; // careful: we use as subscript
   dt->year = bcd2bin(Wire.read()) + 30; // base is 1970
}
void rtc_write(struct datetime_t *dt) { // Write realtime clock data
   Wire.beginTransmission(DS1307_CTRL_ID);
   Wire.write(0x00); // reset register pointer
   Wire.write(bin2bcd(dt->sec));
   Wire.write(bin2bcd(dt->min));
   Wire.write(bin2bcd(dt->hour)); // 24 hour mode
   Wire.write(bin2bcd(dt->wday));
   Wire.write(bin2bcd(dt->date));
   Wire.write(bin2bcd(dt->month));
   Wire.write(bin2bcd(dt->year - 30)); // base is 1970
   Wire.endTransmission(); }

void show_time(byte row) {  // display formatted time from "timenow"
   char string[25];
   char ampm = 'a';
   byte hour = timenow.hour;
   if (hour > 11) {
      hour -= 12;
      ampm = 'p'; }
   if (hour == 0) hour = 12;
   sprintf(string, "%2d %s 20%02d %2d:%02d %cm",
           timenow.date, months[timenow.month], timenow.year - 30, hour, timenow.min, ampm);
   lcd.setCursor(0, row);
   lcd.print(string); }

void show_current_time (byte row) {  // display current time
   rtc_read(&timenow);
   show_time(row); }

//-----------------------------------------------------------------------------------------------------------------
// Digital potentiometer routines
//-----------------------------------------------------------------------------------------------------------------

#define LOW_TEMP 46
#define HIGH_TEMP 102

static byte temp_to_resistance[HIGH_TEMP - LOW_TEMP + 1]
= { // 0..255 representing digital pots settings for resistance from 0 to 50K ohms
   // See the associated spreadsheet.
   // This was empirically measured by attaching a pot to the thermostat and seeing what it reports,
   // rather than computed from the Steinhart–Hart equation using the thermistor's parameters
   102, 100, 98, 96, 94, 92, 89, 87, 85, 83, 81, 80, 78, 76, 75, 74, 72, 71, 69,
   68, 66, 64, 63, 61, 60, 59, 58, 56, 55, 54, 53, 51, 50, 49, 48, 47, 46, 45,
   44, 44, 43, 42, 41, 40, 39, 38, 37, 37, 36, 35, 35, 34, 34, 33, 32, 31, 29 };

void set_resistor(int addr, byte index) { // set one of the digital pots
   byte chipselect;
   chipselect = addr < 2 ? SPI_CS1 : SPI_CS2; // which chip has the right pair of pots
   SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
   digitalWrite(chipselect, LOW);
   SPI.transfer(addr & 1 ? 0x00 : 0x10);  // address one of the two wipers in that chip
   SPI.transfer(index); // follow with wiper index data
   digitalWrite(chipselect, HIGH);
   SPI.endTransaction(); }

void set_temp(int addr, int temp) { // set the temperature thermistor value for a zone
   if (temp < LOW_TEMP) temp = LOW_TEMP; // (We use 0 to indicate not a valid temp)
   if (temp > HIGH_TEMP) temp = HIGH_TEMP;
   set_resistor(addr, temp_to_resistance[temp - LOW_TEMP]);
   if (DEBUG) {
      char string[40];
      sprintf(string, "set zone %d to temp %dF, pot index %d", addr + 1, temp, temp_to_resistance[temp - LOW_TEMP]);
      Serial.println(string); } }

//-----------------------------------------------------------------------------------------------------------------
//      packet processing
//-----------------------------------------------------------------------------------------------------------------
struct pkt_temp_t rcv_pkt;
struct pkt_resp_t xmt_pkt;

void get_packet(void) {  // receive and process a packet from the remote sensors
   enum RFERR error;

   if (RFradio_gotpkt()) {
      int8_t rssi, snr;
      RFRadio_pktstats(&rssi, &snr); // get signal strength and signal-to-noise ratio
      error = RFradio_getpkt((byte *)&rcv_pkt, RF_TEMP_PKT_SIZE);
      if (error == RF_OK && rcv_pkt.addr <= 3) { // got a well-formed packet
         if (DEBUG) {
            Serial.print(RF_TEMP_PKT_SIZE);
            Serial.print(" pkt bytes : ");
            for (unsigned int i = 0; i < RF_TEMP_PKT_SIZE; ++i) {
               Serial.print(((byte *)(&rcv_pkt))[i], DEC);
               Serial.print(' '); }
            Serial.println(); }
         // We know the the packet is well-formed and has a good node address.
         // Check that all other fields are valid.
         if (rcv_pkt.type != PT_TEMP
               || rcv_pkt.temperature < 32 || rcv_pkt.temperature > 120
               || rcv_pkt.humidity > 100
               || rcv_pkt.battery < 30 || rcv_pkt.battery > 50
               || rcv_pkt.sw_version < 10 || rcv_pkt.sw_version > 99) {
            ++zone[rcv_pkt.addr].bad_packets;  // count a bad packet
         }
         else { // we like everything about this packet
            ++zone[rcv_pkt.addr].good_packets;  // count a good packet
            zone[rcv_pkt.addr].rssi = rssi;  // record signal strength
            zone[rcv_pkt.addr].snr = snr;
            xmt_pkt.addr = rcv_pkt.addr;  // quickly create response packet with the current time
            xmt_pkt.type = PT_RESP; // so that the temp sensor can get back to deep sleep
            rtc_read(&timenow); // (do we need to do this?)
            xmt_pkt.datetime = timenow;   // (structure copy)
            RFradio_transmit((byte *)&xmt_pkt, RF_RESP_PKT_SIZE); // send it
            zone[rcv_pkt.addr].sw_version = rcv_pkt.sw_version;
            if (rcv_pkt.sequence == 0) {  // the sensor just restarted
               zone[rcv_pkt.addr].missed_packets = 0;
               zone[rcv_pkt.addr].last_sequence = 0; }
            else { // the sensor has been running for a while
               if (zone[rcv_pkt.addr].last_sequence != RF_MAX_SEQUENCE // if we've heard from it before
                     && rcv_pkt.sequence > zone[rcv_pkt.addr].last_sequence) // and sequence number has increased
                  zone[rcv_pkt.addr].missed_packets += rcv_pkt.sequence - zone[rcv_pkt.addr].last_sequence - 1; // tally missed packets
               zone[rcv_pkt.addr].last_sequence = rcv_pkt.sequence; }
            zone[rcv_pkt.addr].msec_since_packet = 0;
            if (zone[rcv_pkt.addr].lost_comm   // if we are recovering from lost communications
                  || zone[rcv_pkt.addr].temperature != rcv_pkt.temperature  // or the temp changed
                  || zone[rcv_pkt.addr].humidity != rcv_pkt.humidity // or the humidity changed
                  || zone[rcv_pkt.addr].battery != rcv_pkt.battery) { // or the battery level changed
               set_temp(rcv_pkt.addr, rcv_pkt.temperature);  // record the info
               zone[rcv_pkt.addr].temperature = rcv_pkt.temperature;
               zone[rcv_pkt.addr].humidity = rcv_pkt.humidity;
               zone[rcv_pkt.addr].battery = rcv_pkt.battery;
               refresh_zoneinfo = true; }
            zone[rcv_pkt.addr].lost_comm = false;  // we're communicating
            packet_log[packetlog_index].addr = rcv_pkt.addr;  // make a packet log entry
            packet_log[packetlog_index].temperature = rcv_pkt.temperature;
            packet_log[packetlog_index].battery = rcv_pkt.battery;
            rtc_read(&timenow);
            packet_log[packetlog_index].timestamp = timenow;
            if (packetlog_count < PACKETLOG_SIZE) ++packetlog_count;
            if (++packetlog_index >= PACKETLOG_SIZE) packetlog_index = 0; } }
      else {  //  malformed packet, or from unknown node
         ++malformed_packets;
         if (DEBUG) Serial.println("malformed packet received"); }
      RFradio_receive();  // start the receive for the next packet
   } }

//-----------------------------------------------------------------------------------------------------------------
// Speaker beep
//-----------------------------------------------------------------------------------------------------------------
// This is a simplified version of Paul Stoffregen's "tone" library
// but it drives the piezo-electric speaker differentially for twice the volume.

#define BEEP_FREQUENCY 4000 // Hz

IntervalTimer beep_timer;
unsigned long int interrupt_count;
bool doing_beep = false;

void beep_interrupt(void) {
   if (doing_beep) { // doing a beep?
      if (--interrupt_count) { // more to do: toggle both leads
         digitalWriteFast(SPEAKER_POS, digitalReadFast(SPEAKER_POS) ^ 1);
         digitalWriteFast(SPEAKER_NEG, digitalReadFast(SPEAKER_NEG) ^ 1); }
      else {
         beep_timer.end(); // stop the interrupts
         doing_beep = false; } } }

void speaker_beep (unsigned long int millisec) {
   while (doing_beep) ; // wait for any previous beep to finish
   interrupt_count = (millisec * BEEP_FREQUENCY / 1000) * 2;
   digitalWriteFast(SPEAKER_POS, HIGH); // start with opposite polarity
   digitalWriteFast(SPEAKER_NEG, LOW);
   doing_beep = true;
   assert(beep_timer.begin(beep_interrupt, 1000000 / BEEP_FREQUENCY / 2), "beep timer failed", FATAL_BEEPTIMER); }

//-----------------------------------------------------------------------------------------------------------------
// Initialization
//-----------------------------------------------------------------------------------------------------------------

void setup() {
   char string[25];
   static uint8_t degrees_char[8] = { // the degree symbol for the LCD display
      B11100,
      B10100,
      B11100,
      B00000,
      B00000,
      B00000,
      B00000,
      B00000 };
#define DEGREES 0x01

   #if DEBUG
   delay(1000);
   Serial.begin(115200);
   Serial.println("Receiver started.\n");
   #endif

   lcd.begin(20, 4); // start LCD display
   lcd.createChar(DEGREES, degrees_char);
   lcd.noCursor();
   sprintf(string, "initializing v%s", VERSION);
   center_message(0, string);
   delay(500);

   pinMode(LED, OUTPUT);   // configure miscellaneous I/O pins
   pinMode(SPEAKER_POS, OUTPUT);
   pinMode(SPEAKER_NEG, OUTPUT);
   pinMode(MODESW_1, INPUT_PULLUP);
   pinMode(MODESW_2, INPUT_PULLUP);
   pinMode(MODESW_3, INPUT_PULLUP);
   pinMode(BUTTON_1, INPUT_PULLUP);
   pinMode(BUTTON_2, INPUT_PULLUP);

   for (int i = 0; i < 3; ++i) {  // announce our presence
      speaker_beep(100);
      digitalWrite(LED, HIGH);
      delay(100);
      digitalWrite(LED, LOW);
      delay(100); }

   #if 0
   while (1) {
      for (int i = 3000; i < 5000; i += 100) { // test chirp frequencies
         for (int j = 0; j < 5; ++j) {
            tone(SPEAKER, i, 200);
            lcd.setCursor(0, 0);
            lcd.print(i);
            delay(300);
            if (digitalRead(BUTTON_2) == BUTTON_PUSHED) goto stoptones; } } }
stoptones:
   #endif

   for (int i = 0; i < 4; ++i) { // initialize all four zones
      zone[i].temperature = 0;
      zone[i].humidity = 0;
      zone[i].battery = 0;
      zone[i].lost_comm = true;  // start out assuming no communication
      zone[i].good_packets = 0;
      zone[i].bad_packets = 0;
      zone[i].msec_since_packet = 0;
      zone[i].last_sequence = RF_MAX_SEQUENCE; // "never heard from"
      zone[i].rssi = 0;
      zone[i].snr = 0; }
   zone[0].name = "Entry";
   zone[1].name = " Kitc";
   zone[2].name = " MBR ";
   zone[3].name = "Guest";

   Wire.begin();   // start onewire for temperature sensor access
   rtc_read(&timenow);  // check for valid realtime clock data
   if (timenow.sec > 59  || timenow.min > 59 || timenow.hour > 23
         || timenow.wday  == 0 || timenow.wday > 7 || timenow.date == 0 || timenow.date > 31
         || timenow.month == 0 || timenow.month > 12 || timenow.year > 30 + 30)
      rtc_write (&clock_init); // reset if bad

   digitalWrite(SPI_CS1, HIGH);
   digitalWrite(SPI_CS2, HIGH);
   pinMode(SPI_CS1, OUTPUT);   // digital pot select lines
   pinMode(SPI_CS2, OUTPUT);

   RFradio_init(RF_CS, RF_RESET); // initialize the RF radio
   if (DEBUG) RFradio_printRegisters("RF regs after init : ");

   #if TESTS
   for (int index = 0; index < 256; index += 15) { // 0..255 in 18 steps
      lcd.setCursor(0, 3);
      sprintf(string, "index %3d", index);
      lcd.print(string);
      set_resistor(0, index); // going up
      set_resistor(1, 255 - index); // going down
      set_resistor(2, index);
      set_resistor(3, 255 - index);
      delay(1000); }
   #endif
   for (int i = 0; i < 4; ++i) // set all resistors to simulate 70 degrees F
      set_temp(i, 70);

   lcd.setCursor(0, 0);   // display zone names on the top line
   for (int i = 0; i < 4; ++i) {
      lcd.print(zone[i].name); }

   RFradio_receive(); // start receiving packets
}

//-----------------------------------------------------------------------------------------------------------------
// Process button commands
//-----------------------------------------------------------------------------------------------------------------
//  This is an experiment to see how much we can do with just one button!
//  The UI depends a lot on pushing the button at the right time.

#define BUTTON_TIMEOUT 2*1000  // button push timeout in msec
#define DEBOUNCE_DELAY 50      // button debounce in msec

void wait_for_button_release (byte pin) {
   speaker_beep(100);
   delay (DEBOUNCE_DELAY);
   while (digitalRead(pin) == BUTTON_PUSHED) ; // wait for release
   delay (DEBOUNCE_DELAY); }

bool wait_for_button_push (byte pin) {
   unsigned long msec_start;
   if (digitalRead(pin) == BUTTON_PUSHED) {  // if it is currently pushed
      while (digitalRead(pin) == BUTTON_PUSHED) ; // wait for it to be released first
      delay(DEBOUNCE_DELAY); }
   msec_start = millis();    // start timing
   while (millis() - msec_start < BUTTON_TIMEOUT) {
      if (digitalRead(pin) == BUTTON_PUSHED) { // got a new push
         wait_for_button_release(pin);
         return true; } }
   return false; }

byte dayofweek(int day, int month, int year) {
   // https://www.geeksforgeeks.org/find-day-of-the-week-for-a-given-date/
   static int t[] = { 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };
   year -= month < 3;
   return ( year + year / 4 - year / 100 + year / 400 + t[month - 1] + day) % 7 + 1; }

void set_timedate(void) {   // change the date and time
   show_time(3);
   center_message(2, "Change date");
   while (wait_for_button_push(BUTTON_1)) {
      ++timenow.date; if (timenow.date > 31) timenow.date = 1;
      show_time(3); }
   center_message(2, "Change month");
   while (wait_for_button_push(BUTTON_1)) {
      ++timenow.month; if (timenow.month > 12) timenow.month = 1;
      show_time(3); }
   center_message(2, "Change year");
   while (wait_for_button_push(BUTTON_1)) {
      ++timenow.year; if (timenow.year > 30 + 25) timenow.year = 30 + 15;
      show_time(3); }
   center_message(2, "Change hour");
   while (wait_for_button_push(BUTTON_1)) {
      ++timenow.hour; if (timenow.hour > 23) timenow.hour = 0;
      show_time(3); }
   center_message(2, "Change minute");
   while (wait_for_button_push(BUTTON_1)) {
      ++timenow.min; if (timenow.min > 60) timenow.min = 1;
      show_time(3); }
   timenow.wday = dayofweek(timenow.date, timenow.month, (int)timenow.year + 1970);
   timenow.sec = 0;
   rtc_write(&timenow);
   center_message(2, "Done.");
   show_time(3);
   delay(2000); }

void show_packet_log(void) { // display the event log in reverse chronological order
   int index, count;
   char string[25];
   center_message(2, "");
   index = packetlog_index; // points at oldest event, prior is newest
   for (count = packetlog_count; count > 0; --count) {
      if (--index < 0) index = PACKETLOG_SIZE - 1;
      sprintf(string, "z%u %02u/%02u %02u:%02u:%02u %02u",
              packet_log[index].addr + 1,
              packet_log[index].timestamp.month,
              packet_log[index].timestamp.date,
              packet_log[index].timestamp.hour,
              packet_log[index].timestamp.min,
              packet_log[index].timestamp.sec,
              packet_log[index].temperature);
      if (DEBUG) Serial.println(string);
      center_message(3, string);
      for (int wait = 0; wait < 30; ++wait) { // number of tenths of a second
         delay(100);
         if (digitalRead(BUTTON_1) == BUTTON_PUSHED) { // abort if button pushed
            wait_for_button_release(BUTTON_1);
            return; } }
      center_message(3, ""); // blink between log entries
      delay(100); } }

void show_error_log(void) { // display the error log in reverse chronological order
   int index, count;
   char string[25];
   center_message(2, "");
   if (errorlog_count == 0) {
      center_message(3, "No errors");
      delay(500);
      return; }
   index = errorlog_index; // points at oldest error, prior is newest
   for (count = errorlog_count; count > 0; --count) {
      if (--index < 0) index = ERRORLOG_SIZE - 1;
      sprintf(string, "z%u %02u/%02u %02u:%02u:%02u",
              error_log[index].addr + 1,
              error_log[index].timestamp.month,
              error_log[index].timestamp.date,
              error_log[index].timestamp.hour,
              error_log[index].timestamp.min,
              error_log[index].timestamp.sec);
      if (DEBUG) Serial.println(string);
      center_message(3, string);
      for (int wait = 0; wait < 30; ++wait) { // number of tenths of a second
         delay(100);
         if (digitalRead(BUTTON_1) == BUTTON_PUSHED) { // abort if button pushed
            wait_for_button_release(BUTTON_1);
            return; } }
      center_message(3, ""); // blink between log entries
      delay(100); } }

void show_last_packet_times(void) { // show the time we last received a packet from each zone
   char string[25];
   unsigned int hrs, mins, secs, fsecs;
   unsigned long tm;
   center_message(2, "");
   for (int i = 0; i < 4; ++i) {
      tm = zone[i].msec_since_packet;
      fsecs = tm % 1000;  tm /= 1000;
      secs = tm % 60; tm /= 60;
      mins = tm % 60; hrs = tm / 60;
      sprintf(string, "Z%d: %uh %um %u.%03us", i + 1, hrs, mins, secs, fsecs);
      center_message(3, string);
      if (DEBUG) Serial.println(string);
      delay(3000); } }

void process_commands(void) {  // allow the user to select which command mode to enter
   speaker_beep(100);
   refresh_zoneinfo = true;  // setup to redraw rows 1 and 2
   center_message(2, "Select a command :");
   center_message(3, "Show error log?");
   if (wait_for_button_push(BUTTON_1)) {
      show_error_log();
      return; }
   center_message(3, "Show event log?");
   if (wait_for_button_push(BUTTON_1)) {
      show_packet_log();
      return; }
   center_message(3, "Show last pkt times?");
   if (wait_for_button_push(BUTTON_1)) {
      show_last_packet_times();
      return; }
   center_message(3, "Set time/date?");
   if (wait_for_button_push(BUTTON_1)) {
      set_timedate();
      return; }
   center_message(2, "");
   center_message(3, "Goodbye");
   delay(1000); }

//-----------------------------------------------------------------------------------------------------------------
//          The main loop
//-----------------------------------------------------------------------------------------------------------------

void condensed_count (char *string, unsigned long val) {
   // create a 4-character indication of an arbitrarily large number
   if (val <= 9999UL) sprintf(string, " %4lu", val); // exact for values from 0 to 9999
   else if (val <= 99999UL) sprintf(string, " %1lut%02lu", val / 10000UL, val % 100); // ten thousands, "9t99"
   else if (val <= 999999UL) sprintf(string, " %1luh%02lu", val / 100000UL, val % 100); // hundred thousands, "9h99"
   else if (val <= 9999999UL) sprintf(string, " %1lum%02lu", val / 1000000UL, val % 100); // millions, "9m99"
   else sprintf (string, " %1lub%02lu", val / 10000000L, val % 100); // ten millions, "9b99"
}
unsigned long last_msec = 0, new_msec;
int chirp_delay = 1, blink_delay = 1, info_delay = 1, info_type = 0;
bool blink_on = false;

void loop() {
   char string[20];

   get_packet(); // get and process a possible packet from a remote sensor

   if (digitalRead(BUTTON_1) == BUTTON_PUSHED)  // process button push commands
      process_commands();

   new_msec = millis();
   if (new_msec != last_msec) { // a millisecond has gone by: do our periodic stuff
      last_msec = new_msec;

      for (int i = 0; i < 4; ++i) {  //**** count time in each zone since good packet data
         ++zone[i].msec_since_packet; // (bugette: will overflow after 50 days of no packets)
         if (!zone[i].lost_comm && zone[i].msec_since_packet > LOST_COMM_TIME * 1000) { // if more than xx seconds,
            zone[i].lost_comm = true; // declare communications is broken
            refresh_zoneinfo = true;
            error_log[errorlog_index].addr = i;  // make an error log entry
            rtc_read(&timenow);
            error_log[errorlog_index].timestamp = timenow;
            if (errorlog_count < ERRORLOG_SIZE) ++errorlog_count;
            if (++errorlog_index >= ERRORLOG_SIZE) errorlog_index = 0;
            if (DEBUG) {
               Serial.print("lost comm zone "); Serial.println(i); } } }

      if (refresh_zoneinfo) { //**** optionally redraw zone info on rows 1 and 2
         for (int i = 0; i < 4; ++i) {
            lcd.setCursor(5 * i, 1); // row 1: temperature
            sprintf(string, zone[i].temperature == 0 ? "     "  // have never heard from this zone
                    : zone[i].lost_comm ? "  ?  " : "%3d\x01 " /*DEGREES*/, zone[i].temperature);
            lcd.print(string);
            lcd.setCursor (5 * i, 2); // row 2: humidity, or battery low indication
            sprintf(string, zone[i].battery == 0 ? "     "  // have never heard from this zone
                    // "low battery" should be persistent even if we're not hearing from it now
                    : zone[i].battery < BATTERY_THRESHOLD ? "LoBat"
                    : zone[i].lost_comm ? "     " : "%3d%% ", zone[i].humidity);
            lcd.print(string); }
         refresh_zoneinfo = false; }

      if (--chirp_delay == 0) {   //**** make a low-battery alarm sound
         for (int i = 0; i < 4; ++i)
            if (zone[i].battery != 0 && zone[i].battery < BATTERY_THRESHOLD) { // at least one battery is low
               speaker_beep(300); // chirp for 300 msec
               break; }
         chirp_delay = 10 * 1000; //  seconds between chirps
      }
      if (--blink_delay == 0) { //**** blink the low-battery display
         for (int i = 0; i < 4; ++i) {
            if (zone[i].battery != 0 && zone[i].battery < BATTERY_THRESHOLD) { // flash any "LoBAT"s
               lcd.setCursor(5 * i, 2);
               lcd.print(blink_on ? "     " : "LoBat"); } }
         blink_on = !blink_on;
         blink_delay = 500;  // half second between blink changes
      }
      //**** if button 2 is not pushed, change miscellaneous stuff on the last line every few seconds
      if (digitalRead(BUTTON_2) == BUTTON_NOT_PUSHED && --info_delay == 0) {
         bool any_missed_packets = false;
         bool any_bad_packets = false;
         for (int i = 0; i < 4; ++i) {
            if (zone[i].missed_packets) any_missed_packets = true;
            if (zone[i].bad_packets) any_bad_packets = true; }
         for (int i = 0; i < 4; ++i) { // for each zone
            lcd.setCursor(5 * i, 3); // prepare to write in the spot for this zone, 5 chars max
#define HIGHEST_INFO_TYPE 9 // don't show software version any more
            switch (info_type) {
               case 0: //**** do several cycles of the current time
               case 1:
               case 2:
               case 3:
                  if (i == 0) show_current_time(3);
                  break;
               case 4: //**** show count of good packets
                  condensed_count(string, zone[i].good_packets); // create 4-char abbreviated count
                  lcd.print(string);
                  break;
               case 5: //**** show count of missed packets, if any
                  if (any_missed_packets) {
                     sprintf(string, zone[i].missed_packets == 0 ? "   -0" : "%5d",
                             -min((int)zone[i].missed_packets, 9999));
                     lcd.print(string);
                     break; }
                  ++info_type; // otherwise skip to next
               case 6: //**** show count of bad packets, if any
                  if (any_bad_packets) {
                     sprintf(string, zone[i].bad_packets == 0 ? "   x0" :
                             zone[i].bad_packets <   10 ? "   x%d" :
                             zone[i].bad_packets <  100 ? "  x%d" :
                             zone[i].bad_packets < 1000 ? " x%d" : "x%d",
                             min((int)zone[i].bad_packets, 9999));
                     lcd.print(string);
                     break; }
                  ++info_type; // otherwise skip to next
               case 7:  //**** show count of malformed or unidentified packets
                  if (malformed_packets > 0) {
                     if (i == 0) {
                        sprintf(string, "malformed: %ld", malformed_packets);
                        center_message(3, string); }
                     break; }
                  ++info_type; // otherwise skip to next
               case 8: //**** show signal strength in dB
                  if (zone[i].rssi != 0)
                     sprintf(string, "%3ddB", zone[i].rssi);
                  else sprintf(string, "     ");
                  lcd.print(string);
                  break;
               case 9: //**** show battery voltage
                  if (zone[i].battery != 0)
                     sprintf(string, " %d.%dV", zone[i].battery / 10, zone[i].battery % 10);
                  else sprintf(string, "     ");
                  lcd.print(string);
                  break;
               case 10: // show software version number, maybe
                  sprintf(string, " v%d.%d", zone[i].sw_version / 10, zone[i].sw_version % 10);
                  lcd.print(string);
                  break; } }
         if (++info_type > HIGHEST_INFO_TYPE) info_type = 0; // cycle around
         info_delay = 1 * 1500;  // change every 1.5 seconds
      } } }

