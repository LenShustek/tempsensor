/**************************************************************************************

    remote temperature receiver

  This software is inside a box that receives wireless data from four remote
  temperature sensors. We replaces the thermistors in Mitsubishi PAR-21MAA
  controller/thermostats with a digitally-generated resistance that reflects
  the temperature at the remote locations.

  The custom-built hardware contains the following:
  - Teensy 3.1 microcontroller from www.pjrc.com
  - Seeed HC-12 433 Mhz RF transceiver from www.seeedstudio.com
  - four Microchip MCP4251-series 50K digital potentiometers
  - 4-line by 20-character LCD display
  - Chronodot realtime clock with DS3231 and CR1632 3V battery
    from http://docs.macetech.com/doku.php/chronodot_v2.0
  - a piezo-electric buzzer for making low-battery alarm chirps
  - two pushbutton switches (one is currently unused)
  - four mode-setting slide switches (all are currently unused)
  - Chronodot realtime clock with battery backup

  The display is used as follows:
    line 1:  titles of the four zones
    line 2:  current zone temperatures
    line 3:  remote sensor battery statuses: "ok" or "LoBat"
    line 4:  the current date and time, alternating with various
             statistical and debugging information

  The design of the hardware and software for this receiver and the corresponding
  sensors is open-source.  See https://github.com/LenShustek/tempsensor.

  -------------------------------------------------------------------------------
    (C) Copyright 2015, Len Shustek

    This program is free software: you can redistribute it and/or modify
    it under the terms of version 3 of the GNU General Public License as
    published by the Free Software Foundation at http://www.gnu.org/licenses,
    with Additional Permissions under term 7(b) that the original copyright
    notice and author attibution must be preserved, and under term 7(c) that
    modified versions be marked as different from the original.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
  -------------------------------------------------------------------------------
  Change log

  11 Nov 2015, V1.0, L. Shustek; first version
  29 Nov 2015, V1.1, L. Shustek; add pushbutton commands
  14 Dec 2015, V1.2, L. Shustek; renumber to zones 1..4

*/
#define VERSION "1.2"

#define DEBUG 1
#define TESTS 0
#define REALTIME_CLOCK 1

#include <arduino.h>
#include <LiquidCrystal.h>  // for LCD display
#include <SPI.h>    // for digital pots using SPI
#include <Wire.h>   // for realtime clock using I2C
#include "IntervalTimer.h"

#define SPEAKER_POS 20 // speaker pins
#define SPEAKER_NEG 21

#define HC12_SET 4    // RF transceiver "set" pin
#define MODESW_1 22   // mode slide switch pins
#define MODESW_2 23
#define MODESW_3 16
#define MODESW_4 17
#define BUTTON_1 2    // action pushbutton pins; normally closed
#define BUTTON_2 3

LiquidCrystal lcd(/*RS*/ 5, /*E*/ 6, /*D4-D7*/ 7, 8, 9, 10);  // LCD pins

#define SPI_MOSI 11  // MicroChip MCP4251 digital potentiometer pins
#define SPI_MISO 12
#define SPI_CLK 13
#define LED 13  // (also connected to LED)
#define SPI_CS1 14  // first two pots, IC U$2
#define SPI_CS2 15  // second two pots, IC U$4

struct {  // the packet we expect to receive
  byte stx;         // STX=0x02
  byte addr;        // our address, 0...3, mapped to "zones" 1..4
  byte type;        // this message type: 1
  byte temperature; // temperature in degrees F, 1 to 120
  byte humidity;    // humidity in percent, 0..100
  byte battery;     // battery voltage*10
  byte etx;         // ETX=0x03
  byte lrc;         // XOR of stx through etx
} packet;
#define STX 0x02
#define ETX 0x03
unsigned long malformed_packets = 0;

struct { // current information about each zone
  const char *name;
  byte temperature;
  byte battery;
  bool lost_comm; // did we lose communication?
  unsigned long good_packets;
  unsigned long bad_packets;
  unsigned long msec_since_packet;
} zone[4];
#define BATTERY_THRESHOLD 40  // minimum good battery level in tenths of a volt
bool refresh_zoneinfo = false;  // do we need to refresh zone info display on rows 1 and 2?

#define DS1307_CTRL_ID B1101000  //DS1307 realtime clock id
struct datetime {
  byte sec, min, hour /*1-12*/, ampm, day/*1-7*/, date, month, year;
} now,
clock_init = { // (when we first wrote the code)
  50, 10, 8, 1, 4, 26, 11, 15
};

static const char *months[] = { // index 1..12 from realtime clock
  "???", "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};
static const byte days_in_month []  = {
  99, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};

#define PACKETLOG_SIZE 100
struct { // the log of packets we record for debugging
  byte addr; // 0..3
  byte temperature;
  byte battery;
  struct datetime timestamp;
} packet_log[PACKETLOG_SIZE];
int packetlog_count = 0; // number of events stored
int packetlog_index = 0; // next log array location to write

#define ERRORLOG_SIZE 25
struct { // the log of errors we record for debugging
  byte addr;   // currently we only store "disconnect" events
  struct datetime timestamp;
} error_log[ERRORLOG_SIZE];
int errorlog_count = 0; // number of errors stored
int errorlog_index = 0; // next log array location to write

//-----------------------------------------------------------------------------------------------------------------
// display routines
//-----------------------------------------------------------------------------------------------------------------

void assert (boolean test, const char *msg) {
  if (!test) {
    lcd.clear();
    lcd.print("** INTERNAL ERROR **");
    lcd.setCursor(0, 1);
    lcd.print("assertion failed:");
    lcd.setCursor(0, 2);
    lcd.print(msg);
    while (true) ;
  }
}
void center_message (byte row, char *msg) {
  byte len, nblanks;
  len = strlen(msg);
  assert (len <= 20, "big center_message");
  nblanks = (20 - len) >> 1;
  lcd.setCursor(0, row);
  for (byte i = 0; i < nblanks; ++i) lcd.print(" ");
  lcd.print(msg);
  nblanks = (20 - nblanks) - len;
  for (byte i = 0; i < nblanks; ++i) lcd.print(" ");
}

//-----------------------------------------------------------------------------------------------------------------
//  Realtime clock routines
//-----------------------------------------------------------------------------------------------------------------

byte bcd2bin (byte val) {
  return val - 6 * (val >> 4);
}
byte bin2bcd (byte val) {
  return val + 6 * (val / 10);
}
void rtc_read(struct datetime *dt) { // Read realtime clock data, in 12-hour mode
  byte hour;
  Wire.beginTransmission(DS1307_CTRL_ID);
  Wire.write(0x00);
  Wire.endTransmission();
  // request the 7 bytes of data  (secs, min, hr, day, date. mth, yr)
  Wire.requestFrom(DS1307_CTRL_ID, 7);
  dt->sec = bcd2bin(Wire.read());
  dt->min = bcd2bin(Wire.read());
  hour = Wire.read();
  dt->hour = bcd2bin(hour & 0x1f);
  dt->ampm = (hour >> 5) & 1; // 0=AM, 1=PM
  dt->day = bcd2bin(Wire.read());
  dt->date = bcd2bin(Wire.read());
  dt->month = bcd2bin(Wire.read());
  if (dt->month > 12) dt->month = 12; // careful: we use as subscript
  dt->year = bcd2bin(Wire.read());
}
void rtc_write(struct datetime *dt) { // Write realtime clock data
  Wire.beginTransmission(DS1307_CTRL_ID);
  Wire.write(0x00); // reset register pointer
  Wire.write(bin2bcd(dt->sec));
  Wire.write(bin2bcd(dt->min));
  Wire.write(bin2bcd(dt->hour) | ((dt->ampm) << 5) | 0x40); // 12 hour mode
  Wire.write(bin2bcd(dt->day));
  Wire.write(bin2bcd(dt->date));
  Wire.write(bin2bcd(dt->month));
  Wire.write(bin2bcd(dt->year));
  Wire.endTransmission();
}
void show_time(byte row) {  // display formatted time
  char string[25];
  sprintf(string, "%2d %s 20%02d %2d:%02d %s",
          now.date, months[now.month], now.year, now.hour, now.min, now.ampm ? "PM" : "AM");
  lcd.setCursor(0, row);
  lcd.print(string);
}
void show_current_time (byte row) {  // display current time
  if (REALTIME_CLOCK) rtc_read(&now);
  show_time(row);
}

//-----------------------------------------------------------------------------------------------------------------
// HC-12 433 Mhz wireless RF transceiver routines
//-----------------------------------------------------------------------------------------------------------------

void send_command(const char *cmd) {  // Sent control command to the transceiver
  char response[100];
  byte response_length;
  Serial1.println(cmd);
  if (DEBUG) {
    Serial.print("cmd: ");
    Serial.println(cmd);
  }
  Serial1.flush();  // wait for all bytes to be sent
  Serial1.setTimeout(100); // wait up to 100 msec for a response
  response_length = Serial1.readBytes(response, sizeof(response) - 1);
  response[response_length] = '\0';
  if (DEBUG) {
    Serial.print("rsp: ");
    Serial.println(response);
  }
}
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
  44, 44, 43, 42, 41, 40, 39, 38, 37, 37, 36, 35, 35, 34, 34, 33, 32, 31, 29
};
void set_resistor(int addr, byte index) { // set one of the digital pots
  byte chipselect;
  chipselect = addr < 2 ? SPI_CS1 : SPI_CS2; // which chip has the right pair of pots
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  digitalWrite(chipselect, LOW);
  SPI.transfer(addr & 1 ? 0x00 : 0x10);  // address one of the two wipers in that chip
  SPI.transfer(index); // follow with wiper index data
  digitalWrite(chipselect, HIGH);
  SPI.endTransaction();
}
void set_temp(int addr, int temp) { // set the temperature thermistor value for a zone
  if (temp < LOW_TEMP) temp = LOW_TEMP; // (We use 0 to indicate not a valid temp)
  if (temp > HIGH_TEMP) temp = HIGH_TEMP;
  set_resistor(addr, temp_to_resistance[temp - LOW_TEMP]);
  if (DEBUG) {
    char string[40];
    sprintf(string, "set zone %d to temp %dF, pot index %d", addr+1, temp, temp_to_resistance[temp - LOW_TEMP]);
    Serial.println(string);
  }
}

//-----------------------------------------------------------------------------------------------------------------
//      packet processing
//-----------------------------------------------------------------------------------------------------------------

void get_packet(void) {  // receive and process a packet from the remote sensors
  byte lrc;
  int numbytes;
  char string[20];
  if ((numbytes = Serial1.available()) > 0) { // there are some bytes to read
    if (DEBUG) {
      Serial.print("bytes available: ");
      Serial.println(numbytes);
    }
    Serial1.setTimeout(500); // msec timeout for a packet (should take 8 at 9600 baud)
    numbytes = Serial1.readBytes((byte *)&packet, sizeof(packet));
    if (DEBUG) {
      Serial.print(numbytes);
      Serial.print(" pkt bytes: ");
      for (int i = 0; i < numbytes; ++i) {
        Serial.print(((byte *)(&packet))[i], DEC);
        Serial.print(' ');
      }
      Serial.println();
    }
    if (numbytes == sizeof(packet)   // got a full packet
        && packet.stx == STX && packet.etx == ETX  // with STX and ETX
        && packet.addr < 4) {  // and a good addr number
      lrc = 0;
      for (unsigned i = 0; i < sizeof(packet) - 1; ++i) lrc ^= ((byte *)(&packet))[i];
      if (lrc != packet.lrc) {
        ++zone[packet.addr].bad_packets;  // count a bad packet
      }
      else {  // record data from a good packet
        ++zone[packet.addr].good_packets;  // count a good packet
        zone[packet.addr].msec_since_packet = 0;
        if (zone[packet.addr].lost_comm   // if we are recovering from lost communications
            || zone[packet.addr].temperature != packet.temperature  // or the temp changed
            || zone[packet.addr].battery != packet.battery) { // or the battery level changed
          set_temp(packet.addr, packet.temperature);  // record the info
          zone[packet.addr].temperature = packet.temperature;
          zone[packet.addr].battery = packet.battery;
          refresh_zoneinfo = true;
        }
        zone[packet.addr].lost_comm = false;  // we're communicating
        packet_log[packetlog_index].addr = packet.addr;  // make a packet log entry
        packet_log[packetlog_index].temperature = packet.temperature;
        packet_log[packetlog_index].battery = packet.battery;
        if (REALTIME_CLOCK) rtc_read(&now);
        packet_log[packetlog_index].timestamp = now;
        if (packetlog_count < PACKETLOG_SIZE) ++packetlog_count;
        if (++packetlog_index >= PACKETLOG_SIZE) packetlog_index = 0;
      }
    }
    else {  // packet from unidentified zone
      ++malformed_packets;
      if (DEBUG) Serial.println("bad size packet received");
    }
  }
}

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
      digitalWriteFast(SPEAKER_NEG, digitalReadFast(SPEAKER_NEG) ^ 1);
    } else {
      beep_timer.end(); // stop the interrupts
      doing_beep = false;
    }
  }
}
void speaker_beep (unsigned long int millisec) {
  while (doing_beep) ; // wait for any previous beep to finish
  interrupt_count = (millisec * BEEP_FREQUENCY / 1000) * 2;
  digitalWriteFast(SPEAKER_POS, HIGH); // start with opposite polarity
  digitalWriteFast(SPEAKER_NEG, LOW);
  doing_beep = true;
  assert(beep_timer.begin(beep_interrupt, 1000000 / BEEP_FREQUENCY / 2), "beep timer failed");
}

//-----------------------------------------------------------------------------------------------------------------
// Initialization
//-----------------------------------------------------------------------------------------------------------------

void setup() {
  char string[80];

#if DEBUG
  delay(1000);
  Serial.begin(115200);
  Serial.println("Receiver started.\n");
#endif

  lcd.begin(20, 4); // start LCD display
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
  pinMode(MODESW_4, INPUT_PULLUP);
  pinMode(BUTTON_1, INPUT_PULLUP);
  pinMode(BUTTON_2, INPUT_PULLUP);

  for (int i = 0; i < 3; ++i) {  // announce our presence
    speaker_beep(100);
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);
    delay(100);
  }

#if 0
  while (1) {
    for (int i = 3000; i < 5000; i += 100) { // test chirp frequencies
      for (int j = 0; j < 5; ++j) {
        tone(SPEAKER, i, 200);
        lcd.setCursor(0, 0);
        lcd.print(i);
        delay(300);
        if (digitalRead(BUTTON_2) == HIGH) goto stoptones;
      }
    }
  }
stoptones:
#endif

  for (int i = 0; i < 4; ++i) { // initialize all four zones
    zone[i].temperature = 0;
    zone[i].battery = 0;
    zone[i].lost_comm = true;  // start out assuming no communication
    zone[i].good_packets = 0;
    zone[i].bad_packets = 0;
    zone[i].msec_since_packet = 0;
  }
  zone[0].name = "Entry";
  zone[1].name = " Kitc";
  zone[2].name = " MBR ";
  zone[3].name = "Guest";

#if REALTIME_CLOCK
  Wire.begin();   // start onewire for temperature sensor access
  rtc_read(&now);  // check for valid realtime clock data
  if (now.sec > 59  || now.min > 59 || now.hour == 0 || now.hour > 12
      || now.day  == 0 || now.day > 7 || now.date == 0 || now.date > 31
      || now.month == 0 || now.month > 12 || now.year > 99)
    rtc_write (&clock_init); // reset if bad
#else
  now = clock_init;
#endif

  pinMode(HC12_SET, OUTPUT);  // start serial port to HC-12 RF transceiver
  digitalWriteFast(HC12_SET, HIGH);
  Serial1.begin(9600, SERIAL_8N1);
  digitalWriteFast(HC12_SET, LOW); // put HC-12 in command mode
  send_command("AT");     // wake up the transceiver
  send_command("AT+FU1"); // set FU1 mode: medium speed, 3.6 ma
  send_command("AT+C001"); // set channel 001: 433.4 Mhz
  if (DEBUG) {
    send_command("AT+V");  // get version number
    send_command("AT+RX"); // get all parameters
  }
  digitalWriteFast(HC12_SET, HIGH); // put HC-12 in data mode
  delay(200); // wait a bit

  pinMode(SPI_CS1, OUTPUT);   // start SPI bus to control digital potentiometers
  pinMode(SPI_CS2, OUTPUT);
  digitalWrite(SPI_CS1, HIGH);
  digitalWrite(SPI_CS2, HIGH);
  SPI.begin();
#if TESTS
  for (int index = 0; index < 256; index += 15) { // 0..255 in 18 steps
    lcd.setCursor(0, 3);
    sprintf(string, "index %3d", index);
    lcd.print(string);
    set_resistor(0, index); // going up
    set_resistor(1, 255 - index); // going down
    set_resistor(2, index);
    set_resistor(3, 255 - index);
    delay(1000);
  }
#endif
  for (int i = 0; i < 4; ++i) // set all resistors to simulate 70 degrees F
    set_temp(i, 70);

  lcd.setCursor(0, 0);   // display zone names on the top line
  for (int i = 0; i < 4; ++i) {
    lcd.print(zone[i].name);
  }
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
  while (digitalRead(pin) == 1) ; // wait for release
  delay (DEBOUNCE_DELAY);
}
bool wait_for_button_push (byte pin) {
  unsigned long msec_start;
  if (digitalRead(pin) == 1) {  // if it is currently pushed
    while (digitalRead(pin) == 1) ; // wait for it to be released first
    delay(DEBOUNCE_DELAY);
  }
  msec_start = millis();    // start timing
  while (millis() - msec_start < BUTTON_TIMEOUT) {
    if (digitalRead(pin) == 1) { // got a new push
      wait_for_button_release(pin);
      return true;
    }
  }
  return false;
}
void set_timedate(void) {   // change the date and time
  show_time(3);
  center_message(2, "Change date");
  while (wait_for_button_push(BUTTON_1)) {
    ++now.date; if (now.date > 31) now.date = 1;
    show_time(3);
  }
  center_message(2, "Change month");
  while (wait_for_button_push(BUTTON_1)) {
    ++now.month; if (now.month > 12) now.month = 1;
    show_time(3);
  }
  center_message(2, "Change year");
  while (wait_for_button_push(BUTTON_1)) {
    ++now.year; if (now.year > 25) now.year = 15;
    show_time(3);
  }
  center_message(2, "Change hour");
  while (wait_for_button_push(BUTTON_1)) {
    ++now.hour; if (now.hour > 12) now.hour = 1;
    show_time(3);
  }
  center_message(2, "Change minute");
  while (wait_for_button_push(BUTTON_1)) {
    ++now.min; if (now.min > 60) now.min = 1;
    show_time(3);
  }
  center_message(2, "Change AM/PM");
  while (wait_for_button_push(BUTTON_1)) {
    now.ampm ^= 1;
    show_time(3);
  }
  rtc_write(&now);
  center_message(2, "Done.");
  show_time(3);
  delay(2000);
}
void show_packet_log(void) { // display the event log in reverse chronological order
  int index, count;
  char string[25];
  center_message(2, "");
  index = packetlog_index; // points at oldest event, prior is newest
  for (count = packetlog_count; count > 0; --count) {
    if (--index < 0) index = PACKETLOG_SIZE - 1;
    sprintf(string, "z%u %02u/%02u %02u:%02u:%02u %02u",
            packet_log[index].addr+1,
            packet_log[index].timestamp.month,
            packet_log[index].timestamp.date,
            packet_log[index].timestamp.hour + (packet_log[index].timestamp.ampm ? 12 : 0),
            packet_log[index].timestamp.min,
            packet_log[index].timestamp.sec,
            packet_log[index].temperature);
    if (DEBUG) Serial.println(string);
    center_message(3, string);
    for (int wait = 0; wait < 30; ++wait) { // number of tenths of a second
      delay(100);
      if (digitalRead(BUTTON_1) == 1) { // abort if button pushed
        wait_for_button_release(BUTTON_1);
        return;
      }
    }
    center_message(3, ""); // blink between log entries
    delay(100);
  }
}
void show_error_log(void) { // display the error log in reverse chronological order
  int index, count;
  char string[25];
  center_message(2, "");
  if (errorlog_count == 0) {
    center_message(3, "No errors");
    delay(500);
    return;
  }
  index = errorlog_index; // points at oldest error, prior is newest
  for (count = errorlog_count; count > 0; --count) {
    if (--index < 0) index = ERRORLOG_SIZE - 1;
    sprintf(string, "z%u %02u/%02u %02u:%02u:%02u",
            error_log[index].addr+1,
            error_log[index].timestamp.month,
            error_log[index].timestamp.date,
            error_log[index].timestamp.hour + (error_log[index].timestamp.ampm ? 12 : 0),
            error_log[index].timestamp.min,
            error_log[index].timestamp.sec);
    if (DEBUG) Serial.println(string);
    center_message(3, string);
    for (int wait = 0; wait < 30; ++wait) { // number of tenths of a second
      delay(100);
      if (digitalRead(BUTTON_1) == 1) { // abort if button pushed
        wait_for_button_release(BUTTON_1);
        return;
      }
    }
    center_message(3, ""); // blink between log entries
    delay(100);
  }
}
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
    sprintf(string, "Z%d: %uh %um %u.%03us", i+1, hrs, mins, secs, fsecs);
    center_message(3, string);
    if (DEBUG) Serial.println(string);
    delay(3000);
  }
}
void process_commands(void) {  // allow the user to select which command mode to enter
  speaker_beep(100);
  refresh_zoneinfo = true;  // setup to redraw rows 1 and 2
  center_message(2, "Select a command:");

  center_message(3, "Show error log?");
  if (wait_for_button_push(BUTTON_1)) {
    show_error_log();
    return;
  }
  center_message(3, "Show event log?");
  if (wait_for_button_push(BUTTON_1)) {
    show_packet_log();
    return;
  }
  center_message(3, "Show last pkt times?");
  if (wait_for_button_push(BUTTON_1)) {
    show_last_packet_times();
    return;
  }
  center_message(3, "Set time/date?");
  if (wait_for_button_push(BUTTON_1)) {
    set_timedate();
    return;
  }
  center_message(2, "");
  center_message(3, "Goodbye");
  delay(1000);
}

//-----------------------------------------------------------------------------------------------------------------
//          The main loop
//-----------------------------------------------------------------------------------------------------------------

unsigned long last_msec = 0, new_msec;
int chirp_delay = 1, blink_delay = 1, info_delay = 1, info_type = 0;
bool blink_on = false;

void loop() {
  char string[20];

  get_packet(); // get and process a packet from a remote sensor

  if (digitalRead(BUTTON_1))  // process button push commands
    process_commands();

  new_msec = millis();
  if (new_msec != last_msec) { // a millisecond has gone by: do our periodic stuff
    last_msec = new_msec;

    for (int i = 0; i < 4; ++i) {  //**** count time in each zone since good packet data
      ++zone[i].msec_since_packet; // (bug: will overflow after 50 days of no packets)
      if (!zone[i].lost_comm && zone[i].msec_since_packet > 60 * 1000) { // if more than 60 seconds,
        zone[i].lost_comm = true; // declare communications is broken
        refresh_zoneinfo = true;
        error_log[errorlog_index].addr = i;  // make an error log entry
        if (REALTIME_CLOCK) rtc_read(&now);
        error_log[errorlog_index].timestamp = now;
        if (errorlog_count < ERRORLOG_SIZE) ++errorlog_count;
        if (++errorlog_index >= ERRORLOG_SIZE) errorlog_index = 0;
        if (DEBUG) {
          Serial.print("lost comm zone "); Serial.println(i);
        }
      }
    }
    if (refresh_zoneinfo) { //**** optionally redraw zone info on rows 1 and 2
      for (int i = 0; i < 4; ++i) {
        lcd.setCursor(5 * i, 1);
        sprintf(string, zone[i].temperature == 0 ? "     "  // have never heard from this zone
                : zone[i].lost_comm ? "  ?  " : " %3d ", zone[i].temperature);
        lcd.print(string);
        lcd.setCursor (5 * i, 2);
        lcd.print(zone[i].battery == 0 ? "     "  // have never heard from this zone
                  // "low battery" should be persistent even if we're not hearing from it now
                  : zone[i].battery < BATTERY_THRESHOLD ? "LoBat"
                  : zone[i].lost_comm ? "     " : "  Ok "  );
      }
      refresh_zoneinfo = false;
    }
    if (--chirp_delay == 0) {   //**** make a low-battery alarm sound
      for (int i = 0; i < 4; ++i)
        if (zone[i].battery != 0 && zone[i].battery < BATTERY_THRESHOLD) { // at least one battery is low
          speaker_beep(300); // chirp for 300 msec
          break;
        }
      chirp_delay = 3 * 1000; //  seconds between chirps
    }
    if (--blink_delay == 0) { //**** blink the low-battery display
      for (int i = 0; i < 4; ++i) {
        if (zone[i].battery != 0 && zone[i].battery < BATTERY_THRESHOLD) { // flash any "LoBAT"s
          lcd.setCursor(5 * i, 2);
          lcd.print(blink_on ? "     " : "LoBat");
        }
      }
      blink_on = !blink_on;
      blink_delay = 500;  // half second between blink changes
    }
    if (--info_delay == 0) {   //**** show miscellaneous stuff on the last line
      bool any_bad_packets = false;
      for (int i = 0; i < 4; ++i) if (zone[i].bad_packets) any_bad_packets = true;
      for (int i = 0; i < 4; ++i) { // for each zone
        lcd.setCursor(5 * i, 3); // prepare to write in the spot for this zone
        switch (info_type) {
          case 0: // several cycles of current time
          case 1:
          case 2:
          case 3:
            if (i == 0) show_current_time(3);
            break;
          case 4: // count of good packets for each zone
            sprintf(string, "%5lu", zone[i].good_packets);
            lcd.print(string);
            break;
          case 5: // count of bad packets for each zone
            if (any_bad_packets) {
              sprintf(string, zone[i].bad_packets == 0 ? "   -0" : "%5d",  -(int)zone[i].bad_packets);
              lcd.print(string);
              break;
            }
            ++info_type; // otherwise skip
          case 6:  // count of bad-length (unidentified) packets for all zones
            if (malformed_packets > 0) {
              if (i == 0) {
                sprintf(string, "bad pkts: %-8ld", malformed_packets);
                lcd.print(string);
              }
              break;
            }
            ++info_type; // otherwise skip
          case 7: // battery voltage for each zone
            if (zone[i].battery != 0)
              sprintf(string, " %d.%dV", zone[i].battery / 10, zone[i].battery % 10);
            else sprintf(string, "     ");
            lcd.print(string);
            break;
        }
      }
      if (++info_type > 7) info_type = 0; // cycle around
      info_delay = 1 * 1000;  // change every second
    }
  }
}

