/**************************************************************************************

    Battery-powered remote wireless temperature sensor

  This is the software for a small (3" x 3" x 1") battery-powered wireless sensor
  that measure temperature (and humidity), displays it on a 2-digit LCD screen,
  and periodically sends it to a receiver that digitally simulates the resistance
  of a thermistor inside a Mitsubishi thermostate at the temperature the sensor.

  The custom-built hardware consist of:
  - Teensy LC microcontroller from www.pjrc.com
  - Seeed HC-12 433 Mhz RF transceiver from www.seeedstudio.com
  - Varitronix VI-201 2-digit 7-segment LCD display
  - TI HDC1008 I2C digital temperature/humidity sensor, from www.adafruit.com
  - Linear LT1004 2.5V micropower voltage reference to measure the battery
  - 4 slide switches to set the sensor address and mode
  - 3-cell AAA battery holder

  Since we're battery-powered, we try hard to minimize power utilization
  by using the various sleep/hiberate modes of the CPU and the RF transmitter.
  It looks like we average about 100 uA, which means that standard 1000 mAh
  alkaline AAA batteries should last about a year. We'll see.

  The design of the hardware and software for the sensors and the corresponding
  receiver is open-source.  See https://github.com/LenShustek/tempsensor.

  --------------------------------------------------------------------------
    (C) Copyright 2015, Len Shustek

    This program is free software: you can redistribute it and/or modify
    it under the terms of version 3 of the GNU General Public License as
    published by the Free Software Foundation at http://www.gnu.org/licenses,
    with Additional Permissions under term 7(b) that the original copyright
    notice and author attibution must be preserved and under term 7(c) that
    modified versions be marked as different from the original.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
  --------------------------------------------------------------------------
  Change log

  11 Nov 2015, V1.0, L. Shustek, First version
  30 Nov 2015, V1.1, L. Shustek, add retry for HC-12 errors

*************************************************************************************/

#define DEBUG 0

#define DEEP_SLEEP 1
#define HIBERNATE 1
#define TESTS 0

#include <arduino.h>
#include <Snooze.h>
#include <Wire.h>


SnoozeBlock sleep_config;

// the packet we send

struct {
  byte stx;         // STX=0x02
  byte addr;        // our address
  byte type;        // this message type: 1
  byte temperature; // temperature in degrees F, 0 to 120
  byte humidity;    // humidity in percent, 0..100
  byte battery;     // battery voltage*10
  byte etx;         // ETX=0x03
  byte lrc;         // XOR of stx through etx
} packet;
#define STX 0x02
#define ETX 0x03
unsigned long count_sent_packets = 0;

// battery voltage test stuff

#define BATTERY_PIN A12  // the pin where battery voltage/2 appears
#define AREF_MV 2500     // the analog reference voltage in millivolts

// LCD 2-digit 7-segment display: Varitronix VI-201

#define LCD_DIGIT1_SEGMENTS GPIOC_PDOR  // all of register D are digit 1 segments
#define LCD_DIGIT2_SEGMENTS GPIOD_PDOR  // all of register C are digit 2 segments
#define LCD_DIGIT_COM  3                // common for both digits

byte lcd_pins[] = {
  2, 14, 7, 8, 6, 20, 21, 5,
  15, 22, 23, 9, 10, 13, 11, 12,
  LCD_DIGIT_COM
}; // note that pin 13 (PTC5) is attached to a LED on the Teensy board that should be removed to save power

byte digit_segments[11] = { // bit map of segments to light up, with A as LSB, for 0-9 and 'E'
  0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x79
};
#define POINT1 1
#define POINT2 2
#define NOPOINTS 0

// address and mode slide switches

#define SW1 16
#define SW2 17  // (something's wrong with this -- always zero???)
#define SW3 24
#define SW4 25

// temperature/humidity sensor: Adafruit's breakout board for the TI HDC1008

#define HDC1000_I2CADDR       0x40
#define HDC1000_TEMP          0x00
#define HDC1000_HUMID         0x01
#define HDC1000_CONFIG        0x02
#define HDC1000_CONFIG_RST    (1 << 15)
#define HDC1000_CONFIG_HEAT   (1 << 13)
#define HDC1000_CONFIG_MODE   (1 << 12)
#define HDC1000_CONFIG_BATT   (1 << 11)
#define HDC1000_CONFIG_TRES_14  0
#define HDC1000_CONFIG_TRES_11  (1 << 10)
#define HDC1000_CONFIG_HRES_14  0
#define HDC1000_CONFIG_HRES_11  (1 << 8)
#define HDC1000_CONFIG_HRES_8   (1 << 9)

#define HDC1000_SERIAL1       0xFB
#define HDC1000_SERIAL2       0xFC
#define HDC1000_SERIAL3       0xFD
#define HDC1000_MANUFID       0xFE
#define HDC1000_DEVICEID      0xFF

uint16_t current_temp;    // degrees F
uint16_t current_humid;   // percent, 0..100

// processoer sleep modes, using the "Snooze" library at https://github.com/duff2013/Snooze

#if DEEP_SLEEP && !DEBUG  // use this while RF transmitter is alive
void sleep_delay(int milliseconds) { // delay in deep sleep; power consumption is about 230 uA
  sleep_config.setTimer(milliseconds);
  Snooze.deepSleep(sleep_config);
}
#else
#define sleep_delay delay   // (going into sleep mode kills the USB connection to the monitor window)
#endif

#if HIBERNATE && !DEBUG  // ok to use when the RF transmitter is sleeping
void hibernate_delay(int milliseconds) { // delay in hibernation; power consumption is about 10 uA
  sleep_config.setTimer(milliseconds);
  Snooze.hibernate(sleep_config);
}
#else
#define hibernate_delay sleep_delay
#endif

//-----------------------------------------------------------------------------------------------------------------
//  Temperature/humidity sensor routines
//  The TI HDC1008 communicates using the I2C protocol (SCL/SDA)
//-----------------------------------------------------------------------------------------------------------------

void HDC1000_reset (void) { // reset, and select 14 bit temp & humidity
  uint16_t config = HDC1000_CONFIG_RST | HDC1000_CONFIG_MODE | HDC1000_CONFIG_TRES_14 | HDC1000_CONFIG_HRES_14;
  Wire.beginTransmission(HDC1000_I2CADDR);
  Wire.write(HDC1000_CONFIG); // was missing from Adafruit library???
  Wire.write(config >> 8);
  Wire.write(config & 0xFF);
  Wire.endTransmission();
  hibernate_delay(15);
}
uint16_t HDC1000_read16(uint8_t a) {
  Wire.beginTransmission(HDC1000_I2CADDR);
  Wire.write(a);
  Wire.endTransmission();
  hibernate_delay(2);
  Wire.requestFrom(HDC1000_I2CADDR, (uint8_t)2);
  uint16_t r = Wire.read();
  r <<= 8;
  r |= Wire.read();
  return r;
}
boolean HDC1000_begin(void) {
  Wire.begin();
  HDC1000_reset();
  if (HDC1000_read16(HDC1000_MANUFID) != 0x5449) return false;
  if (HDC1000_read16(HDC1000_DEVICEID) != 0x1000) return false;
  return true;
}
void HDC1000_gettemphum(void) {
  Wire.beginTransmission(HDC1000_I2CADDR);
  Wire.write(HDC1000_TEMP);
  Wire.endTransmission();
  hibernate_delay(20);
  Wire.requestFrom(HDC1000_I2CADDR, (uint8_t)4);
  current_temp = Wire.read();
  current_temp <<= 8;
  current_temp |= Wire.read();
  current_humid = Wire.read();
  current_humid <<= 8;
  current_humid |= Wire.read();
  if (0) {
    Serial.print("raw temp, humid: ");
    Serial.print(current_temp); Serial.print(", ");
    Serial.print(current_humid); Serial.println();
  }
  // convert raw T to degrees F: ((T/2^16)*165-40)*9/5+32
  current_temp = (((((uint32_t)current_temp * 1485) >> 16) - 360) / 5) + 32;
  // convert raw H to relative humidity in %
  current_humid = ((uint32_t)current_humid * 100) >> 16;
  if (DEBUG) {
    Serial.print("temp, humid: ");
    Serial.print(current_temp); Serial.print(", ");
    Serial.print(current_humid); Serial.println();
  }
}
//-----------------------------------------------------------------------------------------------------------------
//    LCD display routines
//
//    The segments, including the decimal points, are wired to Teensy ports C and D so we can bulk-write them.
//    The common cathodes for both digits are wired together to a single I/O port.
//    The displays are not multiplexed, and need to be driven by AC to avoid image degradation and reduced life.
//-----------------------------------------------------------------------------------------------------------------

void shownum (byte num, byte points, int time) { // display 2-digit number, or "En" for value 100+n
#define DISPLAY_DELAY 25  // 25*2 msec is 20 Hz, which flickers a little when viewed from the side
  // but slower is better to minimize power utilization
  int cycles = time / (2 * DISPLAY_DELAY);
  byte d1segments = digit_segments[(num / 10) % 11] | (points & POINT1 ? 0x80 : 0);
  byte d2segments = digit_segments[num % 10] | (points & POINT2 ? 0x80 : 0);
  do {
    digitalWriteFast(LCD_DIGIT_COM, LOW); // first cycle: COM is low
    LCD_DIGIT1_SEGMENTS = d1segments; // and lit segments are high
    LCD_DIGIT2_SEGMENTS = d2segments;
    hibernate_delay(DISPLAY_DELAY);
    digitalWriteFast(LCD_DIGIT_COM, HIGH); // second cycle: COM is high
    LCD_DIGIT1_SEGMENTS = ~d1segments; // and lit segments are low
    LCD_DIGIT2_SEGMENTS = ~d2segments;
    hibernate_delay(DISPLAY_DELAY);
  } while (--cycles);
  digitalWriteFast(LCD_DIGIT_COM, LOW);
  LCD_DIGIT1_SEGMENTS = 0;
  LCD_DIGIT2_SEGMENTS = 0;
}
void fatal_error (int errnum) {   // become catatonic with flashing error number "En"
  while (1) {
    shownum(100 + errnum, NOPOINTS, 500);
    hibernate_delay(500);
  }
}
//-----------------------------------------------------------------------------------------------------------------
// HC-12 433 Mhz wireless serial port transceiver routines
//
// This is connected to the second serial UART port, Serial1 on pins 0 and 2.
// The "set" line controls whether we're sending commands or data.
//-----------------------------------------------------------------------------------------------------------------

#define HC12_SET 4    // port number for "set" input of transceiver

void send_command(const char *cmd) {
  Serial1.println(cmd);
  if (DEBUG) {
    Serial.print("cmd: ");
    Serial.println(cmd);
  }
  Serial1.flush();  // wait for all bytes to be sent
}
bool get_response(const char *expected_rsp) {
  char response[50];
  byte response_length;
  Serial1.setTimeout(50); // give it up to these many msec to process
  response_length = Serial1.readBytes(response, sizeof(response) - 1);
  response[response_length] = '\0';
  if (DEBUG) {
    Serial.print("rsp: ");
    if (response_length == 0) Serial.print("<none>");
    else for (int i = 0; response[i] > 0x0d; ++i) Serial.print(response[i]);
    Serial.println();
  }
  if (expected_rsp) {
    if (strcmp(response, expected_rsp) != 0) { // didn't get expected response
      if (DEBUG) Serial.println("Didn't get expected response");
      return false;
    }
    if (DEBUG) Serial.println("Got expected response");
  }
  return true;
}

//-----------------------------------------------------------------------------------------------------------------
//    Initialization
//-----------------------------------------------------------------------------------------------------------------

void setup() {
  int tries;

  if (DEBUG) {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Sensor started.\n");
  }
  pinMode(SW1, INPUT_PULLUP); // configure addr/mode switches
  pinMode(SW2, INPUT_PULLUP);
  pinMode(SW3, INPUT_PULLUP);
  pinMode(SW4, INPUT_PULLUP);
  analogReference(EXTERNAL);  // for battery voltage check, use external reference voltage

  LCD_DIGIT1_SEGMENTS = 0; // initialize 2-digit display
  LCD_DIGIT2_SEGMENTS = 0;
  for (int i = 0; i < sizeof(lcd_pins); ++i)
    pinMode(lcd_pins[i], OUTPUT);
  shownum(88, POINT1 + POINT2, 1000); // show all segments for a second
  delay(500);
  if (TESTS) {
    for (int i = 0; i < 9; ++i) {// display all possible digits
      shownum(i * 10 + i + 1, NOPOINTS, 300);
    }
  }
  HDC1000_begin(); // start current_temperature/humidity sensor

  pinMode(HC12_SET, OUTPUT);  // start HC-12 RF transceiver
  digitalWriteFast(HC12_SET, HIGH);
  Serial1.begin(9600, SERIAL_8N1);
  delay(10);
  digitalWriteFast(HC12_SET, LOW); // put HC-12 in command mode
  delay(10);
  tries = 1;
  do {
    if (++tries > 10) fatal_error(1); // E1 error
    send_command("AT");  // wake it up
    // That sometimes fails. When batteries are weak? Should we delay more, decrease the
    // filter electrolytic capacitor, try a few times, or some combination of all of those?
  } while (!get_response("OK\x0d\x0a")); // for now: try a bunch of times
  //send_command("AT+DEFAULT");  // setting everything to default doesn't work if  FU1 follows!?!
  //get_response(NULL);
  send_command("AT+FU1"); // set FU1 mode: medium speed, 3.6 ma while active
  get_response(NULL);
  if (DEBUG) {
    send_command("AT+V");  // get version number
    get_response(NULL);
    send_command("AT+RX"); // get all parameters
    get_response(NULL);
  }
  digitalWriteFast(HC12_SET, HIGH); // put HC-12 in data mode
  delay(100); // wait a bit

  packet.stx = STX;    // initialize the packet to send
  packet.etx = ETX;
  packet.addr = ((digitalRead(SW3) << 1) | digitalRead(SW4)) ^ 0x03; // 0..3 unit address
  for (int i = 0; i < 5; ++i) {  // flash our address a few times
    shownum(packet.addr, NOPOINTS, 500);
    delay(500);
  }
  if (DEBUG) {
    Serial.print("unit address: ");
    Serial.println(packet.addr);
  }
  packet.type = 1;

  if (TESTS) {  // display battery test point voltage
    for (int i = 0; i < 5; ++i) {
      uint32_t aval;
      uint16_t volt_tenths;
      aval = analogRead(BATTERY_PIN);
      volt_tenths = aval * AREF_MV / (1023 * 100);
      if (DEBUG) {
        Serial.print("analog pin = ");
        Serial.println(aval);
        Serial.println(volt_tenths);
      }
      shownum(volt_tenths, POINT2, 1000); // voltage in tenths
      delay(500);
    }
  }
}

//-----------------------------------------------------------------------------------------------------------------
//    The main loop
//-----------------------------------------------------------------------------------------------------------------

unsigned long send_data_time;
void loop() {
  int tries;

  // 1. Put the RF transmitter to sleep so it only uses about 22 uA of power

  digitalWriteFast(HC12_SET, LOW); // enter command mode
  sleep_delay(10);
  send_command("AT"); // the first command is ignored!
  get_response(NULL);
  do {
    sleep_delay(10);
    send_command("AT+SLEEP");
  } while (!get_response("OK+SLEEP\x0d\x0a")); // ... might get "ERROR\x0d\0xa" if busy, so keep trying
  if (DEBUG) {
    Serial.print("send took "); Serial.print(millis() - send_data_time); Serial.println(" msec");;
  }
  digitalWriteFast(HC12_SET, HIGH);  // back to data mode

  // 2. Display the temperature for a while, uisng hibernation waits to minimize power drain

  for (int loops = 0; loops < 10; ++loops) {  // 10 * 1.5 seconds: transmit every 15 seconds
    HDC1000_gettemphum();  // get current temperature and humidity
    shownum(current_temp, NOPOINTS, 500); // show temp for half a second
    if (TESTS) shownum(current_humid, POINT1, 250);  // (briefly show humidity)
    hibernate_delay(1000); // sleep for one second
  }

  // 3. Wake up the transmitter send a packet with the latest temperature

  hibernate_delay (100 * packet.addr); // "random" delay based on our address, to minimize collisions
  packet.temperature = min(max(current_temp, 0), 120);
  packet.humidity = min(max(current_humid, 0), 100);
  packet.battery = (uint32_t)analogRead(BATTERY_PIN) * AREF_MV / (1023 * 50); // VCC/2 resistor network!
  packet.lrc = 0;
  for (int i = 0; i < sizeof(packet) - 1; ++i) packet.lrc ^= ((byte *)(&packet))[i];
  digitalWriteFast(HC12_SET, LOW); // put HC-12 in command mode
  // from here on we wait with the cpu in sleep mode instead of hibernate, to keep the
  // transmitter active with the full 3.3V on the VDD pin
  sleep_delay(10);
  tries = 1;
  do {
    if (++tries > 10) fatal_error(2); // E2 error
    send_command("AT");  // wake it up
  } while (!get_response("OK\x0d\x0a")); // keep at it until it responds
  digitalWriteFast(HC12_SET, HIGH); // put HC-12 in data mode
  sleep_delay(50); // need this! 25 is not enough
  if (DEBUG) {
    Serial.print("send data packet #"); Serial.print(++count_sent_packets); Serial.print(": ");
    for (int i = 0; i < sizeof(packet); ++i) {
      Serial.print(((byte *)(&packet))[i], DEC);  Serial.print(' ');
    }
    Serial.println();
  }
  Serial1.write((byte *)(&packet), sizeof(packet)); // send the packet data
  Serial1.flush(); // wait for it all to go out
  send_data_time = millis();
  sleep_delay(50); // give it a time to start
}
