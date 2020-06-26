//file: RFradio.cpp
/*---------------------------------------------------------------------------------

   RFM95 LoRa RF radio routines for tempsensor/temprcvr

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

#include <Arduino.h>
#include <SPI.h>
#include <RFradio.h>

#define DEBUG 0

// The following bitbang macros are much faster, for 8-bit AVR processors,
// than digitalWrite: about 8 usec instead of 80!
// For Teensy 3.x they don't make as much a difference. But they still work even
// though Paul Stroffregen cleverly uses the bitband alias addressing on the Cortex M4:
// https://forum.pjrc.com/threads/26225-problem-bit-banging-using-digitalPinToBitMask-et-al
// The only problem is: they fail the first time after code is downloaded. Ok, just cycle
// the power. Is it fixed by doing pinMode() before the first call to digitalWrite()?

#define setbit(pin,reg,bitmask) *reg |= bitmask
#define clrbit(pin,reg,bitmask) *reg &= ~(bitmask)

static byte cs_pin, cs_bitmask, reset_pin;
volatile byte *cs_reg;

// Flash-resident strings describing the radio errors.
// This must match the enum RFERR definition in RFradio.h!

const char err_RF_OK[] PROGMEM         = "Ok";
const char err_RF_RXTIMEOUT[] PROGMEM  = "timeout";
const char err_RF_BADHDR[] PROGMEM     = "bad hdr";
const char err_RF_TOOSMALL[] PROGMEM   = "too small";
const char err_RF_TOOBIG[] PROGMEM     = "too big";
const char err_RF_BADCRC[] PROGMEM     = "bad CRC";
const char err_RF_NOPKT[] PROGMEM      = "no pkt";
const char err_RF_UNKNOWN[] PROGMEM    = "unknown";

const char* const RF_errors[] PROGMEM = {
   err_RF_OK, err_RF_RXTIMEOUT, err_RF_BADHDR, err_RF_TOOSMALL,
   err_RF_TOOBIG, err_RF_BADCRC, err_RF_NOPKT, err_RF_UNKNOWN };
   
byte power_level;

//inline byte RFspi_read (byte regno) __attribute__((always_inline));
inline byte RFspi_read (byte regno) {
   //SPI.beginTransaction(settings);
   clrbit(cs_pin, cs_reg, cs_bitmask); // digitalWrite(cs_pin, LOW);
   SPI.transfer(regno); // write register number, discard status
   byte val = SPI.transfer(0);  // get register data, output data is ignored
   setbit(cs_pin, cs_reg, cs_bitmask); // digitalWrite(cs_pin, HIGH);
   //SPI.endTransaction();
   return val; }

//inline byte RFspi_cmd (byte cmd) __attribute__((always_inline));
inline byte RFspi_cmd (byte cmd) {
   //SPI.beginTransaction(settings);
   clrbit(cs_pin, cs_reg, cs_bitmask); // digitalWrite(cs_pin, LOW);
   byte status = SPI.transfer(cmd); // write command, get status
   setbit(cs_pin, cs_reg, cs_bitmask); // digitalWrite(cs_pin, HIGH);
   //SPI.endTransaction();
   return status; }

//inline void RFspi_write (byte regno, byte data) __attribute__((always_inline));
inline void RFspi_write (byte regno, byte data) {
   //SPI.beginTransaction(settings);
   clrbit(cs_pin, cs_reg, cs_bitmask); // digitalWrite(cs_pin, LOW);
   SPI.transfer(regno+0x80); // write register number, discard status
   SPI.transfer(data);  // write data
   #if defined(__arm__) && defined(CORE_TEENSY)
   // Sigh: some devices, such as MRF89XA don't work properly on Teensy 3.1:
   // At 1MHz, the clock returns low _after_ slave select goes high, which prevents SPI
   // write working. This delay gives time for the clock to return low.
   delayMicroseconds(5);
   #endif
   setbit(cs_pin, cs_reg, cs_bitmask); // digitalWrite(cs_pin, HIGH);
   //SPI.endTransaction();
}
void RFspi_burstread(byte reg, byte* dest, byte len) {
   //SPI.beginTransaction(settings);
   clrbit(cs_pin, cs_reg, cs_bitmask); // digitalWrite(cs_pin, LOW);
   SPI.transfer(reg); // Send the start address
   while (len--)	*dest++ = SPI.transfer(0);
   setbit(cs_pin, cs_reg, cs_bitmask); // digitalWrite(cs_pin, HIGH);
   //SPI.endTransaction();
}
void RFspi_burstwrite(byte reg, const byte* src, byte len) {
   //SPI.beginTransaction(settings);
   clrbit(cs_pin, cs_reg, cs_bitmask); // digitalWrite(cs_pin, LOW);
   SPI.transfer(reg+0x80); // Send the start address
   while (len--) SPI.transfer(*src++);
   setbit(cs_pin, cs_reg, cs_bitmask); // digitalWrite(cs_pin, HIGH);
   //SPI.endTransaction();
}
void RFradio_printRegisters(const char *title) {
   byte registers[] = { 0x01, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10,
                        0x11, 0x12, 0x13, 0x014, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
                        0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27 };
   Serial.println(title);
   for (byte i = 0; i < sizeof(registers); i++) {
      Serial.print("  ");
      Serial.print(registers[i], HEX);
      Serial.print(":");
      Serial.print(RFspi_read(registers[i]), HEX); }
   Serial.println(); }

// These declarations are adapted from RadioHead, http://www.airspayce.com/mikem/arduino/RadioHead/

// The crystal oscillator frequency of the module
#define RF95_FXOSC 32000000.0

// The Frequency Synthesizer step = RF95_FXOSC / 2^^19
#define RF95_FSTEP  (RF95_FXOSC / 524288)

// Register names (LoRa Mode, from table 85)
#define RF95_REG_00_FIFO                           0x00
#define RF95_REG_01_OP_MODE                        0x01
#define RF95_REG_02_RESERVED                       0x02
#define RF95_REG_03_RESERVED                       0x03
#define RF95_REG_04_RESERVED                       0x04
#define RF95_REG_05_RESERVED                       0x05
#define RF95_REG_06_FRF_MSB                        0x06
#define RF95_REG_07_FRF_MID                        0x07
#define RF95_REG_08_FRF_LSB                        0x08
#define RF95_REG_09_PA_CONFIG                      0x09
#define RF95_REG_0A_PA_RAMP                        0x0a
#define RF95_REG_0B_OCP                            0x0b
#define RF95_REG_0C_LNA                            0x0c
#define RF95_REG_0D_FIFO_ADDR_PTR                  0x0d
#define RF95_REG_0E_FIFO_TX_BASE_ADDR              0x0e
#define RF95_REG_0F_FIFO_RX_BASE_ADDR              0x0f
#define RF95_REG_10_FIFO_RX_CURRENT_ADDR           0x10
#define RF95_REG_11_IRQ_FLAGS_MASK                 0x11
#define RF95_REG_12_IRQ_FLAGS                      0x12
#define RF95_REG_13_RX_NB_BYTES                    0x13
#define RF95_REG_14_RX_HEADER_CNT_VALUE_MSB        0x14
#define RF95_REG_15_RX_HEADER_CNT_VALUE_LSB        0x15
#define RF95_REG_16_RX_PACKET_CNT_VALUE_MSB        0x16
#define RF95_REG_17_RX_PACKET_CNT_VALUE_LSB        0x17
#define RF95_REG_18_MODEM_STAT                     0x18
#define RF95_REG_19_PKT_SNR_VALUE                  0x19
#define RF95_REG_1A_PKT_RSSI_VALUE                 0x1a
#define RF95_REG_1B_RSSI_VALUE                     0x1b
#define RF95_REG_1C_HOP_CHANNEL                    0x1c
#define RF95_REG_1D_MODEM_CONFIG1                  0x1d
#define RF95_REG_1E_MODEM_CONFIG2                  0x1e
#define RF95_REG_1F_SYMB_TIMEOUT_LSB               0x1f
#define RF95_REG_20_PREAMBLE_MSB                   0x20
#define RF95_REG_21_PREAMBLE_LSB                   0x21
#define RF95_REG_22_PAYLOAD_LENGTH                 0x22
#define RF95_REG_23_MAX_PAYLOAD_LENGTH             0x23
#define RF95_REG_24_HOP_PERIOD                     0x24
#define RF95_REG_25_FIFO_RX_BYTE_ADDR              0x25
#define RF95_REG_26_MODEM_CONFIG3                  0x26

#define RF95_REG_40_DIO_MAPPING1                   0x40
#define RF95_REG_41_DIO_MAPPING2                   0x41
#define RF95_REG_42_VERSION                        0x42

#define RF95_REG_4B_TCXO                           0x4b
#define RF95_REG_4D_PA_DAC                         0x4d
#define RF95_REG_5B_FORMER_TEMP                    0x5b
#define RF95_REG_61_AGC_REF                        0x61
#define RF95_REG_62_AGC_THRESH1                    0x62
#define RF95_REG_63_AGC_THRESH2                    0x63
#define RF95_REG_64_AGC_THRESH3                    0x64

// RF95_REG_01_OP_MODE                             0x01
#define RF95_LONG_RANGE_MODE                       0x80
#define RF95_ACCESS_SHARED_REG                     0x40
#define RF95_LOW_FREQUENCY_MODE                    0x08
#define RF95_MODE                                  0x07
#define RF95_MODE_SLEEP                            0x00
#define RF95_MODE_STDBY                            0x01
#define RF95_MODE_FSTX                             0x02
#define RF95_MODE_TX                               0x03
#define RF95_MODE_FSRX                             0x04
#define RF95_MODE_RXCONTINUOUS                     0x05
#define RF95_MODE_RXSINGLE                         0x06
#define RF95_MODE_CAD                              0x07

// RF95_REG_09_PA_CONFIG                           0x09
#define RF95_PA_SELECT                             0x80
#define RF95_MAX_POWER                             0x70
#define RF95_OUTPUT_POWER                          0x0f

// RF95_REG_0A_PA_RAMP                             0x0a
#define RF95_LOW_PN_TX_PLL_OFF                     0x10
#define RF95_PA_RAMP                               0x0f
#define RF95_PA_RAMP_3_4MS                         0x00
#define RF95_PA_RAMP_2MS                           0x01
#define RF95_PA_RAMP_1MS                           0x02
#define RF95_PA_RAMP_500US                         0x03
#define RF95_PA_RAMP_250US                         0x0
#define RF95_PA_RAMP_125US                         0x05
#define RF95_PA_RAMP_100US                         0x06
#define RF95_PA_RAMP_62US                          0x07
#define RF95_PA_RAMP_50US                          0x08
#define RF95_PA_RAMP_40US                          0x09
#define RF95_PA_RAMP_31US                          0x0a
#define RF95_PA_RAMP_25US                          0x0b
#define RF95_PA_RAMP_20US                          0x0c
#define RF95_PA_RAMP_15US                          0x0d
#define RF95_PA_RAMP_12US                          0x0e
#define RF95_PA_RAMP_10US                          0x0f

// RF95_REG_0B_OCP                                 0x0b
#define RF95_OCP_ON                                0x20
#define RF95_OCP_TRIM                              0x1f

// RF95_REG_0C_LNA                                 0x0c
#define RF95_LNA_GAIN                              0xe0
#define RF95_LNA_GAIN_G1                           0x20
#define RF95_LNA_GAIN_G2                           0x40
#define RF95_LNA_GAIN_G3                           0x60
#define RF95_LNA_GAIN_G4                           0x80
#define RF95_LNA_GAIN_G5                           0xa0
#define RF95_LNA_GAIN_G6                           0xc0
#define RF95_LNA_BOOST_LF                          0x18
#define RF95_LNA_BOOST_LF_DEFAULT                  0x00
#define RF95_LNA_BOOST_HF                          0x03
#define RF95_LNA_BOOST_HF_DEFAULT                  0x00
#define RF95_LNA_BOOST_HF_150PC                    0x11

// RF95_REG_11_IRQ_FLAGS_MASK                      0x11
#define RF95_RX_TIMEOUT_MASK                       0x80
#define RF95_RX_DONE_MASK                          0x40
#define RF95_PAYLOAD_CRC_ERROR_MASK                0x20
#define RF95_VALID_HEADER_MASK                     0x10
#define RF95_TX_DONE_MASK                          0x08
#define RF95_CAD_DONE_MASK                         0x04
#define RF95_FHSS_CHANGE_CHANNEL_MASK              0x02
#define RF95_CAD_DETECTED_MASK                     0x01

// RF95_REG_12_IRQ_FLAGS                           0x12
#define RF95_RX_TIMEOUT                            0x80
#define RF95_RX_DONE                               0x40
#define RF95_PAYLOAD_CRC_ERROR                     0x20
#define RF95_VALID_HEADER                          0x10
#define RF95_TX_DONE                               0x08
#define RF95_CAD_DONE                              0x04
#define RF95_FHSS_CHANGE_CHANNEL                   0x02
#define RF95_CAD_DETECTED                          0x01

// RF95_REG_18_MODEM_STAT                          0x18
#define RF95_RX_CODING_RATE                        0xe0
#define RF95_MODEM_STATUS_CLEAR                    0x10
#define RF95_MODEM_STATUS_HEADER_INFO_VALID        0x08
#define RF95_MODEM_STATUS_RX_ONGOING               0x04
#define RF95_MODEM_STATUS_SIGNAL_SYNCHRONIZED      0x02
#define RF95_MODEM_STATUS_SIGNAL_DETECTED          0x01

// RF95_REG_1C_HOP_CHANNEL                         0x1c
#define RF95_PLL_TIMEOUT                           0x80
#define RF95_RX_PAYLOAD_CRC_IS_ON                  0x40
#define RF95_FHSS_PRESENT_CHANNEL                  0x3f

// RF95_REG_1D_MODEM_CONFIG1                       0x1d
#define RF95_BW                                    0xf0

#define RF95_BW_7_8KHZ                             0x00
#define RF95_BW_10_4KHZ                            0x10
#define RF95_BW_15_6KHZ                            0x20
#define RF95_BW_20_8KHZ                            0x30
#define RF95_BW_31_25KHZ                           0x40
#define RF95_BW_41_7KHZ                            0x50
#define RF95_BW_62_5KHZ                            0x60
#define RF95_BW_125KHZ                             0x70
#define RF95_BW_250KHZ                             0x80
#define RF95_BW_500KHZ                             0x90
#define RF95_CODING_RATE                           0x0e
#define RF95_CODING_RATE_4_5                       0x02
#define RF95_CODING_RATE_4_6                       0x04
#define RF95_CODING_RATE_4_7                       0x06
#define RF95_CODING_RATE_4_8                       0x08
#define RF95_IMPLICIT_HEADER_MODE_ON               0x01

// RF95_REG_1E_MODEM_CONFIG2                       0x1e
#define RF95_SPREADING_FACTOR                      0xf0
#define RF95_SPREADING_FACTOR_64CPS                0x60
#define RF95_SPREADING_FACTOR_128CPS               0x70
#define RF95_SPREADING_FACTOR_256CPS               0x80
#define RF95_SPREADING_FACTOR_512CPS               0x90
#define RF95_SPREADING_FACTOR_1024CPS              0xa0
#define RF95_SPREADING_FACTOR_2048CPS              0xb0
#define RF95_SPREADING_FACTOR_4096CPS              0xc0
#define RF95_TX_CONTINUOUS_MOE                     0x08

#define RF95_PAYLOAD_CRC_ON                        0x04
#define RF95_SYM_TIMEOUT_MSB                       0x03

// RF95_REG_4B_TCXO                                0x4b
#define RF95_TCXO_TCXO_INPUT_ON                    0x10

// RF95_REG_4D_PA_DAC                              0x4d
#define RF95_PA_DAC_DISABLE                        0x04
#define RF95_PA_DAC_ENABLE                         0x07

static byte current_mode;

void set_radio_mode(byte mode) {
   RFspi_write(RF95_REG_01_OP_MODE, mode + RF95_LONG_RANGE_MODE);
   current_mode = mode; }

byte RFradio_init(byte cspin, byte resetpin) { // one-time initialization
// returns the radio version number
   cs_pin = cspin;
   reset_pin = resetpin;
   digitalWrite(cs_pin, HIGH);
   pinMode(cs_pin, OUTPUT);
   cs_reg = portOutputRegister(digitalPinToPort(cs_pin));
   cs_bitmask = digitalPinToBitMask(cs_pin);
   digitalWrite(reset_pin, HIGH);
   pinMode(reset_pin, OUTPUT);

   #if 0 // code to investigate Teensy 3.x's use of aliased bitband regions on the Cortex M4
   char msg[25];
   sprintf(msg, "%08lX %02X %02X", (uint32_t)cs_reg, cs_bitmask, *cs_reg);
   void display_line(const char *str);
   display_line(msg);
   #endif

   delay(10);            // reset the radio
   digitalWrite(reset_pin, LOW);
   delay(10);
   digitalWrite(reset_pin, HIGH);
   delay(10);
   SPI.begin();
   SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
   RFspi_read(RF95_REG_01_OP_MODE); // Do test register read. Why? See comment in temprcvr.ino.
   if (DEBUG) RFradio_printRegisters("after reset:");

   // Set sleep mode and long range mode
   set_radio_mode(RF95_MODE_SLEEP);
   delay(10); // Wait for sleep mode to take over from say, CAD
   // Check we are in sleep mode, with LORA set
   if (RFspi_read(RF95_REG_01_OP_MODE) != (RF95_MODE_SLEEP | RF95_LONG_RANGE_MODE)) {
      //	Serial.println(RFspi_read(RF95_REG_01_OP_MODE), HEX);
      fatal("No radio", FATAL_NO_RADIO); // No device present?
   }
   byte radio_version = RFspi_read(RF95_REG_42_VERSION);

   // Configure so the entire 256 byte FIFO can be used for either receive
   // or transmit, but not both at the same time
   RFspi_write(RF95_REG_0E_FIFO_TX_BASE_ADDR, 0);
   RFspi_write(RF95_REG_0F_FIFO_RX_BASE_ADDR, 0);
   // Packet format is: explicit-header + payload + crc
   // header: length, REC rate+CRC bit, CRC
   // max message data length is 255 - 4 = 251 octets

   RFspi_write(RF95_REG_01_OP_MODE, RF95_MODE_STDBY); // enter standby mode

   // Set up default configuration
   // No Sync Words in LORA mode.
   RFspi_write(RF95_REG_1D_MODEM_CONFIG1,0x72);
   RFspi_write(RF95_REG_1E_MODEM_CONFIG2,0x74);
   RFspi_write(RF95_REG_26_MODEM_CONFIG3,0x04);

   RFspi_write(RF95_REG_20_PREAMBLE_MSB, 0); // preamble length is 8
   RFspi_write(RF95_REG_21_PREAMBLE_LSB, 8);

   // Set frequency: Frf = FRF / FSTEP
   //float centre = 915.0;  // for center frequency of 915 Khz (3" antenna)
   float centre = 868.0;  // for center frequency of 868 Khz (3.25" antenna)
   uint32_t frf = (centre * 1000000.0) / RF95_FSTEP;
   RFspi_write(RF95_REG_06_FRF_MSB, (frf >> 16) & 0xff);
   RFspi_write(RF95_REG_07_FRF_MID, (frf >> 8) & 0xff);
   RFspi_write(RF95_REG_08_FRF_LSB, frf & 0xff);

   //#define OutputPower 7  // about 10 dBm
   #define OutputPower 14   // about 16 dBm
   RFspi_write(RF95_REG_4D_PA_DAC, RF95_PA_DAC_DISABLE);
   // From the RadioHead library: "Pout = 2 + OutputPower.
	//    The documentation is pretty confusing on this topic: PaSelect says the max power is 20dBm,
	//    but OutputPower claims it would be 17dBm. My measurements show 20dBm is correct"
   RFspi_write(RF95_REG_09_PA_CONFIG, RF95_PA_SELECT | OutputPower);
   power_level = OutputPower; // for display purposes
   if (DEBUG) RFradio_printRegisters("after init:"); 
   return radio_version; }

void RFradio_sleep(void) {
   set_radio_mode(RF95_MODE_SLEEP+RF95_LONG_RANGE_MODE);
   delay(10); // Wait for sleep mode to take over from say, CAD
}
void RFradio_standby(void) {
   set_radio_mode(RF95_MODE_STDBY);    // enter standby mode
}
void RFradio_receive(void) {
   set_radio_mode(RF95_MODE_STDBY);         // make sure we're in standby mode
   RFspi_write(RF95_REG_12_IRQ_FLAGS, 0xff);  // clear all interrupt bits
   RFspi_write(RF95_REG_11_IRQ_FLAGS_MASK, 0x07);  // enable only receive bits
   set_radio_mode(RF95_MODE_RXCONTINUOUS);  // then enter receive mode
}
bool RFradio_gotpkt(void) {
   if (current_mode != RF95_MODE_RXCONTINUOUS) fatal("bad gotpkt", FATAL_BAD_GOTPKT);
   byte irq_flags = RFspi_read(RF95_REG_12_IRQ_FLAGS);
   return (irq_flags & RF95_RX_DONE) != 0; }

enum RFERR RFradio_getpkt (byte *ptr, byte length) {
   enum RFERR err;
   byte irq_flags = RFspi_read(RF95_REG_12_IRQ_FLAGS);
   if ((irq_flags & RF95_RX_DONE) == 0) fatal("bad getpkt", FATAL_BAD_GETPKT);
   byte rcvlen = RFspi_read(RF95_REG_13_RX_NB_BYTES);
   if (irq_flags & RF95_RX_TIMEOUT) err = RF_RXTIMEOUT;
   else if (irq_flags & RF95_PAYLOAD_CRC_ERROR) err = RF_BADCRC;
   else if (!(irq_flags & RF95_VALID_HEADER)) err = RF_BADHDR;
   else if (rcvlen < length) err = RF_TOOSMALL;
   else if (rcvlen > length) err = RF_TOOBIG;
   else { // got good packet
      err = RF_OK;
      RFspi_write(RF95_REG_0D_FIFO_ADDR_PTR, RFspi_read(RF95_REG_10_FIFO_RX_CURRENT_ADDR));
      RFspi_burstread(RF95_REG_00_FIFO, ptr, length); // copy out the packet payload data
      RFspi_write(RF95_REG_12_IRQ_FLAGS, 0xff);       // Clear all IRQ flags
   }
   set_radio_mode(RF95_MODE_STDBY);
   return err; }
   
void RFRadio_pktstats(int8_t *rssi, int8_t *snr) { // get stats on last packet
  // received signal strength indicator, in dB
  *rssi = (int8_t)RFspi_read(RF95_REG_1A_PKT_RSSI_VALUE) - 137; 
  // signal-to-noise ratio, in dB
  *snr = (int8_t)RFspi_read(RF95_REG_19_PKT_SNR_VALUE) / 4;
}

void RFradio_transmit(byte *ptr, int length) {
   set_radio_mode(RF95_MODE_STDBY);             // enter standby mode?? TEMP
   //RFradio_printRegisters("before filling xmt buffer:");
   RFspi_write(RF95_REG_0D_FIFO_ADDR_PTR, 0);      // point to start of FIFO
   RFspi_burstwrite(RF95_REG_00_FIFO, ptr, length);   // write all the data
   RFspi_write(RF95_REG_22_PAYLOAD_LENGTH, length);   // tell it how big
   RFspi_write(RF95_REG_12_IRQ_FLAGS, 0xff);  // clear all interrupt bits
   RFspi_write(RF95_REG_11_IRQ_FLAGS_MASK, ~RF95_TX_DONE);  // enable only TX_DONE
   //RFradio_printRegisters("after filling xmt buffer:");
   set_radio_mode(RF95_MODE_TX); // enter transmit mode
   //RFradio_printRegisters("after setting xmt:");
   byte irq_flags, last_irq_flags = 0xff;
   do {
      irq_flags = RFspi_read(RF95_REG_12_IRQ_FLAGS);
      if (DEBUG && irq_flags != last_irq_flags) {
         //Serial.print("IRQ flags: ");
         //Serial.println(irq_flags, HEX);
         //RFradio_printRegisters("at IRQ flags change:");
         last_irq_flags = irq_flags; }
      if (irq_flags & ~RF95_TX_DONE) {
         RFspi_write(RF95_REG_12_IRQ_FLAGS, irq_flags); // reset any other IRQ flags?
         //Serial.print("resetting IRQ bits "); Serial.println(irq_flags, HEX);
         //RFradio_printRegisters("after IRQ resetting:");
      } }
   while ((irq_flags & RF95_TX_DONE) == 0);
   RFspi_write(RF95_REG_12_IRQ_FLAGS, irq_flags);
   // timeout??
// after transmit we are in standby mode
   current_mode = RF95_MODE_STDBY; }

//*