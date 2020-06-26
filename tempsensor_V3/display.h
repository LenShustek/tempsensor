//file: display.h

// pin numbers are *Arduino* fake pin numbers
#define DSP_CS_PIN 5  // chip select, PD5, PDIP pin 11, TQFP pin 9
#define DSP_CS_PORT 0x0b    // r/w data port D, 0-based address for SBI/CBI instructions
#define DSP_CS_BIT  5       // bit number
#define DSP_DC_PIN 4   // data/cmd, PD4, PDIP pin 6, TQFP pin 2
#define DSP_DC_PORT 0x0b    // r/w data port D, 0-based address for SBI/CBI instructions
#define DSP_DC_BIT  4       // bit number
#define DSP_RESET_PIN 3  // reset, PD3, PDIP pin 5, TQFP pin 1
#define DSP_BUSY_PIN 2   // busy, PD2, PDIP pin 4, TQFP pin 32


#define EPD_WIDTH   200
#define EPD_HEIGHT  200

#define DSP_MIN_ASCII 32   // space
#define DSP_MAX_ASCII 126  // tilde
#define DSP_NUM_ASCII (DSP_MAX_ASCII-DSP_MIN_ASCII+1)

struct font_t { // generalized description of of a fixed-width font, in flash memory
   byte charw;         // width of each character in pixels; must be a multiple of 8
   byte charh;         // height of each character in pixels; can be arbitrary
   byte charmap[DSP_NUM_ASCII];   // map from ASCII into the character position below; 0xff if absent
   const byte * PROGMEM fontdata;   // ptr to an array in flash memory of charw/8 * charh bytes for each character present in the charmap
} ;
#define XX 0xff     // marker for "no character"

extern const struct font_t font24 PROGMEM;
extern const struct font_t font34 PROGMEM;
extern const struct font_t font67 PROGMEM;

bool dsp_init (void);
void dsp_sleep(void) ;
void dsp_start(bool full_update);
void dsp_clearmem(void);
void dsp_displaymem(bool wait);
bool dsp_busy(void);
void dsp_setmemarea(int x_start, int y_start, int x_end, int y_end);  // set window position in display
void dsp_setmemptr (int x, int y);
void dsp_senddata(byte *ptr, int cnt);
void dsp_writestr_mag(const struct font_t *font, int x, int y, int magnification, char *msg);
void dsp_writestr(const struct font_t *font, int x, int y, char *msg);

//* display.h

