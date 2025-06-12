#include <stdint.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <avr/pgmspace.h>
#include "logo.h"

#include "terminal6.h"
#include "terminal12.h"
#include "DejaVuSansMonoBold_32.h"

#include "i2c.h"

#define DISPLAY_USE_SH1106 1
#define VERTICAL_DISPLAY   1

#define SSD1306_DEFAULT_ADDRESS 0x78
#define SSD1306_SETCONTRAST 0x81
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON 0xA5
#define SSD1306_NORMALDISPLAY 0xA6
#define SSD1306_INVERTDISPLAY 0xA7
#define SSD1306_DISPLAYOFF 0xAE
#define SSD1306_DISPLAYON 0xAF
#define SSD1306_SETDISPLAYOFFSET 0xD3
#define SSD1306_SETCOMPINS 0xDA
#define SSD1306_SETVCOMDETECT 0xDB
#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETPRECHARGE 0xD9
#define SSD1306_SETMULTIPLEX 0xA8
#define SSD1306_SETLOWCOLUMN 0x00
#define SSD1306_SETHIGHCOLUMN 0x10
#define SSD1306_SETSTARTLINE 0x40
#define SSD1306_MEMORYMODE 0x20
#define SSD1306_COLUMNADDR 0x21
#define SSD1306_PAGEADDR   0x22
#define SSD1306_COMSCANINC 0xC0
#define SSD1306_COMSCANDEC 0xC8
#define SSD1306_SEGREMAP 0xA0
#define SSD1306_CHARGEPUMP 0x8D
#define SSD1306_SWITCHCAPVCC 0x2
#define SSD1306_NOP 0xE3
#define SSD1306_DEACTIVATE_SCROLL 0x2E

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define SCREEN_BUFF_SIZE (SCREEN_WIDTH * SCREEN_HEIGHT / 8)
uint8_t ScreenBuff[SCREEN_BUFF_SIZE];

// Issue single command to SSD1306, using I2C or hard/soft SPI as needed.
// Because command calls are often grouped, SPI transaction and selection
// must be started/ended in calling function for efficiency.
static void ssd1306_command(uint8_t command) {
    TWIStart(SSD1306_DEFAULT_ADDRESS);
    TWIWrite(0x00);         // Co = 0, D/C = 0
    TWIWrite(command);
    TWIStop();
}

#ifndef DISPLAY_USE_SH1106
// Issue list of commands to SSD1306, same rules as above re: transactions.
// This is a private function, not exposed.
static void ssd1306_commandList(const uint8_t *c, uint8_t n) {
    TWIStart(SSD1306_DEFAULT_ADDRESS);
    TWIWrite((uint8_t)0x00); // Co = 0, D/C = 0
    while (n--) {
      TWIWrite(pgm_read_byte(c++));
    }
    TWIStop();
}
#endif


/*!
    @brief  Push data currently in RAM to SSD1306 display.
    @return None (void).
    @note   Drawing operations are not visible until this function is
            called. Call after each graphics command, or after a whole set
            of graphics commands, as best needed by one's own application.
*/
void ssd1306_display(void) {
#ifdef DISPLAY_USE_SH1106 // 1.3 inch sh1106
    uint8_t *ptr = ScreenBuff;
    uint16_t count;

    for (uint8_t p = 0 ; p < 8; p++) {
        ssd1306_command(0xB0+p);
        ssd1306_command(0x10);
        ssd1306_command(0x02);  // SH1106 offset (132 vs 128)
        TWIStart(SSD1306_DEFAULT_ADDRESS);
        TWIWrite((uint8_t)0x40);  // D/C = 1
        count = 128;
        while (count--) {
            TWIWrite(*ptr++);
        }
        TWIStop();
    }
#else  // 0.96 inch
    static const uint8_t PROGMEM dlist1[] = {
      SSD1306_PAGEADDR,
      0,                      // Page start address
      0xFF,                   // Page end (not really, but works here)
      SSD1306_COLUMNADDR, 0}; // Column start address
    ssd1306_commandList(dlist1, sizeof(dlist1));
    ssd1306_command(SCREEN_WIDTH - 1); // Column end address

    uint16_t count = SCREEN_BUFF_SIZE;
    uint8_t *ptr = ScreenBuff;

    TWIStart(SSD1306_DEFAULT_ADDRESS);
    TWIWrite((uint8_t)0x40);
    while (count--) {
        TWIWrite(*ptr++);
    }
    TWIStop();
#endif
}




/*
 * Clear data currently in RAM
 *
 *****************************************************************/
static void clear_display(void){
    uint16_t i;
    //memset(ScreenBuff, 0, SCREEN_BUFF_SIZE);
    for ( i=0 ; i< SCREEN_BUFF_SIZE ; i++ ){
        ScreenBuff[i] = 0x00;
    }
}
#ifdef VERTICAL_DISPLAY
#define SCREEN_WIDTH_V  SCREEN_HEIGHT
#define SCREEN_HEIGHT_V SCREEN_WIDTH
static void writePixel(uint8_t x, uint8_t y, uint8_t pixel_state )
{
    if (x >= SCREEN_WIDTH_V || y >= SCREEN_HEIGHT_V) {
        return;
    }
    //y = SCREEN_HEIGHT_V-y-1;
    x = SCREEN_WIDTH_V-x-1;

    if ( pixel_state ) {
        ScreenBuff[y+(x/8)*SCREEN_HEIGHT_V] |= (1 << (x&7));
    }
    else {
        ScreenBuff[y+(x/8)*SCREEN_HEIGHT_V] &= ~(1 << (x&7));
    }
}
#else
static void writePixel(uint8_t x, uint8_t y, uint8_t pixel_state )
{
    if (x >= SCREEN_WIDTH || y >= SCREEN_HEIGHT) {
        return;
    }

    if ( pixel_state ) {
        ScreenBuff[x+(y/8)*SCREEN_WIDTH] |= (1 << (y&7));
    }
    else {
        ScreenBuff[x+(y/8)*SCREEN_WIDTH] &= ~(1 << (y&7));
    }
}
#endif

/**************************************************************************/
/*!
   @brief      Draw a PROGMEM-resident 1-bit image at the specified (x,y)
   position, using the specified foreground color (unset bits are transparent).
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    bitmap  byte array with monochrome bitmap
    @param    w   Width of bitmap in pixels
    @param    h   Height of bitmap in pixels

    https://github.com/adafruit/Adafruit-GFX-Library/blob/master/Adafruit_GFX.cpp
*/
/**************************************************************************/
static void draw_pgm_bmp(uint8_t x, uint8_t y, const uint8_t bitmap[],
                              uint8_t w, uint8_t h) {

    uint8_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
    uint8_t byte = 0;

    for (uint8_t j = 0; j < h; j++, y++) {
        for (uint8_t i = 0; i < w; i++) {
            if (i & 7)
                byte <<= 1;
            else
                byte = pgm_read_byte(&bitmap[j * byteWidth + i / 8]);
            writePixel(x + i, y, (byte & 0x80) );
        }
    }
}


static void draw_6x8_rect(uint8_t x, uint8_t y, uint8_t draw) {
    uint8_t byte = 0;

    for (uint8_t i = 0; i < 6; i++, x++) {
        if ( i == 5 )
            byte = 0;
        else
            byte = draw;
        for (uint8_t j = 0; j < 8; j++) {
            writePixel(x, y+j, byte );
        }
    }
}

static void draw_char(uint8_t x, uint8_t y, char c) {
    uint8_t byte = 0;

    for (uint8_t i = 0; i < 6; i++, x++) {
        if ( i == 5 )
            byte = 0;
        else
            byte = pgm_read_byte(&Terminal6x8[c-0x20][i]);
        for (uint8_t j = 0; j < 8; j++) {
            writePixel(x, y+j, (byte & 0x01) );
            byte >>= 1;
        }
    }
}

static void draw_large_char(uint8_t x, uint8_t y, char c) {
    uint16_t word = 0;

    for (uint8_t i = 0; i < 11; i++, x++) {
        word = (uint16_t)pgm_read_word(&Terminal11x16[c-0x20][i]);
        for (uint8_t j = 0; j < 16; j++) {
            writePixel(x, y+j, (word & 0x01) );
            word >>= 1;
        }
    }
}

static void draw_32x24_digit(uint8_t x, uint8_t y, uint8_t c) {
    uint32_t word = 0;

    for (uint8_t i = 0; i < 24; i++,x++) {
        if ( c > 9 )
            word = 0; // Blank
        else
            word = (uint32_t)pgm_read_dword(&DejaVuSansMonoBold[c][i]);
        for (uint8_t j = 0; j < 32; j++) {
            writePixel(x, y+j, (word & 0x01) );
            word >>= 1;
        }
    }
}


void splash( void ){
    draw_pgm_bmp(10, 15, star_bmp, star_width, star_height);
    draw_pgm_bmp(50, 20, star_bmp, star_width, star_height);
    draw_pgm_bmp(80, 35, star_bmp, star_width, star_height);
    draw_pgm_bmp(100, 50, star_bmp, star_width, star_height);
}



void ssd1306_init(void) {
    TWIInit();
    _delay_ms(100);

    // Init sequence for 128x64 OLED module
    ssd1306_command(SSD1306_DISPLAYOFF);                    // 0xAE

    ssd1306_command(SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
    ssd1306_command(0x80);                 // the suggested ratio 0x80

    ssd1306_command(SSD1306_SETMULTIPLEX);                  // 0xA8
    ssd1306_command(0x3F);

    ssd1306_command(SSD1306_SETDISPLAYOFFSET);              // 0xD3
    ssd1306_command(0x0);                                   // no offset

    ssd1306_command(SSD1306_SETSTARTLINE);// | 0x0);        // line #0

    ssd1306_command(SSD1306_CHARGEPUMP);                    // 0x8D
    ssd1306_command(0x14);  // using internal VCC

    ssd1306_command(SSD1306_MEMORYMODE);                    // 0x20
    ssd1306_command(0x00);          // 0x00 horizontal addressing

    ssd1306_command(SSD1306_SEGREMAP | 0x1); // rotate screen 180

    ssd1306_command(SSD1306_COMSCANDEC); // rotate screen 180

    ssd1306_command(SSD1306_SETCOMPINS);                    // 0xDA
    ssd1306_command(0x12);

    ssd1306_command(SSD1306_SETCONTRAST);                   // 0x81
    ssd1306_command(0xCF);

    ssd1306_command(SSD1306_SETPRECHARGE);                  // 0xd9
    ssd1306_command(0xF1);

    ssd1306_command(SSD1306_SETVCOMDETECT);                 // 0xDB
    ssd1306_command(0x40);

    ssd1306_command(SSD1306_DISPLAYALLON_RESUME);           // 0xA4

    ssd1306_command(SSD1306_NORMALDISPLAY);                 // 0xA6

    ssd1306_command(SSD1306_DEACTIVATE_SCROLL);

    ssd1306_command(SSD1306_DISPLAYON);                     //switch on OLED


    clear_display();
    //splash();
    ssd1306_display();
}

void ssd1306_displayoff( void ){
    ssd1306_command(SSD1306_DISPLAYOFF);
}

void ssd1306_displayon( void ){
    ssd1306_command(SSD1306_DISPLAYON);
}


void ssd1306_clear( void ){
    clear_display();
    ssd1306_display();
}



void ssd1306_DrawStringXY(char *characters, unsigned char x, unsigned char y ){
    char character;

    while ( (character = *characters) != '\0' ){
        draw_char(x, y, character );
        characters++;
        x+=6;  // Spacing ... */
    }
}


void ssd1306_DrawStringLargeXY(char *characters, unsigned char x, unsigned char y ){
    char character;

    //draw_large_char(x, y, 'X' );
    while ( (character = *characters) != '\0' ){
        draw_large_char(x, y, character );
        characters++;
        x+=12;  // Spacing ... */
    }
}

void ssd1306_DrawDigitsHugeXY(unsigned short value, unsigned char x, unsigned char y ){
    draw_32x24_digit(x+24,y, (value % 10) );
    if ( value >= 10){
        value /=10;
        draw_32x24_digit(x+0,y, (value % 10) );
    }
    else{
        draw_32x24_digit(x+0,y, 0xFF );  // Blank
    }

}

void ssd1306_DrawSingleDigitHugeXY(unsigned char value, unsigned char x, unsigned char y ){
    if ( value < 10){
        draw_32x24_digit(x,y, value);
    }
    else{
        draw_32x24_digit(x+0,y, 0xFF );  // Blank
    }
}

void ssd1306_DrawBatteryBars( unsigned char bars, unsigned char x, unsigned char y ){

    for (uint8_t i = 0; i < 4; i++) {
        if ( bars > i ){
            draw_6x8_rect( x,  y, 1 );
        }
        else{
            draw_6x8_rect( x,  y, 0 );
        }
        x += 6;
    }
}




