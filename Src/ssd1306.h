#ifndef _SSD1306_H_
#define _SSD1306_H_

void ssd1306_init(void);
void ssd1306_display(void);
void ssd1306_clear(void);

void ssd1306_displayon(void);
void ssd1306_displayoff(void);

void ssd1306_DrawStringXY(char *characters, unsigned char x, unsigned char y );
void ssd1306_DrawStringLargeXY(char *characters, unsigned char x, unsigned char y );
void ssd1306_DrawDigitsHugeXY(unsigned short value, unsigned char x, unsigned char y );
void ssd1306_DrawSingleDigitHugeXY(unsigned char value, unsigned char x, unsigned char y );

void ssd1306_DrawBatteryBars( unsigned char bars, unsigned char x, unsigned char y );

void splash( void );

#endif // _SSD1306_H_
