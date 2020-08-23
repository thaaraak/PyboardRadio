//YWROBOT
#ifndef LiquidCrystal_I2C_h
#define LiquidCrystal_I2C_h

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

#define LCD_BASE_ADDRESS 0x27

#define En 0b00000100  // Enable bit
#define Rw 0b00000010  // Read/Write bit
#define Rs 0b00000001  // Register select bit

class LiquidCrystal_I2C
{
public:
  LiquidCrystal_I2C(I2C_HandleTypeDef* hi2c, uint8_t addr = LCD_BASE_ADDRESS, uint8_t lcd_cols = 16, uint8_t lcd_rows = 2);
  void begin(uint8_t cols, uint8_t rows, uint8_t charsize = LCD_5x8DOTS );
  void clear();
  void home();
  void noDisplay();
  void display();
  void noBlink();
  void blink();
  void noCursor();
  void cursor();
  void scrollDisplayLeft();
  void scrollDisplayRight();
  void printLeft();
  void printRight();
  void leftToRight();
  void rightToLeft();
  void shiftIncrement();
  void shiftDecrement();
  void noBacklight();
  void backlight();
  void autoscroll();
  void noAutoscroll(); 
  void createChar(uint8_t, uint8_t[]);
  void setCursor(uint8_t, uint8_t); 
  void command(uint8_t);
  void init();
  void printf(const char* fmt, ...);
  void putchar( uint8_t c );


private:
  void init_priv();
  void send(uint8_t, uint8_t);
  void expanderWrite(uint8_t);
  void pulseEnable(uint8_t);
  uint8_t writeI2C(uint8_t data);

  I2C_HandleTypeDef* _hi2c;
  uint8_t _addr;
  uint8_t _displayfunction;
  uint8_t _displaycontrol;
  uint8_t _displaymode;
  uint8_t _numlines;
  uint8_t _cols;
  uint8_t _rows;
  uint8_t _backlightval;
};

#endif
