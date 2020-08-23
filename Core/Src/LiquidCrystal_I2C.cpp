// Based on the work by DFRobot

#include "LiquidCrystal_I2C.h"
#include <stdint.h>



// When the display powers up, it is configured as follows:
//
// 1. Display clear
// 2. Function set: 
//    DL = 1; 8-bit interface data 
//    N = 0; 1-line display 
//    F = 0; 5x8 dot character font 
// 3. Display on/off control: 
//    D = 0; Display off 
//    C = 0; Cursor off 
//    B = 0; Blinking off 
// 4. Entry mode set: 
//    I/D = 1; Increment by 1
//    S = 0; No shift 
//
// Note, however, that resetting the Arduino doesn't reset the LCD, so we
// can't assume that its in that state when a sketch starts (and the
// LiquidCrystal constructor is called).

LiquidCrystal_I2C::LiquidCrystal_I2C(I2C_HandleTypeDef* hi2c, uint8_t addr, uint8_t lcd_cols,uint8_t lcd_rows)
{
  _hi2c = hi2c;
  _addr = addr;
  _cols = lcd_cols;
  _rows = lcd_rows;
  _backlightval = LCD_NOBACKLIGHT;
}

void LiquidCrystal_I2C::init(){
	init_priv();
}

void LiquidCrystal_I2C::init_priv()
{
	_displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
	begin(_cols, _rows);  
}

void LiquidCrystal_I2C::begin(uint8_t cols, uint8_t lines, uint8_t dotsize)
{
	if (lines > 1) {
		_displayfunction |= LCD_2LINE;
	}
	_numlines = lines;

	// for some 1 line displays you can select a 10 pixel high font
	if ((dotsize != 0) && (lines == 1)) {
		_displayfunction |= LCD_5x10DOTS;
	}

	// SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
	// according to datasheet, we need at least 40ms after power rises above 2.7V
	// before sending commands.
	HAL_Delay(50);
  
	// Now we pull both RS and R/W low to begin commands
	expanderWrite(_backlightval);	// reset expander and turn backlight off (Bit 8 =1)
	HAL_Delay(1000);

  	//put the LCD into 4 bit mode
	// this is according to the hitachi HD44780 datasheet
	// figure 24, pg 46

	pulseEnable(0x03 << 4);
	pulseEnable(0x03 << 4);
	pulseEnable(0x03 << 4);
	pulseEnable(0x02 << 4);


	// set # lines, font size, etc.
	command(LCD_FUNCTIONSET | _displayfunction);  
	
	// turn the display on with no cursor or blinking default
	_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
	display();
	
	// clear it off
	clear();
	
	// Initialize to default text direction (for roman languages)
	_displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
	
	// set the entry mode
	command(LCD_ENTRYMODESET | _displaymode);
	
	home();
  
}

/********** high level commands, for the user! */
void LiquidCrystal_I2C::clear(){
	command(LCD_CLEARDISPLAY);// clear display, set cursor position to zero
	HAL_Delay(2);  // this command takes a long time!
}

void LiquidCrystal_I2C::home(){
	command(LCD_RETURNHOME);  // set cursor position to zero
	HAL_Delay(2);  // this command takes a long time!
}

void LiquidCrystal_I2C::setCursor(uint8_t col, uint8_t row){
	int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
	if ( row > _numlines ) {
		row = _numlines-1;    // we count rows starting w/0
	}
	command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

// Turn the display on/off (quickly)
void LiquidCrystal_I2C::noDisplay() {
	_displaycontrol &= ~LCD_DISPLAYON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void LiquidCrystal_I2C::display() {
	_displaycontrol |= LCD_DISPLAYON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turns the underline cursor on/off
void LiquidCrystal_I2C::noCursor() {
	_displaycontrol &= ~LCD_CURSORON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void LiquidCrystal_I2C::cursor() {
	_displaycontrol |= LCD_CURSORON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turn on and off the blinking cursor
void LiquidCrystal_I2C::noBlink() {
	_displaycontrol &= ~LCD_BLINKON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void LiquidCrystal_I2C::blink() {
	_displaycontrol |= LCD_BLINKON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// These commands scroll the display without changing the RAM
void LiquidCrystal_I2C::scrollDisplayLeft(void) {
	command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void LiquidCrystal_I2C::scrollDisplayRight(void) {
	command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void LiquidCrystal_I2C::leftToRight(void) {
	_displaymode |= LCD_ENTRYLEFT;
	command(LCD_ENTRYMODESET | _displaymode);
}

// This is for text that flows Right to Left
void LiquidCrystal_I2C::rightToLeft(void) {
	_displaymode &= ~LCD_ENTRYLEFT;
	command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'right justify' text from the cursor
void LiquidCrystal_I2C::autoscroll(void) {
	_displaymode |= LCD_ENTRYSHIFTINCREMENT;
	command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'left justify' text from the cursor
void LiquidCrystal_I2C::noAutoscroll(void) {
	_displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
	command(LCD_ENTRYMODESET | _displaymode);
}

// Allows us to fill the first 8 CGRAM locations
// with custom characters
void LiquidCrystal_I2C::createChar(uint8_t location, uint8_t charmap[]) {
	location &= 0x7; // we only have 8 locations 0-7
	command(LCD_SETCGRAMADDR | (location << 3));
	for (int i=0; i<8; i++) {
		writeI2C(charmap[i]);
	}
}

// Turn the (optional) backlight off/on
void LiquidCrystal_I2C::noBacklight(void) {
	_backlightval=LCD_NOBACKLIGHT;
	expanderWrite(0);
}

void LiquidCrystal_I2C::backlight(void) {
	_backlightval=LCD_BACKLIGHT;
	expanderWrite(0);
}



/*********** mid level commands, for sending data/cmds */

inline void LiquidCrystal_I2C::command(uint8_t value) {
	send(value, 0);
}


/************ low level data pushing commands **********/

void LiquidCrystal_I2C::printf(const char* fmt, ...)
{
    char buff[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buff, sizeof(buff), fmt, args);

    char* c = buff;
    while ( *c )
    	putchar( *c++ );

    va_end(args);
}

void LiquidCrystal_I2C::putchar( uint8_t data )
{
	send( data, Rs );
}

void LiquidCrystal_I2C::send(uint8_t value, uint8_t mode)
{
	uint8_t highnib=value&0xf0;
	uint8_t lownib=(value<<4)&0xf0;
    pulseEnable((highnib)|mode);
	pulseEnable((lownib)|mode);
}


void LiquidCrystal_I2C::expanderWrite(uint8_t _data)
{
	writeI2C((int)(_data) | _backlightval);
}

// Write the data with the Enable bit first set and
// then repeat the write with Enable unset

void LiquidCrystal_I2C::pulseEnable(uint8_t _data)
{
	writeI2C(_data | En | _backlightval);
	writeI2C((_data & ~En) | _backlightval );
}


uint8_t LiquidCrystal_I2C::writeI2C(uint8_t data)
{
	HAL_StatusTypeDef status = HAL_OK;

	while (HAL_I2C_IsDeviceReady(_hi2c, (uint16_t)(_addr<<1), 3, 100) != HAL_OK) { }


	status = HAL_I2C_Master_Transmit(
			_hi2c,
			(uint8_t)(_addr<<1),
			&data,
			1,
			100
			);

    return status;
}

