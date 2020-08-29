#include "FrequencyDisplayLCD.h"


FrequencyDisplayLCD::FrequencyDisplayLCD( LiquidCrystal_I2C* lcd, Encoder* encoder, int frequency )
{
	_lcd = lcd;
	_encoder = encoder;

	_frequency = frequency;
	_radix = 2;
	_mode = Mode::MODE_CHANGE_FREQUENCY;

	displayFrequency();
}
  
void FrequencyDisplayLCD::change()
{
	if ( _mode == Mode::MODE_CHANGE_FREQUENCY )
		changeFrequency();
	else
		changeRadix();

	displayFrequency();

}

void FrequencyDisplayLCD::changeMode()
{
	if ( _mode == Mode::MODE_CHANGE_FREQUENCY )
		_mode = Mode::MODE_CHANGE_RADIX;
	else
		_mode = Mode::MODE_CHANGE_FREQUENCY;
}

void FrequencyDisplayLCD::changeFrequency()
{
	int mult = rpos[_radix].mult;

	if ( _encoder->isUp() )
		  _frequency += mult;
	else
		  _frequency -= mult;
}

void FrequencyDisplayLCD::changeRadix()
{
	int pos = rpos[_radix].pos;
//	_oled->fillRect( pos*_xadvance, 55, _xadvance, 4, _bgcolor );

	if ( _encoder->isUp() && _radix < MAX_RADIX-1)
		_radix++;
	else if ( _encoder->isDown() && _radix > 0 )
		_radix--;
}


void FrequencyDisplayLCD::displayFrequency()
{
	  int millions = _frequency / 1000000;
	  int thousands = ( _frequency - millions * 1000000 ) / 1000;
	  int units = _frequency % 1000;

	  _lcd->setCursor( 0, 0 );
	  _lcd->printf( "%02d.%03d.%02d ", millions, thousands, units/10 );

//	  int pos = rpos[_radix].pos;
//	  _oled->fillRect( pos*_xadvance, 55, _xadvance, 4, _color );


}

void FrequencyDisplayLCD::displaySSB( const char* ssb )
{
	  _lcd->setCursor( 12, 1 );
	  _lcd->printf( ssb );

}
