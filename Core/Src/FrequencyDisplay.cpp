#include "FrequencyDisplay.h"


FrequencyDisplay::FrequencyDisplay( OLED_GFX* oled, Encoder* encoder, int frequency )
{
	_oled = oled;
	_encoder = encoder;
	_font = (GFXfont *)&FreeMono12pt7b;

	_oled->Device_Init();
	_oled->setFont( _font );
	_oled->setTextColor( RED );

	_frequency = frequency;
	_radix = 2;
	_mode = Mode::MODE_CHANGE_FREQUENCY;

	_color = RED;
	_bgcolor = BLACK;

	GFXglyph glyph = _font->glyph[0];
	_xadvance = glyph.xAdvance;
	_yadvance = _font->yAdvance;

	displayFrequency();
}
  
void FrequencyDisplay::change()
{
	if ( _mode == Mode::MODE_CHANGE_FREQUENCY )
		changeFrequency();
	else
		changeRadix();

	displayFrequency();

}

void FrequencyDisplay::changeMode()
{
	if ( _mode == Mode::MODE_CHANGE_FREQUENCY )
		_mode = Mode::MODE_CHANGE_RADIX;
	else
		_mode = Mode::MODE_CHANGE_FREQUENCY;
}

void FrequencyDisplay::changeFrequency()
{
	int mult = rpos[_radix].mult;

	if ( _encoder->isUp() )
		  _frequency += mult;
	else
		  _frequency -= mult;
}

void FrequencyDisplay::changeRadix()
{
	int pos = rpos[_radix].pos;
	_oled->fillRect( pos*_xadvance, 55, _xadvance, 4, _bgcolor );

	if ( _encoder->isUp() && _radix < MAX_RADIX-1)
		_radix++;
	else if ( _encoder->isDown() && _radix > 0 )
		_radix--;
}


void FrequencyDisplay::displayFrequency()
{
	  int millions = _frequency / 1000000;
	  int thousands = ( _frequency - millions * 1000000 ) / 1000;
	  int units = _frequency % 1000;

	  _oled->setCursor( 0, 50 );
	  _oled->setTextColor( _color, _bgcolor );

	  _oled->printf( "%02d.%03d.%02d ", millions, thousands, units/10 );

	  int pos = rpos[_radix].pos;
	  _oled->fillRect( pos*_xadvance, 55, _xadvance, 4, _color );


}
