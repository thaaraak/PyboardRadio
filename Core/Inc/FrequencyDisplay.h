#ifndef __FREQUENCY_DISPLAY_H
#define __FREQUENCY_DISPLAY_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#include "Encoder.h"
#include "OLED_Driver.h"
#include "OLED_GFX.h"
#include "gfxfont.h"
#include "FreeMono12pt7b.h"

struct radixpos
{
	int	pos;
	int mult;
};

#define MAX_RADIX	6

class FrequencyDisplay
{
  
  public:
      
    FrequencyDisplay( OLED_GFX* oled, Encoder* encoder, int frequency );

    void change();
    void changeMode();
    void changeFrequency();
    void changeRadix();
    void displayFrequency();

    void setColor( uint16_t color, uint16_t bgcolor ) { _color = color; _bgcolor = bgcolor; }
      
  private:
    
    enum Mode
    {
          MODE_CHANGE_FREQUENCY,
          MODE_CHANGE_RADIX
    };

    radixpos rpos[MAX_RADIX] =
    {
    		{ 8, 10 },
    		{ 7, 100 },
    		{ 5, 1000 },
    		{ 4, 10000 },
    		{ 3, 100000 },
    		{ 1, 1000000 }
    };

    OLED_GFX*	_oled;
    Encoder*	_encoder;
    GFXfont*	_font;

    int 		_frequency;
    int			_radix;
    Mode		_mode;
    uint16_t 	_color;
    uint16_t 	_bgcolor;

    uint8_t		_xadvance;
    uint8_t		_yadvance;
};


#endif
