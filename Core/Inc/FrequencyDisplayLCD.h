#ifndef __FREQUENCY_DISPLAYLCD_H
#define __FREQUENCY_DISPLAYLCD_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#include "Encoder.h"
#include "LiquidCrystal_I2C.h"



#define MAX_RADIX	6

class FrequencyDisplayLCD
{
  
  public:
      
    FrequencyDisplayLCD( LiquidCrystal_I2C* lcd, Encoder* encoder, int frequency );

    void change();
    void changeMode();
    void changeFrequency();
    void changeRadix();
    void displayFrequency();
    void displaySSB( const char* ssb );

    int getFrequency() { return _frequency; }


  private:
    
    struct radixpos
    {
    	int	pos;
    	int mult;
    };

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

    LiquidCrystal_I2C*	_lcd;
    Encoder*	_encoder;

    int 		_frequency;
    int			_radix;
    Mode		_mode;
};


#endif
