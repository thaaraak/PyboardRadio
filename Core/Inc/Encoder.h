#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>


class Encoder
{
  
  public:
      
    Encoder( GPIO_TypeDef* clkBase, uint16_t clkPin, GPIO_TypeDef* dtBase, uint16_t dtPin );
      
    void clkInterrupt();
    void dtInterrupt();

    int getEncoderValue() { return _encoderValue; }

    bool hasChanged() { return _lastEncoderValue != _encoderValue; }
    bool isUp() { return _encoderValue > _lastEncoderValue; }
    bool isDown() { return _encoderValue < _lastEncoderValue; }

    void reset() { _lastEncoderValue = _encoderValue; }
    
  private:
    
    volatile int dtFlag = 0;
    volatile int clkFlag = 0;

    volatile int _lastEncoderValue = 0;
    volatile int _encoderValue;
    
    GPIO_TypeDef* _clkBase;
    uint16_t _clkPin;
    GPIO_TypeDef* _dtBase;
    uint16_t _dtPin;


};


#endif
