#ifndef __OLED_DRIVER_H
#define __OLED_DRIVER_H


#include "Adafruit_GFX.h"
#include "stm32f4xx_hal.h"

#define SSD1351_WIDTH   128
#define SSD1351_HEIGHT  128

#define SSD1351_CMD_SETCOLUMN       0x15
#define SSD1351_CMD_SETROW          0x75
#define SSD1351_CMD_WRITERAM        0x5C
#define SSD1351_CMD_READRAM         0x5D
#define SSD1351_CMD_SETREMAP        0xA0
#define SSD1351_CMD_STARTLINE       0xA1
#define SSD1351_CMD_DISPLAYOFFSET 	0xA2
#define SSD1351_CMD_DISPLAYALLOFF 	0xA4
#define SSD1351_CMD_DISPLAYALLON  	0xA5
#define SSD1351_CMD_NORMALDISPLAY 	0xA6
#define SSD1351_CMD_INVERTDISPLAY 	0xA7
#define SSD1351_CMD_FUNCTIONSELECT 	0xAB
#define SSD1351_CMD_DISPLAYOFF      0xAE
#define SSD1351_CMD_DISPLAYON     	0xAF
#define SSD1351_CMD_PRECHARGE       0xB1
#define SSD1351_CMD_DISPLAYENHANCE  0xB2
#define SSD1351_CMD_CLOCKDIV        0xB3
#define SSD1351_CMD_SETVSL          0xB4
#define SSD1351_CMD_SETGPIO 		    0xB5
#define SSD1351_CMD_PRECHARGE2      0xB6
#define SSD1351_CMD_SETGRAY         0xB8
#define SSD1351_CMD_USELUT          0xB9
#define SSD1351_CMD_PRECHARGELEVEL 	0xBB
#define SSD1351_CMD_VCOMH           0xBE
#define SSD1351_CMD_CONTRASTABC     0xC1
#define SSD1351_CMD_CONTRASTMASTER  0xC7
#define SSD1351_CMD_MUXRATIO        0xCA
#define SSD1351_CMD_COMMANDLOCK     0xFD
#define SSD1351_CMD_HORIZSCROLL     0x96
#define SSD1351_CMD_STOPSCROLL      0x9E
#define SSD1351_CMD_STARTSCROLL     0x9F


#define swap(a, b) { uint16_t t = a; a = b; b = t; }

#define OLED_RST(x)   HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, x)
#define OLED_DC(x)    HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, x)
#define OLED_CS(x)    HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, x)

#if INTERFACE_3WIRE_SPI

#define OLED_SCK(x)   HAL_GPIO_WritePin(OLED_SCK_GPIO_Port, OLED_SCK_Pin, x)
#define OLED_DIN(x)   HAL_GPIO_WritePin(OLED_DIN_GPIO_Port, OLED_DIN_Pin, x)

#endif

#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF


extern uint8_t color_byte[2];
extern uint8_t color_fill_byte[2];

#ifdef __cplusplus
extern "C" {
#endif

  class OLED_Driver : public virtual Adafruit_GFX
  {
    
    public: 
      OLED_Driver(void) : Adafruit_GFX( SSD1351_WIDTH, SSD1351_HEIGHT ) {}

      void Device_Init(void);
      void Clear_Screen(void);
      void Fill_Color(uint16_t color);
      void Set_Coordinate(uint16_t x, uint16_t y);

      void Write_text(uint8_t data1);
      void Set_Address(uint8_t column, uint8_t row);
      
      void Set_Color(uint16_t color);
      void Set_FillColor(uint16_t color);
    
      void Invert(bool v);
      void Draw_Pixel(int16_t x, int16_t y);
    
      void Write_Data(uint8_t dat);
      void Write_Data(uint8_t* dat_p, uint16_t length);
    
      void Draw_FastHLine(int16_t x, int16_t y, int16_t length);
      void Draw_FastVLine(int16_t x, int16_t y, int16_t length);
      void Write_Command(uint8_t data1);
      
      void Display_bmp(void);
      void Display_Interface(void);
      
      virtual void drawPixel(int16_t x, int16_t y, uint16_t color);


    private:  
      
      void RAM_Address(void);

  };



#ifdef __cplusplus
}
#endif
  
#endif

