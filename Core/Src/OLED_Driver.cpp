#include "OLED_Driver.h"
#include "ASCII_Font.h"
#include "main.h"




uint8_t color_byte[2],color_fill_byte[2];
extern SPI_HandleTypeDef hspi1;

#ifdef __cplusplus
extern "C"  {
#endif



void OLED_Driver::Set_Color(uint16_t color)  {
  
  color_byte[0] = (uint8_t)(color >> 8);
  color_byte[1] = (uint8_t)(color & 0x00ff);
}


void OLED_Driver::Set_FillColor(uint16_t color)  {
  
  color_fill_byte[0] = (uint8_t)(color >> 8);
  color_fill_byte[1] = (uint8_t)(color & 0x00ff);
}
  

void OLED_Driver::Write_Command(uint8_t cmd)  {
  
  OLED_CS(GPIO_PIN_RESET);
  
#if  INTERFACE_4WIRE_SPI
  
  OLED_DC(GPIO_PIN_RESET);
  
  while(HAL_SPI_Transmit(&hspi1,&cmd,0x01,0x10) != HAL_OK);
  
  OLED_DC(GPIO_PIN_SET);
  
#elif INTERFACE_3WIRE_SPI
  
  uint8_t i;
	uint16_t hwData = 0;
	
  hwData = (uint16_t)cmd & ~0x0100;

	for(i = 0; i < 9; i ++) {
		OLED_SCK(GPIO_PIN_RESET);
    if(hwData & 0x0100) {
      OLED_DIN(GPIO_PIN_SET);
		}
    else  {
      OLED_DIN(GPIO_PIN_RESET);
		}
    OLED_SCK(GPIO_PIN_SET);
		hwData <<= 1;
	}

  
#endif
  
  OLED_CS(GPIO_PIN_SET);
}


void OLED_Driver::Write_Data(uint8_t dat) {
  
  OLED_CS(GPIO_PIN_RESET);
  
#if  INTERFACE_4WIRE_SPI
  
  OLED_DC(GPIO_PIN_SET);
  
  while(HAL_SPI_Transmit(&hspi1,&dat,0x01,0x10) != HAL_OK);
  
  OLED_DC(GPIO_PIN_RESET);
  
#elif INTERFACE_3WIRE_SPI
  
  uint8_t i;
	uint16_t hwData = 0;
	
  hwData = (uint16_t)dat | 0x0100;
	
	for(i = 0; i < 9; i ++) {
    OLED_SCK(GPIO_PIN_RESET);
		if(hwData & 0x0100) {
      OLED_DIN(GPIO_PIN_SET);
		}
    else  {
      OLED_DIN(GPIO_PIN_RESET);
		}
    OLED_SCK(GPIO_PIN_SET);
		hwData <<= 1;
	}
  
#endif
  
  OLED_CS(GPIO_PIN_SET);
  
}

void OLED_Driver::Write_Data(uint8_t* dat_p, uint16_t length) {
  
  OLED_CS(GPIO_PIN_RESET);
  
#if INTERFACE_4WIRE_SPI
  
  OLED_DC(GPIO_PIN_SET);
  
  while(HAL_SPI_Transmit(&hspi1,dat_p,length,0x10) != HAL_OK);
  
  OLED_DC(GPIO_PIN_RESET);
  
#elif INTERFACE_3WIRE_SPI
  
  uint8_t i,j;
	uint16_t hwData = 0;
	

  for(i = 0; i < length; i++) {
    
    hwData = (uint16_t)dat_p[i] | 0x0100;
    
    for(j = 0; j < 9; j ++) {
      OLED_SCK(GPIO_PIN_RESET);
      if(hwData & 0x0100) {
        OLED_DIN(GPIO_PIN_SET);
      } else {
        OLED_DIN(GPIO_PIN_RESET);
      }
      OLED_SCK(GPIO_PIN_SET);
      hwData <<= 1;
    }
  }
#endif
  
  OLED_CS(GPIO_PIN_SET);
  
}


void OLED_Driver::RAM_Address(void)  {
  
  Write_Command(0x15);
  Write_Data(0x00);
  Write_Data(0x7f);

  Write_Command(0x75);
  Write_Data(0x00);
  Write_Data(0x7f);
}


void OLED_Driver::Clear_Screen(void)  {
  
  int i,j;
  
  uint8_t clear_byte[] = {0x00, 0x00};
  RAM_Address();
  Write_Command(0x5C);
  for(i=0;i<128;i++)  {
    for(j=0;j<128;j++)  {
      Write_Data(clear_byte,2);//RAM data clear
    }
  }
}
  

void OLED_Driver::Fill_Color(uint16_t color)  {
  
  int i,j;

  RAM_Address();
  Write_Command(0x5C);
  Set_Color(color);
  for(i = 0; i < 128; i++)  {
    for(j = 0; j < 128; j++)  {
      Write_Data(color_byte,2);
    }
  }
}
  
void OLED_Driver::Set_Coordinate(uint16_t x, uint16_t y)  {

  if ((x >= SSD1351_WIDTH) || (y >= SSD1351_HEIGHT)) 
    return;
  //Set x and y coordinate
  Write_Command(SSD1351_CMD_SETCOLUMN);
  Write_Data(x);
  Write_Data(SSD1351_WIDTH-1);
  Write_Command(SSD1351_CMD_SETROW);
  Write_Data(y);
  Write_Data(SSD1351_HEIGHT-1);
  Write_Command(SSD1351_CMD_WRITERAM);
}
  
void OLED_Driver::Set_Address(uint8_t column, uint8_t row)  {
  
  Write_Command(SSD1351_CMD_SETCOLUMN);  
  Write_Data(column);	//X start 
  Write_Data(column);	//X end 
  Write_Command(SSD1351_CMD_SETROW); 
  Write_Data(row);	//Y start 
  Write_Data(row+7);	//Y end 
  Write_Command(SSD1351_CMD_WRITERAM); 
}
  
void OLED_Driver::Write_text(uint8_t dat) {
    
  uint8_t i;
    
  for(i=0;i<8;i++)  {
    if (dat & 0x01)	
      Write_Data(color_byte,2);
    else  {
      Write_Data(0x00);
      Write_Data(0x00);
    }
    dat >>= 1;
  }
}
  
void  OLED_Driver::Invert(bool v) {
  
  if (v)
    Write_Command(SSD1351_CMD_INVERTDISPLAY);
  else
    Write_Command(SSD1351_CMD_NORMALDISPLAY);
}

void OLED_Driver::Draw_Pixel(int16_t x, int16_t y)
{
  // Bounds check.
  if ((x >= SSD1351_WIDTH) || (y >= SSD1351_HEIGHT)) return;
  if ((x < 0) || (y < 0)) return;

  Set_Address(x, y);
  
  // transfer data
  Write_Data(color_byte,2);
  
}

void OLED_Driver::drawPixel(int16_t x, int16_t y, uint16_t color)
{
	Set_Color( color );
	Draw_Pixel( x, y );
}
  
  
void OLED_Driver::Device_Init(void) {

#if INTERFACE_3WIRE_SPI
  
  OLED_DC(GPIO_PIN_RESET);
  HAL_SPI_DeInit(&hspi1);
  SPI_GPIO_Init();

#endif

  OLED_CS(GPIO_PIN_RESET);

  OLED_RST(GPIO_PIN_RESET);
  HAL_Delay(500);
  OLED_RST(GPIO_PIN_SET);
  HAL_Delay(500);
    
  Write_Command(0xfd);	// command lock
  Write_Data(0x12);
  Write_Command(0xfd);	// command lock
  Write_Data(0xB1);

  Write_Command(0xae);	// display off
  Write_Command(0xa4); 	// Normal Display mode

  Write_Command(0x15);	//set column address
  Write_Data(0x00);     //column address start 00
  Write_Data(0x7f);     //column address end 95
  Write_Command(0x75);	//set row address
  Write_Data(0x00);     //row address start 00
  Write_Data(0x7f);     //row address end 63	

  Write_Command(0xB3);
  Write_Data(0xF1);

  Write_Command(0xCA);	
  Write_Data(0x7F);

  Write_Command(0xa0);  //set re-map & data format
  Write_Data(0x74);     //Horizontal address increment

  Write_Command(0xa1);  //set display start line
  Write_Data(0x00);     //start 00 line

  Write_Command(0xa2);  //set display offset
  Write_Data(0x00);

  Write_Command(0xAB);	
  Write_Command(0x01);	

  Write_Command(0xB4);	
  Write_Data(0xA0);	  
  Write_Data(0xB5);  
  Write_Data(0x55);    

  Write_Command(0xC1);	
  Write_Data(0xC8);	
  Write_Data(0x80);
  Write_Data(0xC0);

  Write_Command(0xC7);	
  Write_Data(0x0F);

  Write_Command(0xB1);	
  Write_Data(0x32);

  Write_Command(0xB2);	
  Write_Data(0xA4);
  Write_Data(0x00);
  Write_Data(0x00);

  Write_Command(0xBB);	
  Write_Data(0x17);

  Write_Command(0xB6);
  Write_Data(0x01);

  Write_Command(0xBE);
  Write_Data(0x05);

  Write_Command(0xA6);

  Clear_Screen();
  Write_Command(0xaf);	 //display on
}

  
// Draw a horizontal line ignoring any screen rotation.
void OLED_Driver::Draw_FastHLine(int16_t x, int16_t y, int16_t length) {
  // Bounds check
  if ((x >= SSD1351_WIDTH) || (y >= SSD1351_HEIGHT))
    return;

  // X bounds check
  if (x+length > SSD1351_WIDTH) {
    length = SSD1351_WIDTH - x - 1;
  }

  if (length < 0)
    return;

  // set location
  Write_Command(SSD1351_CMD_SETCOLUMN);
  Write_Data(x);
  Write_Data(x+length-1);
  Write_Command(SSD1351_CMD_SETROW);
  Write_Data(y);
  Write_Data(y);
  // fill!
  Write_Command(SSD1351_CMD_WRITERAM);

  for (uint16_t i=0; i < length; i++)
    Write_Data(color_byte,2);
}
  
  // Draw a vertical line ignoring any screen rotation.
void OLED_Driver::Draw_FastVLine(int16_t x, int16_t y, int16_t length)  {
  // Bounds check
  if ((x >= SSD1351_WIDTH) || (y >= SSD1351_HEIGHT))
    return;
  // X bounds check
  if (y+length > SSD1351_HEIGHT) {
    length = SSD1351_HEIGHT - y - 1;
  }
  if (length < 0)
    return;

  // set location
  Write_Command(SSD1351_CMD_SETCOLUMN);
  Write_Data(x);
  Write_Data(x);
  Write_Command(SSD1351_CMD_SETROW);
  Write_Data(y);
  Write_Data(y+length-1);
  // fill!
  Write_Command(SSD1351_CMD_WRITERAM);  
    
  for (uint16_t i=0; i < length; i++)
    Write_Data(color_byte,2);
}

void OLED_Driver::Display_Interface(void) {
  
  uint16_t i,color;
  RAM_Address();
  Write_Command(0x5C);
  for(i = 0 ; i < 128*63 ; i++)  {
    if((interface_1[i/8]>>(i%8))&0x01) {
      
      if(i<128*12)
        color = GREEN+(i<<11);
      else if(i<128*40)
        color = CYAN;
      else
        color = RED+i-128*40;
    }
    else  {
      color = BLACK;
    }
    Set_Color(color);
    Write_Data(color_byte,2);
  }

  for(i = 0 ; i < 128*64*2 ; i+=2)  {
    if(interface_2[i] != 0x00)
      color_byte[0] = interface_2[i];
    else
      color_byte[0] = interface_2[i];
    if(interface_2[i] != 0x00)
      color_byte[1] = 0xe0;
    else
      color_byte[1] = interface_2[i+1];
    Write_Data(color_byte,2);
  }
}

void OLED_Driver::Display_bmp(void) {
  
  uint16_t i = 0;
  RAM_Address();
  Write_Command(0x5C);
  
  for( ; i < 128*128*2; i+=2)  {
    color_byte[0] = gImage_bmp2[i];
    color_byte[1] = gImage_bmp2[i+1];
    Write_Data(color_byte,2);
  }
}

#ifdef __cplusplus
}
#endif


