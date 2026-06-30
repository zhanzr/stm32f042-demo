#include "lcd_st7735.h"
#include "main.h"

static inline void SimpleDelay(uint32_t d) {
  uint32_t t = d * 3;
  while (--t) {
    __NOP();
  }
}

static inline void RS_H(void) {
  HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_SET);
}

static inline void RS_L(void) {
  HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET);
}

static inline void E_H(void) {
  HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET);
}

static inline void E_L(void) {
  HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_RESET);
}

static inline void D4_H(void) {
  HAL_GPIO_WritePin(DB4_GPIO_Port, DB4_Pin, GPIO_PIN_SET);
}

static inline void D4_L(void) {
  HAL_GPIO_WritePin(DB4_GPIO_Port, DB4_Pin, GPIO_PIN_RESET);
}

static inline void D5_H(void) {
  HAL_GPIO_WritePin(DB5_GPIO_Port, DB5_Pin, GPIO_PIN_SET);
}

static inline void D5_L(void) {
  HAL_GPIO_WritePin(DB5_GPIO_Port, DB5_Pin, GPIO_PIN_RESET);
}

static inline void D6_H(void) {
  HAL_GPIO_WritePin(DB6_GPIO_Port, DB6_Pin, GPIO_PIN_SET);
}

static inline void D6_L(void) {
  HAL_GPIO_WritePin(DB6_GPIO_Port, DB6_Pin, GPIO_PIN_RESET);
}

static inline void D7_H(void) {
  HAL_GPIO_WritePin(DB7_GPIO_Port, DB7_Pin, GPIO_PIN_SET);
}

static inline void D7_L(void) {
  HAL_GPIO_WritePin(DB7_GPIO_Port, DB7_Pin, GPIO_PIN_RESET);
}

#define DC_H RS_H
#define DC_L RS_L
#define CS_H E_H
#define CS_L E_L
#define RST_H D4_H
#define RST_L D4_L
#define SCL_H D5_H
#define SCL_L D5_L
#define MOSI_H D7_H
#define MOSI_L D7_L

void soft_write_8bit(unsigned char dat) {
  for (uint8_t i = 0; i < 8; i++) {
    if ((dat & 0x80) != 0) {
      MOSI_H();
    } else {
      MOSI_L();
    }

    dat <<= 1;

    SCL_L();
    SCL_H();
  }
}

void LCD_WR_REG(unsigned int i) {
  CS_L();
  DC_L();

  soft_write_8bit(i);
  CS_H();
}

void LCD_WR_DATA(unsigned int i) {
  CS_L();
  DC_H();

  soft_write_8bit(i);
  CS_H();
}

void LCD_RESET(void) {
  RST_H();
  HAL_Delay(1);
  RST_L();
  HAL_Delay(100);
  RST_H();
  HAL_Delay(20);
}

void LCD_WriteData_16Bit(uint16_t Data) {
	  CS_L();

	  DC_H();
	 soft_write_8bit(Data>>8);
	 soft_write_8bit(Data);

	  CS_H();
}

void Lcd_WriteReg(uint8_t Index,uint8_t Data)
{
	LCD_WR_REG(Index);
  LCD_WR_DATA(Data);
}

void LCD_IO_WriteMultipleData(uint8_t *buf, uint32_t len) {
  CS_L();
  DC_H();
  for (uint32_t i = 0; i < len; ++i) {
    soft_write_8bit(*(buf + i));
  }
  CS_H();
}

void Lcd_Init(void) {
  LCD_RESET();

  LCD_WR_REG(0x11); // Exit Sleep
  HAL_Delay(120);

  LCD_WR_REG(0x21);

  LCD_WR_REG(0xB1);
  LCD_WR_DATA(0x05);
  LCD_WR_DATA(0x3A);
  LCD_WR_DATA(0x3A);

  LCD_WR_REG(0xB2);
  LCD_WR_DATA(0x05);
  LCD_WR_DATA(0x3A);
  LCD_WR_DATA(0x3A);

  LCD_WR_REG(0xB3);
  LCD_WR_DATA(0x05);
  LCD_WR_DATA(0x3A);
  LCD_WR_DATA(0x3A);
  LCD_WR_DATA(0x05);
  LCD_WR_DATA(0x3A);
  LCD_WR_DATA(0x3A);

  LCD_WR_REG(0xB4);
  LCD_WR_DATA(0x03);

  LCD_WR_REG(0xC0);
  LCD_WR_DATA(0x62);
  LCD_WR_DATA(0x02);
  LCD_WR_DATA(0x04);

  LCD_WR_REG(0xC1);
  LCD_WR_DATA(0xC0);

  LCD_WR_REG(0xC2);
  LCD_WR_DATA(0x0D);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0xC3);
  LCD_WR_DATA(0x8D);
  LCD_WR_DATA(0x6A);

  LCD_WR_REG(0xC4);
  LCD_WR_DATA(0x8D);
  LCD_WR_DATA(0xEE);

  LCD_WR_REG(0xC5);  /*VCOM*/
  LCD_WR_DATA(0x0E);

  LCD_WR_REG(0xE0);
  LCD_WR_DATA(0x10);
  LCD_WR_DATA(0x0E);
  LCD_WR_DATA(0x02);
  LCD_WR_DATA(0x03);
  LCD_WR_DATA(0x0E);
  LCD_WR_DATA(0x07);
  LCD_WR_DATA(0x02);
  LCD_WR_DATA(0x07);
  LCD_WR_DATA(0x0A);
  LCD_WR_DATA(0x12);
  LCD_WR_DATA(0x27);
  LCD_WR_DATA(0x37);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x0D);
  LCD_WR_DATA(0x0E);
  LCD_WR_DATA(0x10);

  LCD_WR_REG(0xE1);
  LCD_WR_DATA(0x10);
  LCD_WR_DATA(0x0E);
  LCD_WR_DATA(0x03);
  LCD_WR_DATA(0x03);
  LCD_WR_DATA(0x0F);
  LCD_WR_DATA(0x06);
  LCD_WR_DATA(0x02);
  LCD_WR_DATA(0x08);
  LCD_WR_DATA(0x0A);
  LCD_WR_DATA(0x13);
  LCD_WR_DATA(0x26);
  LCD_WR_DATA(0x36);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x0D);
  LCD_WR_DATA(0x0E);
  LCD_WR_DATA(0x10);

  LCD_WR_REG(0x3A);
  LCD_WR_DATA(0x05);

  LCD_WR_REG(0x36);
  LCD_WR_DATA(0xA8);//

  LCD_WR_REG(0x29);
}

/*************************************************
º¯ÊýÃû£ºLCD_Set_Region
¹¦ÄÜ£ºÉèÖÃlcdÏÔÊ¾ÇøÓò£¬ÔÚ´ËÇøÓòÐ´µãÊý¾Ý×Ô¶¯»»ÐÐ
Èë¿Ú²ÎÊý£ºxyÆðµãºÍÖÕµã
·µ»ØÖµ£ºÎÞ
*************************************************/
void Lcd_SetRegion(uint16_t x_start,uint16_t y_start,uint16_t x_end,uint16_t y_end)
{
	LCD_WR_REG(0x2a);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(x_start+1);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(x_end+1);

	LCD_WR_REG(0x2b);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(y_start+0x1A);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(y_end+0x1A);
	LCD_WR_REG(0x2c);

}

/*************************************************
º¯ÊýÃû£ºLCD_Set_XY
¹¦ÄÜ£ºÉèÖÃlcdÏÔÊ¾ÆðÊ¼µã
Èë¿Ú²ÎÊý£ºxy×ø±ê
·µ»ØÖµ£ºÎÞ
*************************************************/
void Lcd_SetXY(uint16_t x,uint16_t y)
{
  	Lcd_SetRegion(x,y,x,y);
}


/*************************************************
º¯ÊýÃû£ºLCD_DrawPoint
¹¦ÄÜ£º»­Ò»¸öµã
Èë¿Ú²ÎÊý£ºÎÞ
·µ»ØÖµ£ºÎÞ
*************************************************/
void Gui_DrawPoint(uint16_t x,uint16_t y,uint16_t Data)
{
	Lcd_SetRegion(x,y,x+1,y+1);
	LCD_WriteData_16Bit(Data);

}

/*****************************************
 º¯Êý¹¦ÄÜ£º¶ÁTFTÄ³Ò»µãµÄÑÕÉ«
 ³ö¿Ú²ÎÊý£ºcolor  µãÑÕÉ«Öµ
******************************************/
unsigned int Lcd_ReadPoint(uint16_t x,uint16_t y)
{
  unsigned int Data;
  Lcd_SetXY(x,y);

  //Lcd_ReadData();//¶ªµôÎÞÓÃ×Ö½Ú
  //Data=Lcd_ReadData();
  LCD_WR_DATA(Data);
  return Data;
}
/*************************************************
º¯ÊýÃû£ºLcd_Clear
¹¦ÄÜ£ºÈ«ÆÁÇåÆÁº¯Êý
Èë¿Ú²ÎÊý£ºÌî³äÑÕÉ«COLOR
·µ»ØÖµ£ºÎÞ
*************************************************/
void Lcd_Clear(uint16_t Color)
{
   unsigned int i,m;
   Lcd_SetRegion(0,0,X_MAX_PIXEL-1,Y_MAX_PIXEL-1);
   LCD_WR_REG(0x2C);
   for(i=0;i<X_MAX_PIXEL;i++)
    for(m=0;m<Y_MAX_PIXEL;m++)
    {
	  	LCD_WriteData_16Bit(Color);
    }
}

