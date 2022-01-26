//https://controllerstech.com/i2c-lcd-in-stm32/

/**
  ******************************************************************************
  * @file    LCD_i2c.h
  * @author  Jakub
  * Created on: 26 sty 2022
  * @brief   Simple HD44780 with PCF8574T driver library for STM32F7 using code from site: https://controllerstech.com/i2c-lcd-in-stm32/
  *			 and AW repository (https://github.com/adrianwojcikpp/NUCLEO_F746ZG_DEMO)
  *          NOTE!: This code provides only WRITE features, no READ features.
  *
  ******************************************************************************
  */

#ifndef INC_LCD_I2C_H_
#define INC_LCD_I2C_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"


/* Typedef -------------------------------------------------------------------*/
#define LCD_I2CType I2C_HandleTypeDef*

typedef struct {
  LCD_I2CType I2C;
  uint8_t Address;
  uint32_t Timeout;
} LCD_HandleTypeDef;


/* Define --------------------------------------------------------------------*/
#define LCD_ADDRESS (0x27 << 1)

/* Define --------------------------------------------------------------------*/
#define LCD_CLEAR_DISPLAY 0x01

#define LCD_RETURN_HOME 0x02

#define LCD_ENTRY_MODE_SET 0x04
#define LCD_OPT_S   0x01				// Shift entire display to right
#define LCD_OPT_INC 0x02			 	// Cursor increment

#define LCD_DISPLAY_ON_OFF_CONTROL 0x08
#define LCD_OPT_D  0x04					// Turn on display
#define LCD_OPT_C  0x02					// Turn on cursor
#define LCD_OPT_B  0x01					// Turn on cursor blink

#define LCD_CURSOR_DISPLAY_SHIFT 0x10	// Move and shift cursor
#define LCD_OPT_SC 0x08
#define LCD_OPT_RL 0x04

#define LCD_FUNCTION_SET 0x20
#define LCD_OPT_DL 0x10					// Set interface data length
#define LCD_OPT_N  0x08					// Set number of display lines
#define LCD_OPT_F  0x04					// Set alternate font
#define LCD_SETCGRAM_ADDR  0x040
#define LCD_SET_DDRAM_ADDR  0x80	// Set DDRAM address


/* Public function prototypes ------------------------------------------------*/
void lcd_init (LCD_HandleTypeDef* hLCD);

void lcd_send_cmd (LCD_HandleTypeDef* hLCD, char cmd);

void lcd_send_data (LCD_HandleTypeDef* hLCD, char data);

void lcd_SetCursor(LCD_HandleTypeDef* hLCD, uint8_t row, uint8_t col);

void lcd_send_string (LCD_HandleTypeDef* hLCD, char *str, uint8_t row, uint8_t col);

#endif /* INC_LCD_I2C_H_ */
