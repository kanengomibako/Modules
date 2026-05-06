#ifndef _DISPLAY_H
#define _DISPLAY_H

void OLED_init(void);

void OLED_DisplayBuffer(void);
void OLED_clear(void);
void OLED_setpos(uint8_t x, uint8_t y);
void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t white);

void OLED_write(char c);
void OLED_print(char* str);
void OLED_print_XY(char* str, uint8_t x, uint8_t y);

void OLED_print25x32(char* str);

void OLED_DrawCursor(uint8_t x, uint8_t y);
void OLED_DrawPeriod(uint8_t x, uint8_t y);

void Display_Volt(uint32_t);
void Display_Ohm(uint32_t);

void Display_main(void);
void Display_ADC_Calibration(void);
void Display_Ohm_Calibration(void);
void Display_Continuity_Check_Setting(void);

#endif  // _DISPLAY_H
