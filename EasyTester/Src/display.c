#include "common.h"
#include "ch32v00X.h"
#include "display.h"
#include "font.h"
#include "measure.h"

volatile uint8_t SSD1306_Buffer[SSD1306_BUFFER_SIZE] = {0xff, 0xff};
uint8_t cursorX, cursorY;
extern int8_t volt_enable;
extern int8_t ohm_enable;
extern int8_t buzzer_enable;
extern int16_t Continuity_Check_Threshold[];
extern int16_t Continuity_Check_Freq[];
extern int8_t ADC_Calibration_volt[];
extern float ADC_Calibration_offset;
extern float ADC_Calibration_slope;
extern float Resistance_Correction_Factor[];
extern uint8_t Cursor_Position;
extern uint8_t Ohm_Calibration_sequence;
extern float ohm_Calibration_offset[]; 
extern float ohm_Calibration_slope[];

// 初期化データ https://github.com/rotura/CH32V003-SSD1306-OLED-Library より転載
const uint8_t ssd1306_init_sequence [] = {	// Initialization Sequence
	0xAE,			// Set Display ON/OFF - AE=OFF, AF=ON
	0xD5, 0xF0,		// Set display clock divide ratio/oscillator frequency, set divide ratio
	0xA8, 0x3F,		// Set multiplex ratio (1 to 64) ... (height - 1)
	0xD3, 0x00,		// Set display offset. 00 = no offset
	0x40 | 0x00,	// Set start line address, at 0.
	0x8D, 0x14,		// Charge Pump Setting, 14h = Enable Charge Pump
	0x20, 0x00,		// Set Memory Addressing Mode - 00=Horizontal, 01=Vertical, 10=Page, 11=Invalid
	0xA0 | 0x01,	// Set Segment Re-map
	0xC8,			// Set COM Output Scan Direction
	0xDA, 0x12,		// Set COM Pins Hardware Configuration - 128x32:0x02, 128x64:0x12
	0x81, 0x3F,		// Set contrast control register - 0x01 to 0xFF - Default: 0x3F
	0xD9, 0x22,		// Set pre-charge period (0x22 or 0xF1)
	0xDB, 0x20,		// Set Vcomh Deselect Level - 0x00: 0.65 x VCC, 0x20: 0.77 x VCC (RESET), 0x30: 0.83 x VCC
	0xA4,			// Entire Display ON (resume) - output RAM to display
	0xA6,			// Set Normal/Inverse Display mode. A6=Normal; A7=Inverse
	0x2E,			// Deactivate Scroll command
	0xAF,			// Set Display ON/OFF - AE=OFF, AF=ON
	0x22, 0x00, 0x3f,	// Set Page Address (start,end) 0 - 63
	0x21, 0x00,	0x7f,	// Set Column Address (start,end) 0 - 127
};

// OLED初期化
void OLED_init(void) {
    while(I2C1->STAR2 & I2C_STAR2_BUSY);
    I2C_GenerateSTART(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(I2C1, OLED_ADDR, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  
    I2C_SendData(I2C1, OLED_CMD_MODE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    for (int i = 0; i < sizeof(ssd1306_init_sequence); i++) {
        I2C_SendData(I2C1, ssd1306_init_sequence[i]);
        while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    }

    I2C_GenerateSTOP(I2C1, ENABLE);
}

// 画面バッファを転送 DMA不使用
void OLED_DisplayBuffer(void) {
    while(I2C1->STAR2 & I2C_STAR2_BUSY);
    I2C_GenerateSTART(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(I2C1, OLED_ADDR, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  
    I2C_SendData(I2C1, OLED_DAT_MODE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    for (int i = 0; i < SSD1306_BUFFER_SIZE; i++) {
        I2C_SendData(I2C1, SSD1306_Buffer[i]);
        while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    }

    I2C_GenerateSTOP(I2C1, ENABLE);
}

// 画面バッファクリア
void OLED_clear(void) {
    for(uint16_t i = 0; i < SSD1306_BUFFER_SIZE; i++) {
        SSD1306_Buffer[i] = 0;
    } 
}

// XY座標セット
void OLED_setpos(uint8_t x, uint8_t y) {
    cursorX = x;
    cursorY = y;
}

// 1ドット描画
void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t white) {
    if(x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
        return;
    }

    if(white) {
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
    } else { 
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
    }
}

// 1文字描画（5x8フォント）
void OLED_plotChar(char c) {
    uint16_t pos;
    for(short i=0 ; i < 5; i++){
        pos = cursorX + i + cursorY / 8 * SSD1306_WIDTH;
        if (pos < SSD1306_BUFFER_SIZE) SSD1306_Buffer[pos] = font5x8[c-32][i];
    }

    // 空白を1ピクセル分描画
    pos = cursorX + 5 + cursorY / 8 * SSD1306_WIDTH;
    if (pos < SSD1306_BUFFER_SIZE) SSD1306_Buffer[pos] = 0;
}

// 文字描画（5x8フォント）
void OLED_write(char c) {
    c = c & 0x7F; // ASCIIコードの範囲に限定 (最上位ビットを無視)
    
    if (c >= 32) {
        OLED_plotChar(c);
        cursorX += 6;
    } 
}

// 文字列描画（5x8フォント）
void OLED_print(char* str) {
    while (*str) {
        OLED_write(*str++); // 1文字書き込んでポインタを進める
    }
}

// 座標指定して文字列描画（5x8フォント）
void OLED_print_XY(char* str, uint8_t x, uint8_t y) {
    OLED_setpos(x, y);
    OLED_print(str);
}

// 1文字描画（25x32フォント）
void OLED_plotChar25x32(char c)
{
    // 数字とそれ以外の文字を判定
    if ((unsigned)(c - 0x30) < 10) {
        c = c - 0x30;
    } 
    else {
        switch (c) {
            case 0x6b: c = 10; break; // k
            case 0x4d: c = 11; break; // M
            case 0x56: c = 12; break; // V
            case 0x4c: c = 13; break; // L
            case 0x4f: c = 0;  break; // O
            case 0x2e: c = 14; break; // .
            case 0x57: c = 15; break; // W->ohm
            default:   c = 16; break; // スペース
        }
    }

    for (int i = 0; i < 25; i++) {
        uint32_t col = font25x32[c][i];
        int pos = cursorX + i + cursorY / 8 * SSD1306_WIDTH;
        if (pos + 3*SSD1306_WIDTH < SSD1306_BUFFER_SIZE) {
            SSD1306_Buffer[pos + 0*SSD1306_WIDTH] = (uint8_t)(col >> 0);
            SSD1306_Buffer[pos + 1*SSD1306_WIDTH] = (uint8_t)(col >> 8);
            SSD1306_Buffer[pos + 2*SSD1306_WIDTH] = (uint8_t)(col >> 16);
            SSD1306_Buffer[pos + 3*SSD1306_WIDTH] = (uint8_t)(col >> 24);
        }
    }
}

// 文字描画（25x32フォント）
void OLED_write25x32(char c) {
    c = c & 0x7F; // ASCIIコードの範囲に限定 (最上位ビットを無視)
    OLED_plotChar25x32(c);
    cursorX += 29;
}

// 文字列描画（25x32フォント）
void OLED_print25x32(char* str) {
    while(*str) {
        OLED_write25x32(*str++);
    }
}

// カーソル描画
void OLED_DrawCursor(uint8_t x, uint8_t y) {
    OLED_setpos(x, y);
    for(uint8_t i = 0; i < 7; i++) OLED_DrawPixel(cursorX, cursorY + i, 1);
    for(uint8_t i = 0; i < 5; i++) OLED_DrawPixel(cursorX + 1, cursorY + 1 + i, 1);
    for(uint8_t i = 0; i < 3; i++) OLED_DrawPixel(cursorX + 2, cursorY + 2 + i, 1);
    OLED_DrawPixel(cursorX + 3, cursorY + 3, 1);
}

// 小数点描画
void OLED_DrawPeriod(uint8_t x, uint8_t y) {
    for(uint8_t i = 0; i < 4; i++){
        OLED_DrawPixel(x, y+25 + i, 1);
        OLED_DrawPixel(x+9, y+25 + i, 1);
    }
    for(uint8_t i = 0; i < 6; i++){
        OLED_DrawPixel(x+1, y+24 + i, 1);
        OLED_DrawPixel(x+8, y+24 + i, 1);
    }
    for(uint8_t i = 0; i < 8; i++){
        OLED_DrawPixel(x+2, y+23 + i, 1);
        OLED_DrawPixel(x+7, y+23 + i, 1);
    }
    for(uint8_t i = 0; i < 10; i++){
        OLED_DrawPixel(x+3, y+22 + i, 1);
        OLED_DrawPixel(x+4, y+22 + i, 1);
        OLED_DrawPixel(x+5, y+22 + i, 1);
        OLED_DrawPixel(x+6, y+22 + i, 1);
    }
    cursorX += 14;
}

// 電圧表示
void Display_Volt(uint32_t v) {
    char str[30]; // 一時使用文字列
    OLED_setpos(1, 0);

    if (v > MAX_VOLTAGE) { // 測定範囲外
        sprintf(str, " O.L ");
        OLED_print25x32(str);
    }
    else { // **.**V で表示
        sprintf(str, "%2u", v / 1000);
        OLED_print25x32(str);
        OLED_DrawPeriod(1+25+4+25+4, 0);
        sprintf(str, "%02u", (v % 1000) / 10);
        OLED_print25x32(str);
    }

    OLED_setpos(102, 32);
    OLED_print25x32("V");
}

// 抵抗値表示
void Display_Ohm(uint32_t r) {
    char str[30]; // 一時使用文字列
    char unit[30]; // 一時使用文字列
    uint32_t rp; // 四捨五入計算用

    if (r < 1000) {                              // ***Ω
        sprintf(str, "%4u", r);
        sprintf(unit, " W");
    }
    else if ((rp = r + 5) < 10000) {             // *.**k
        sprintf(str, "%u.%02u", rp/1000, (rp%1000)/10);
        sprintf(unit, "kW");
    }
    else if ((rp = r + 50) < 100000) {           // **.*k
        sprintf(str, "%2u.%u", rp/1000, (rp%1000)/100);
        sprintf(unit, "kW");
    }
    else if ((rp = r + 500) < 1000000) {         // ***k
        sprintf(str, "%4u", rp/1000);
        sprintf(unit, "kW");
    }
    else if ((rp = r + 5000) < MAX_RESISTANCE) { // **.*M
        sprintf(str, "%u.%02u", rp/1000000, (rp%1000000)/10000);
        sprintf(unit, "MW");
    }
    else {                                       // O.L
        sprintf(str, " O.L ");
        sprintf(unit, " W");
    }

    OLED_setpos(8, 0);
    OLED_print25x32(str);
    OLED_setpos(66, 32);
    OLED_print25x32(unit);
}

// メイン画面 ON OFF 表示
void Display_main(void) {
    if (volt_enable) OLED_print_XY("VOLT  ON", 5, 40);
    else OLED_print_XY("VOLT OFF", 5, 40);
    if (ohm_enable) OLED_print_XY("OHM   ON", 5, 48);
    else OLED_print_XY("OHM  OFF", 5, 48);
    if (buzzer_enable) OLED_print_XY("BZR   ON", 5, 56);
    else OLED_print_XY("BZR  OFF", 5, 56);
}

// 導通チェック設定画面
void Display_Continuity_Check_Setting() {
    const uint8_t Cursor_XY[13][2] = {
        {24,  8},{78,  8}, // (1) ohm Hz
        {24, 16},{78, 16}, // (2) ohm Hz
        {24, 24},{78, 24}, // (3) ohm Hz
        {24, 32},{78, 32}, // (4) ohm Hz
        {24, 40},{78, 40}, // (5) ohm Hz
        {24, 48},{78, 48}, // (6) ohm Hz
        {46, 56} // SAVE
    };

    char str[30]; // 一時使用文字列

    OLED_print_XY("==CONTINUITY CHECK==", 3, 0);
    
    for(int i = 0; i <= 5; i++) {
        sprintf(str,"(%u)  %3u ohm  %4u Hz", i+1, Continuity_Check_Threshold[i], Continuity_Check_Freq[i]);
        OLED_print_XY(str, 0, 8 * (i + 1));
    }
    OLED_print_XY("SAVE", 52, 56);

    if (Cursor_Position > 12) Cursor_Position = 0;
    OLED_DrawCursor(Cursor_XY[Cursor_Position][0], Cursor_XY[Cursor_Position][1]);
}

// ADCキャリブレーション画面
void Display_ADC_Calibration() {
    const uint8_t Cursor_XY[7][2] = {
        {12, 16}, // RUN
        {12, 32},{36, 32},{48, 32},{60, 32}, // X.XXX V
        {12, 40}, // RUN
        {46, 56} // SAVE
    };

    char str[30]; // 一時使用文字列
    OLED_print_XY("==ADC CALIBRATION==", 6, 0);
    OLED_print_XY("< 0V >", 6, 8);
    OLED_print_XY("RUN", 18, 16);

    // オフセット電圧表示
    uint32_t offset_mV_x10 = 10000.0f * REF_VOLTAGE * ADC_Calibration_offset / (float)ADC_MAX_VALUE + 0.5f;
    sprintf(str,"(OFFSET %2u.%u mV)", offset_mV_x10/10, offset_mV_x10%10);
    OLED_print_XY(str,18, 24);
    sprintf(str,"< %2u. %u %u %u V >", ADC_Calibration_volt[0], ADC_Calibration_volt[1], ADC_Calibration_volt[2], ADC_Calibration_volt[3]);
    OLED_print_XY(str, 6, 32);
    OLED_print_XY("RUN",18, 40);

    // 電圧補正 傾き表示
    uint32_t slope_x1000 = 1000.0f * ADC_Calibration_slope + 0.5f;
    sprintf(str,"(SLOPE  %u.%03u )", slope_x1000/1000, slope_x1000%1000);
    OLED_print_XY(str, 18, 48);
    OLED_print_XY("SAVE", 52, 56);

    if (Cursor_Position > 6) Cursor_Position = 0;
    OLED_DrawCursor(Cursor_XY[Cursor_Position][0], Cursor_XY[Cursor_Position][1]);
}

// OHMキャリブレーション画面
void Display_Ohm_Calibration() {
    char Label[6][5] = {"  10", " 200", "  2k", " 20k", "200k", "2.2M"};

    const uint8_t Cursor_XY[7][2] = {
        { 6,  8}, // 10
        { 6, 16}, // 200
        { 6, 24}, // 2k
        { 6, 32}, // 20k
        { 6, 40}, // 200k
        { 6, 48}, // 2.2M
        {46, 56}  // SAVE
    };

    char str[30]; // 一時使用文字列
    OLED_print_XY("==OHM CALIBRATION==", 6, 0);
    for(int i = 0; i < 6; i++) {
        OLED_print_XY(Label[i], 12, 8 * (i + 1));
        if (i == 0) OLED_print_XY("SLOPE OFFSET", 12+30, 8 * (i + 1));
        else {
            const float factor[5] = {1.0f, 10.0f, 100.0f, 1000.0f, 10000.0f};
            sprintf(str,"%d", (int32_t)(factor[i-1] * ohm_Calibration_slope[i-1])); // 傾きは4桁整数で表示
            OLED_print_XY(str, 12+36, 8 * (i + 1));
            sprintf(str,"%d", (int32_t)ohm_Calibration_offset[i-1]);
            OLED_print_XY(str, 12+72, 8 * (i + 1));
        }
    }
    OLED_print_XY("SAVE", 52, 56);

    Cursor_Position = Ohm_Calibration_sequence;
    if (Cursor_Position > 6) Cursor_Position = 0;
    OLED_DrawCursor(Cursor_XY[Cursor_Position][0], Cursor_XY[Cursor_Position][1]);
}
