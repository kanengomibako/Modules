#include "common.h"
#include "storage.h"
#include "ch32v00X.h"
#include "display.h"
#include "string.h" // memcpy用

uint32_t storage_data[64] = {};
extern int8_t mode;
extern int8_t volt_enable;
extern int8_t ohm_enable;
extern int8_t buzzer_enable;
extern int16_t Continuity_Check_Threshold[];
extern int16_t Continuity_Check_Freq[];
extern int8_t ADC_Calibration_volt[];
extern float ADC_Calibration_offset;
extern float ADC_Calibration_slope;
extern float ohm_Calibration_offset[]; 
extern float ohm_Calibration_slope[];

// floatからuint32_tへデータそのままでコピー
uint32_t float_to_u32(float f) {
    uint32_t u;
    memcpy(&u, &f, sizeof(u));
    return u;
}

// uint32_tからfloatへデータそのままでコピー
float u32_to_float(uint32_t u) {
    float f;
    memcpy(&f, &u, sizeof(f));
    return f;
}

// パラメータが範囲外のときデフォルト値へ変更 int
int32_t param_or_default_int(int32_t value, int32_t min, int32_t max, int32_t def)
{
    if (value >= min && value <= max) return value;
    return def;
}

// パラメータが範囲外のときデフォルト値へ変更 float
float param_or_default_f(float value, float min, float max, float def)
{
    if (value >= min && value <= max) return value;
    return def;
}

// フラッシュ書き込み
void Save_Data() {
    uint32_t addr = FLASH_ADDR;

    storage_data[0] = volt_enable;
    storage_data[1] = ohm_enable;
    storage_data[2] = buzzer_enable;
    for (int i = 0; i < 6; i++) storage_data[3+i] = Continuity_Check_Threshold[i];
    for (int i = 0; i < 6; i++) storage_data[9+i] = Continuity_Check_Freq[i];
    for (int i = 0; i < 4; i++) storage_data[15+i] = ADC_Calibration_volt[i];
    storage_data[19] = float_to_u32(ADC_Calibration_offset);
    storage_data[20] = float_to_u32(ADC_Calibration_slope);
    for (int i = 0; i < 5; i++) storage_data[21+i] = float_to_u32(ohm_Calibration_offset[i]);
    for (int i = 0; i < 5; i++) storage_data[26+i] = float_to_u32(ohm_Calibration_slope[i]);

    FLASH_Unlock_Fast();
    FLASH_ErasePage_Fast(addr);

    FLASH_ROM_WRITE(addr, storage_data, 256);
    
    FLASH_Lock_Fast();

    if (mode == MAIN) OLED_print_XY("SAVED!  ", 5, 56);
    else OLED_print_XY("       SAVED!        ", 1, 56);
    OLED_DisplayBuffer();
    Delay_Ms(STATUS_DISP_MS);
}

// フラッシュ読み込み
void Load_Data() {
    uint32_t addr = FLASH_ADDR;

    // パラメータを読み込みそれぞれ変数に入れ直す 異常な値の場合はデフォルト値とする
    for (int i = 0; i < 64; i++) {
        storage_data[i] = *(uint32_t*)(addr + i * 4);
    }

    volt_enable = param_or_default_int(storage_data[0], 0, 1, 1);
    ohm_enable = param_or_default_int(storage_data[1], 0, 1, 1);
    buzzer_enable = param_or_default_int(storage_data[2], 0, 1, 1);
    for (int i = 0; i < 6; i++) Continuity_Check_Threshold[i] = param_or_default_int(storage_data[3+i], 1, 999, 100);
    for (int i = 0; i < 6; i++) Continuity_Check_Freq[i] = param_or_default_int(storage_data[9+i], 100, 9990, 1000);
    ADC_Calibration_volt[0] = param_or_default_int(storage_data[15], 1, 20, 5);
    ADC_Calibration_volt[1] = param_or_default_int(storage_data[16], 0, 9, 0);
    ADC_Calibration_volt[2] = param_or_default_int(storage_data[17], 0, 9, 0);
    ADC_Calibration_volt[3] = param_or_default_int(storage_data[18], 0, 9, 0);
    ADC_Calibration_offset = param_or_default_f(u32_to_float(storage_data[19]), 0.0f, ADC_CALIBRATION_OFFSET_MAX, 3.0f);
    ADC_Calibration_slope  = param_or_default_f(u32_to_float(storage_data[20]), 0.9f, 1.1f, 0.996f);
    ohm_Calibration_offset[0] = param_or_default_f(u32_to_float(storage_data[21]), -3000.0f, 9000.0f, 1000.0f);
    ohm_Calibration_offset[1] = param_or_default_f(u32_to_float(storage_data[22]), -3000.0f, 9000.0f,  900.0f);
    ohm_Calibration_offset[2] = param_or_default_f(u32_to_float(storage_data[23]), -3000.0f, 9000.0f,  900.0f);
    ohm_Calibration_offset[3] = param_or_default_f(u32_to_float(storage_data[24]), -3000.0f, 9000.0f,  900.0f);
    ohm_Calibration_offset[4] = param_or_default_f(u32_to_float(storage_data[25]), -3000.0f, 9000.0f,  900.0f);
    ohm_Calibration_slope[0] = param_or_default_f(u32_to_float(storage_data[26]), 900.0f, 1500.0f, 1050.0f);
    ohm_Calibration_slope[1] = param_or_default_f(u32_to_float(storage_data[27]), 90.0f, 150.0f, 105.0f);
    ohm_Calibration_slope[2] = param_or_default_f(u32_to_float(storage_data[28]), 9.0f, 15.0f, 10.5f);
    ohm_Calibration_slope[3] = param_or_default_f(u32_to_float(storage_data[29]), 0.9f, 1.5f, 1.05f);
    ohm_Calibration_slope[4] = param_or_default_f(u32_to_float(storage_data[30]), 0.09f, 0.15f, 0.105f);
}
