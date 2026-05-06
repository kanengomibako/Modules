#include "common.h"
#include "ch32v00X.h"
#include "measure.h"
#include "display.h"

// デバッグ用ADC値ログ
//uint32_t adc_log[256] = {};
//uint8_t adc_log_index = 0;

const float Ref_Resistance[5] = {22.0f, 222.0f, 2222.0f, 22022.0f, 220022.0f}; // 基準抵抗 + GPIO出力の抵抗
int8_t Range_Num = R_0R; // 抵抗値レンジ番号
const uint16_t Range_Wait[5] = {1, 1, 1, 1, 10}; // 基準抵抗をセットしてからの待ち時間 ms

volatile int16_t Continuity_Check_Threshold[6] = {}; // 導通チェック 閾値抵抗値
volatile int16_t Continuity_Check_Freq[6] = {}; // 導通チェック 周波数

volatile int8_t ADC_Calibration_volt[4] = {5, 0, 0, 0};
float ADC_Calibration_offset = 1.0f; // ADC線形補間 切片 サンプリング1回あたり 0.81倍でmV
float ADC_Calibration_slope = 1.0f;  // ADC線形補間 傾き

int8_t Ohm_Calibration_sequence = 0; // OHM線形補間 何番目か
float ohm_Calibration_offset[5] = {}; // OHM線形補間 切片
float ohm_Calibration_slope[5] = {}; // OHM線形補間 傾き
const float Calibration_Ohm[6] = {10.0f, 200.0f, 2000.0f, 20000.0f, 200000.0f, 2200000.0f,}; // 校正用抵抗 10～2.2MΩ

// ADC値を一回のみ取得
uint32_t ADC_GetValue_Once(void) {
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)); // 変換完了待機
    return ADC_GetConversionValue(ADC1);
}

// ADC値のサンプリング回数分の和を取得
uint32_t ADC_GetValue_Sum(uint32_t sample_num) {
    uint32_t adc = 0;

    for (uint32_t i = 0; i < sample_num; i++) {
        adc += ADC_GetValue_Once();
    }

// デバッグ用ADC値ログ
//adc_log[adc_log_index] = adc;
//adc_log_index++;

    return adc;
}

// ADC値をキャリブレーションしたオフセットと傾きで線形補間
float Correct_ADC(uint32_t x, uint32_t sample_num) {
    float corrected_x = ((float)x - ADC_Calibration_offset * (float)sample_num) / ADC_Calibration_slope;
    if (corrected_x > 0) return corrected_x;
    else return 0.0f;
}

// 基準抵抗を切替 全てGPIO入力へ
void Reset_Reference_Resistor(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = R0_PIN | R1_PIN | R2_PIN | R3_PIN | R4_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(R0_PORT, &GPIO_InitStructure);
}

// R0ピンを出力Lにし、一度上がった電圧を下がりやすくする
void R0_Reset(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = R0_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(R0_PORT, &GPIO_InitStructure);

    GPIO_WriteBit(R0_PORT, R0_PIN, Bit_RESET);
}

// 基準抵抗を切替 目的のピンをGPIO出力Hに
void Set_Reference_Resistor(uint8_t r) {
    GPIO_TypeDef* ports[] = {R0_PORT, R1_PORT, R2_PORT, R3_PORT, R4_PORT};
    uint16_t pins[] = {R0_PIN, R1_PIN, R2_PIN, R3_PIN, R4_PIN};

    Range_Num = r;
    Reset_Reference_Resistor();

    if (r >= R_MAX) return;

    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = pins[r];
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(ports[r], &GPIO_InitStructure);

    GPIO_WriteBit(ports[r], pins[r], Bit_SET);
}

// 電圧値取得
uint32_t Get_Volt(void) {
    float volt_f; uint32_t adc;
    adc = ADC_GetValue_Sum(ADC_SAMPLE_NUM_VOLT);
    volt_f = REF_VOLTAGE * VOLTAGE_ATTENUATE_FACTOR * Correct_ADC(adc, ADC_SAMPLE_NUM_VOLT) / (ADC_MAX_VALUE * (float)ADC_SAMPLE_NUM_VOLT);
    return (uint32_t)(1000.0f * volt_f + 5.0f); // 1000倍、四捨五入し整数値へ
}

// 抵抗値取得
uint32_t Get_Ohm(void) {
    float ohm_f; uint32_t adc; // 一時計算用

    // 抵抗レンジ判定
    for (int range = 0; range < R_MAX; range++) {
        Set_Reference_Resistor(range);
        Range_Num = range;
        Delay_Ms(Range_Wait[range]);
        adc = ADC_GetValue_Once();
        if (adc < RANGE_CHANGE_THRESHOLD_H) break;
    }

    // 抵抗値計算 キャリブレーション値で線形補間
    adc = ADC_GetValue_Sum(ADC_SAMPLE_NUM_OHM);
    ohm_f = ((float)adc - ohm_Calibration_offset[Range_Num]) / ohm_Calibration_slope[Range_Num];

    // 並列抵抗の補正
    if (ohm_f < PARALLEL_RESISTANCE) ohm_f = ohm_f * PARALLEL_RESISTANCE / (PARALLEL_RESISTANCE - ohm_f);
    
    return (uint32_t)(ohm_f + 0.5f); // 四捨五入し整数値へ
}

// 導通チェック 設定した抵抗値より小さいとき音を鳴らす
void Continuity_Check(uint32_t ohm) {
    const uint8_t Continuity_Check_Index_Max = 6;
    uint8_t Continuity_Check_Index = Continuity_Check_Index_Max;

    // どの閾値に当てはまるか決定 抵抗値が小さい方を優先する
    for (int i = 0; i < Continuity_Check_Index_Max; i++) {
        if (ohm < Continuity_Check_Threshold[i]) {
            if (Continuity_Check_Index >= Continuity_Check_Index_Max) { // 最初に当てはまったとき
                Continuity_Check_Index = i;
            }
            else if (Continuity_Check_Threshold[i] < Continuity_Check_Threshold[Continuity_Check_Index]) {
                Continuity_Check_Index = i;
            }
        }
    }

    // PWMの周波数とデューティ比を設定
    if (Continuity_Check_Index < Continuity_Check_Index_Max) {
        uint32_t arr = 1000000 / (uint32_t)Continuity_Check_Freq[Continuity_Check_Index] - 1;
        TIM_SetAutoreload(TIM1, arr);
        TIM_SetCompare1(TIM1, arr / 2);
    }
    else TIM_SetCompare1(TIM1, 0);
}


// 0VでADCキャリブレーション実行 オフセット算出
void Run_ADC_Calibration_0V(void) {
    uint32_t adc; // 一時計算用

    R0_Reset();
    Delay_Ms(50); // 電圧安定まで待機
    OLED_print_XY("    PROCESSING...    ", 1, 56);
    OLED_DisplayBuffer();

    // ADC 一時的にオフセットを計算
    adc = ADC_GetValue_Sum(CALIBRATION_SAMPLE_NUM);
    float tmp_offset = (float)adc / (float)CALIBRATION_SAMPLE_NUM;
    
    // オフセットが異常値の場合エラー
    if (tmp_offset > ADC_CALIBRATION_OFFSET_MAX) {
        OLED_print_XY("       ERROR!        ", 1, 56);
    }
    else {
        ADC_Calibration_offset = tmp_offset;
        OLED_print_XY("        DONE!        ", 1, 56);
    }

    OLED_DisplayBuffer();
    Delay_Ms(STATUS_DISP_MS);
}

// 設定電圧でADCキャリブレーション実行 傾き算出
void Run_ADC_Calibration_XV(void) {
    uint32_t adc; // 一時計算用

    // キャリブレーションに使用する設定電圧を計算
    float Calibration_volt = (float)ADC_Calibration_volt[0] + 0.1f * (float)ADC_Calibration_volt[1]
        + 0.01f * (float)ADC_Calibration_volt[2] + 0.001f * (float)ADC_Calibration_volt[3];

    R0_Reset();
    Delay_Ms(50); // 電圧安定まで待機
    OLED_print_XY("    PROCESSING...    ", 1, 56);
    OLED_DisplayBuffer();

    // ADC 一時的に傾きを計算
    adc = ADC_GetValue_Sum(CALIBRATION_SAMPLE_NUM);
    float tmp_volt = (float)adc / (float)CALIBRATION_SAMPLE_NUM - ADC_Calibration_offset;
    tmp_volt = REF_VOLTAGE * VOLTAGE_ATTENUATE_FACTOR * tmp_volt / (float)ADC_MAX_VALUE;
    float tmp_slope = tmp_volt / Calibration_volt;
    
    // 5%ずれている場合エラー
    if (tmp_slope < 0.95f || tmp_slope > 1.05f) {
        OLED_print_XY("       ERROR!        ", 1, 56);
    }
    else {
        ADC_Calibration_slope = tmp_slope;
        OLED_print_XY("        DONE!        ", 1, 56);
    }

    OLED_DisplayBuffer();
    Delay_Ms(STATUS_DISP_MS);
}

// 抵抗測定 キャリブレーション実行
void Run_Ohm_Calibration(void) {
    uint32_t adc; // 一時計算用
    int8_t error_flag = 0;
    float Theoretical_value = 1.0f; // ADC 理論値

    static float x1[6] = {}; // OHM線形補間 x1
    static float x2[6] = {}; // OHM線形補間 x2
    static float y1[6] = {}; // OHM線形補間 実測データy1
    static float y2[6] = {}; // OHM線形補間 実測データy2

    OLED_print_XY("    PROCESSING...    ", 1, 56);
    OLED_DisplayBuffer();
    
    // 線形補間用x2, y2, 傾き, 切片を算出
    if (Ohm_Calibration_sequence >= 1 && Ohm_Calibration_sequence <= 5) {
        Range_Num = Ohm_Calibration_sequence - 1; // 1つ下のレンジとする
        Set_Reference_Resistor(Range_Num);
        Delay_Ms(Range_Wait[Range_Num]);
        adc = ADC_GetValue_Sum(CALIBRATION_SAMPLE_NUM);

        // 並列抵抗を計算
        x2[Range_Num] = Calibration_Ohm[Ohm_Calibration_sequence] * PARALLEL_RESISTANCE / (Calibration_Ohm[Ohm_Calibration_sequence] + PARALLEL_RESISTANCE);
        // ADC回数を補正
        y2[Range_Num] = (float)adc * (float)ADC_SAMPLE_NUM_OHM / (float)CALIBRATION_SAMPLE_NUM;
        
        // ADC値が理論値の90%～110%の間でない場合はエラー
        Theoretical_value = (float)ADC_SAMPLE_NUM_OHM * ADC_MAX_VALUE 
            * x2[Range_Num] * OPA_REF_VOLTAGE_FACTOR / Ref_Resistance[Range_Num] / VOLTAGE_ATTENUATE_FACTOR;
        if (y2[Range_Num] < 0.9f * Theoretical_value ||  1.1f * Theoretical_value < y2[Range_Num]) error_flag = 1;
        
        // 線形補間係数を計算
        if (!error_flag) {
            ohm_Calibration_slope[Range_Num] = (y2[Range_Num] - y1[Range_Num]) / (x2[Range_Num] - x1[Range_Num]);
            ohm_Calibration_offset[Range_Num] = y1[Range_Num] - x1[Range_Num] * ohm_Calibration_slope[Range_Num];
        }
    }

    // 線形補間用x1, y1を算出 
    if (Ohm_Calibration_sequence >= 0 && Ohm_Calibration_sequence <= 4){
        Range_Num = Ohm_Calibration_sequence;
        Set_Reference_Resistor(Range_Num);
        Delay_Ms(Range_Wait[Range_Num]);
        adc = ADC_GetValue_Sum(CALIBRATION_SAMPLE_NUM);

        // 並列抵抗を計算
        x1[Range_Num] = Calibration_Ohm[Range_Num] * PARALLEL_RESISTANCE / (Calibration_Ohm[Range_Num] + PARALLEL_RESISTANCE);
        // ADC回数を補正
        y1[Range_Num] = (float)adc * (float)ADC_SAMPLE_NUM_OHM / (float)CALIBRATION_SAMPLE_NUM;
        
        // 取得値が理論値の80%～150%の間でない場合はエラー
        Theoretical_value = (float)ADC_SAMPLE_NUM_OHM * ADC_MAX_VALUE 
            * x1[Range_Num] * OPA_REF_VOLTAGE_FACTOR / Ref_Resistance[Range_Num] / VOLTAGE_ATTENUATE_FACTOR;
        if (y1[Range_Num] < 0.8f * Theoretical_value ||  1.5f * Theoretical_value < y1[Range_Num]) error_flag = 1;
    }

    if (error_flag) {
        OLED_print_XY("       ERROR!        ", 1, 56);
    }
    else {
        OLED_print_XY("        DONE!        ", 1, 56);
        if (Ohm_Calibration_sequence < 6) Ohm_Calibration_sequence++;
    }

    OLED_DisplayBuffer();
    Delay_Ms(STATUS_DISP_MS);
}