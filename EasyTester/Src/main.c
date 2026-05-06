/*
Easy Tester
v1.0 2026-05-04
https://github.com/kanengomibako

MCU: CH32V006F8U6
MounRiver Studio Version: 2.4.0

Optimization -O2
*/

#include "debug.h"
#include "common.h"
#include "main.h"
#include "hardware.h"
#include "measure.h"
#include "display.h"
#include "storage.h"

volatile uint8_t Cursor_Position = 0;
enum sw {SW_C, SW_B, SW_A, SW_MAX};
volatile uint8_t Long_Press_Flag[3] = {}; // スイッチ長押しフラグ 長押し時パラメータを10ずつ変化
volatile uint8_t Run_ADC_Calibration_0V_Flag = 0;
volatile uint8_t Run_ADC_Calibration_XV_Flag = 0;
volatile uint8_t Run_Ohm_Calibration_Flag = 0;
volatile uint8_t Save_Flag = 0;

volatile int8_t mode = 0;
volatile int8_t volt_enable = 1;
volatile int8_t ohm_enable = 1;
volatile int8_t buzzer_enable = 1;

extern int16_t Continuity_Check_Threshold[];
extern int16_t Continuity_Check_Freq[];
extern int8_t ADC_Calibration_volt[];

int main(void)
{
    SystemCoreClockUpdate();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    Delay_Init();

    GPIOs_Init();
    GPIO_WriteBit(LED_PORT, LED_PIN, Bit_SET); // 起動確認LED点灯

    TIM1_PWM_Init(); // 先に初期化しないとPWM出力されない
    I2C1_Init();
    Delay_Ms(100); // 入れないとたまにOLED表示失敗
    OLED_init();
    OPAMP_Init();
    ADC1_Init();
    TIM2_Init();

    Load_Data();

    while(1)
    {
        OLED_clear();

        if (mode != MAIN) { // 設定モード
            if (mode == CONTINUITY_CHECK_SETTING) {
                Display_Continuity_Check_Setting();
            }
            else if (mode == ADC_CALIBRATION) {
                Display_ADC_Calibration();
                if (Run_ADC_Calibration_0V_Flag) {
                    Run_ADC_Calibration_0V();
                    Run_ADC_Calibration_0V_Flag = 0;
                }
                if (Run_ADC_Calibration_XV_Flag) {
                    Run_ADC_Calibration_XV();
                    Run_ADC_Calibration_XV_Flag = 0;
                }
            }
            else if (mode == OHM_CALIBRATION) {
                Display_Ohm_Calibration();
                if (Run_Ohm_Calibration_Flag) {
                    Run_Ohm_Calibration();
                    Run_Ohm_Calibration_Flag = 0;
                }
            }
        }
        else { // メイン画面
            uint32_t volt = 0;
            uint32_t ohm = UINT32_MAX;

            if (volt_enable) volt = Get_Volt();
            if (ohm_enable && volt < OHM_CHANGE_VOLTAGE) ohm = Get_Ohm();
            R0_Reset(); // 次の電圧測定に備えておく

            if (buzzer_enable && volt < OHM_CHANGE_VOLTAGE) Continuity_Check(ohm);
            else Continuity_Check(UINT32_MAX); // ブザーオフ

            if ((!volt_enable || volt < OHM_CHANGE_VOLTAGE) && ohm_enable) Display_Ohm(ohm);
            else if (volt_enable) Display_Volt(volt);

            Display_main();
        }

        if (Save_Flag) {
            Save_Data();
            Save_Flag = 0;
        }

        OLED_DisplayBuffer(); // 画面表示 兼 電圧安定までの待機時間 約25ms
    }
}

// TIM2割り込みハンドラ スイッチ処理
void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        sw_process();
    }
}

// スイッチ処理 1msごとに割り込み
void sw_process(void) {
    static uint16_t swPushDuration[3] = {};
    static uint16_t continuousPushCnt[3] = {};

    GPIO_TypeDef* ports[3] = {SW_C_PORT, SW_B_PORT, SW_A_PORT};
    uint16_t pins[3] = {SW_C_PIN, SW_B_PIN, SW_A_PIN};

    // 負荷軽減のためスイッチを一個ずつ読み取り
    static uint8_t n = 0;
    n++;
    if (n > 2) n = 0;

    if (!GPIO_ReadInputDataBit(ports[n], pins[n])) {
        swPushDuration[n] += 3; // 次の読み取りまで3ms進む
        if (swPushDuration[n] > CONTINUOUS_PRESS_MS + continuousPushCnt[n] * CONTINUOUS_PRESS_MS) { // 押し続けたとき
            switch (n) {
                case SW_C:
                    Sw_C_Continuous_Push();
                    break;
                case SW_B:
                    Sw_B_Continuous_Push();
                    break;
                case SW_A:
                    Sw_A_Continuous_Push();
                    break;
            }
            continuousPushCnt[n]++;
        }
        if (swPushDuration[n] > LONG_PRESS_MS && Long_Press_Flag[n] == 0) { // 長押し 1回のみ
            switch (n) {
                case SW_C:
                    Sw_C_Long_Push();
                    break;
                case SW_B:
                    Sw_B_Long_Push();
                    break;
                case SW_A:
                    Sw_A_Long_Push();
                    break;
            }
            Long_Press_Flag[n] = 9; // 長押し時パラメータを1+9ずつ変化
        }
    }
    else {
        if (swPushDuration[n] > SHORT_PRESS_MS && swPushDuration[n] < LONG_PRESS_MS) { // 短押し 離した時の処理
            switch (n) {
                case SW_C:
                    Sw_C_Short_Push();
                    break;
                case SW_B:
                    Sw_B_Short_Push();
                    break;
                case SW_A:
                    Sw_A_Short_Push();
                    break;
            }
        }
        swPushDuration[n] = 0;
        continuousPushCnt[n] = 0;
        Long_Press_Flag[n] = 0;
    }
}

// スイッチA短押し
void Sw_A_Short_Push(void) {
    if (mode == MAIN) volt_enable = !volt_enable; // 電圧測定 有効/無効切替
    else if (mode == CONTINUITY_CHECK_SETTING) {
        switch (Cursor_Position) {
            case 0:
            case 2:
            case 4:
            case 6:
            case 8:
            case 10:
                Continuity_Check_Threshold[Cursor_Position/2] += 1 + Long_Press_Flag[SW_A];
                if (Continuity_Check_Threshold[Cursor_Position/2] > 999) Continuity_Check_Threshold[Cursor_Position/2] = 999;
                break;
            case 1:
            case 3:
            case 5:
            case 7:
            case 9:
            case 11:
                Continuity_Check_Freq[Cursor_Position/2] += 10 * (1 + Long_Press_Flag[SW_A]);
                if (Continuity_Check_Freq[Cursor_Position/2] > 9990) Continuity_Check_Freq[Cursor_Position/2] = 9990;
                break;
            case 12:
                Save_Flag = 1;
                break;
            default: 
                break;
        }
    }
    else if (mode == ADC_CALIBRATION) {
        switch (Cursor_Position) {
            case 0:
                Run_ADC_Calibration_0V_Flag = 1;
                break;
            case 1:
                ADC_Calibration_volt[Cursor_Position-1] += 1;
                if (ADC_Calibration_volt[Cursor_Position-1] > 20) ADC_Calibration_volt[Cursor_Position-1] = 20;
                break;
            case 2:
            case 3:
            case 4:
                ADC_Calibration_volt[Cursor_Position-1] += 1;
                if (ADC_Calibration_volt[Cursor_Position-1] > 9) ADC_Calibration_volt[Cursor_Position-1] = 9;
                break;
            case 5:
                Run_ADC_Calibration_XV_Flag = 1;
                break;
            case 6:
                Save_Flag = 1;
                break;
            default: 
                break;
        }
    }
    else if (mode == OHM_CALIBRATION) {
        switch (Cursor_Position) {
            case 0:
            case 1:
            case 2:
            case 3:
            case 4:
            case 5:
                Run_Ohm_Calibration_Flag = 1;
                break;
            case 6:
                Save_Flag = 1;
                break;
            default: 
                break;
        }
    }
}

// スイッチA押し続け時 -> スイッチA短押し
void Sw_A_Continuous_Push(void) {
    if (mode != MAIN) Sw_A_Short_Push();
}

// スイッチA長押し
void Sw_A_Long_Push(void) {
    if (mode == MAIN) Save_Flag = 1; // データセーブ
}

// スイッチB短押し
void Sw_B_Short_Push(void) {
    if (mode == MAIN) ohm_enable = !ohm_enable; // 抵抗測定 有効/無効切替
    else if (mode == CONTINUITY_CHECK_SETTING) {
        switch (Cursor_Position) {
            case 0:
            case 2:
            case 4:
            case 6:
            case 8:
            case 10:
                Continuity_Check_Threshold[Cursor_Position/2] -= 1 + Long_Press_Flag[SW_B];
                if (Continuity_Check_Threshold[Cursor_Position/2] < 1) Continuity_Check_Threshold[Cursor_Position/2] = 1;
                break;
            case 1:
            case 3:
            case 5:
            case 7:
            case 9:
            case 11:
                Continuity_Check_Freq[Cursor_Position/2] -= 10 * (1 + Long_Press_Flag[SW_B]);
                if (Continuity_Check_Freq[Cursor_Position/2] < 100) Continuity_Check_Freq[Cursor_Position/2] = 100;
                break;
            default:
                break;
        }
    }
    else if (mode == ADC_CALIBRATION) {
        switch (Cursor_Position) {
            case 1:
            case 2:
            case 3:
            case 4:
                ADC_Calibration_volt[Cursor_Position-1] -= 1;
                if (ADC_Calibration_volt[Cursor_Position-1] < 0) ADC_Calibration_volt[Cursor_Position-1] = 0;
                break;
            default: 
                break;
        }
    }

}

// スイッチB押し続け時 -> スイッチB短押し
void Sw_B_Continuous_Push(void) {
    if (mode != MAIN) Sw_B_Short_Push();
}

// スイッチB長押し 処理なし
void Sw_B_Long_Push(void) {}

// スイッチC短押し
void Sw_C_Short_Push(void) {
    if (mode == MAIN) buzzer_enable = !buzzer_enable; // ブザー 有効/無効切替
    if (mode != OHM_CALIBRATION) Cursor_Position++; // カーソル位置変更
}

// スイッチC押し続け時 処理なし
void Sw_C_Continuous_Push(void) {}

// スイッチC長押し ページ変更
void Sw_C_Long_Push(void) {
    mode = (mode + 1) % MODE_MAX;
    Cursor_Position = 0;
}
