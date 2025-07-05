/*
DropInTap PT2399
v1.0 2025-06-17
https://github.com/kanengomibako

MCU: CH32V003F4U6
MounRiver Studio Version: 2.1.0

system_ch32v00x.c を編集し、内部クロック利用に設定
アンコメント: #define SYSCLK_FREQ_48MHZ_HSI   48000000
コメントアウト: #define SYSCLK_FREQ_48MHz_HSE   48000000

Optimization -O2
*/

#include "debug.h"

// ピン定義 ==============================
#define TAP_SW_PORT GPIOA // PA1 内部プルアップ タップスイッチ入力
#define TAP_SW_PIN  GPIO_Pin_1
#define LED_PORT GPIOA // PA2 LED出力
#define LED_PIN  GPIO_Pin_2
#define TEST_PULSE_PORT GPIOC // PC6 キャリブレーション時、ディレイタイム計測用試験波出力 ※通常動作時はGPIO入力
#define TEST_PULSE_PIN  GPIO_Pin_6
#define PWM_PORT GPIOD // PD2 PWM出力
#define PWM_PIN  GPIO_Pin_2
#define TEST_PULSE_DETECT_PORT GPIOD // PD3 ディレイタイム計測用試験波検出(ADC) ※通常動作時はGPIO入力
#define TEST_PULSE_DETECT_PIN  GPIO_Pin_3
#define PT2399_DETECT_PORT GPIOD // PD4 内部プルダウン PT2399起動確認入力
#define PT2399_DETECT_PIN  GPIO_Pin_4
#define TAP_DIV_SW_PORT GPIOD // PD5 TAP DIVスイッチ(ADC)
#define TAP_DIV_SW_PIN  GPIO_Pin_5
#define DELAY_POT_PORT GPIOD // PD6 ディレイタイム設定ポット(ADC)
#define DELAY_POT_PIN  GPIO_Pin_6

// グローバル変数 定義 ==============================
uint16_t PotBlinkCtrl = 1; // タップテンポ未設定時、ポットのディレイタイム設定に合わせてLEDが点滅する機能
uint8_t ChangeSettingFlag = 0; // 設定変更実行中フラグ
volatile uint16_t PotDelay = 0; // ポットで設定したディレイタイム
volatile uint16_t TapDelay = 0; // タップで設定したディレイタイム
const uint16_t CalibrationDuty[14] = {23, 25, 27, 30, 34, 38, 43, 51, 61, 75, 98, 140, 245, 700}; // キャリブレーション用デューティ比(1000倍) ディレイタイムが約700ms～50msとなる値
const uint32_t CalibrationDutyReciprocal[14] = {434783, 400000, 370370, 333333, 294118, 263158, 232558, 196078, 163934, 133333, 102041, 71429, 40816, 14286}; // 上記デューティ比の逆数
uint16_t DefaultDelays[14] = {}; // 出荷時に設定する実測ディレイタイムデフォルト値(10倍)
const uint16_t InitialDefaultDelays[14] = {7256, 6732, 6282, 5711, 5109, 4628, 4151, 3574, 3060, 2563, 2048, 1532, 1000, 534}; // ディレイタイムデフォルト値設定前の初期値
uint16_t CalibratedDelays[14] = {}; // キャリブレーションで実測したディレイタイム(10倍)
#define FLASH_ADDR 0x08003FC0 // データ保存先フラッシュアドレス 末尾64バイト
#define CALIBRATION_ERROR_THRESHOLD 10 // キャリブレーションで実測したディレイタイムのデフォルト値からのズレをエラー判定する閾値(%)

// 時間設定
volatile uint32_t TimerTick = 10000; // 時間計測用 TIM1の割り込み(0.25msごと)でカウントアップ
#define TICKS_PER_1MS 4 // 1msあたりのTimerTickカウント数
#define SHORT_PRESS_MS 25 // 短押し判定時間
#define LONG_PRESS_MS 2000 // 長押し判定時間
#define LONG_LONG_PRESS_MS 5000 // 長い長押し判定時間
#define LED_ON_MS 10 // タップテンポLEDを点灯させておく時間
#define MAX_DELAY_MS 700 // 設定可能最大ディレイタイム
#define MIN_DELAY_MS 50  // 設定可能最小ディレイタイム
#define MAX_MEASURE_DELAY_MS 900 // 測定可能最大ディレイタイム
#define MAX_DUTY 750 // 最大デューティ比(1000倍) → ディレイタイム最小
#define MAX_TAP_INTERVAL_MS 1000 // タップ最大時間
#define MAX_TAP_INTERVALS 4 // 記録するタップ間隔の最大数 タップ5回：4つの間隔

// ADC関係
volatile uint16_t adc[3]; // ADC取得値 PD3 (ADC4), PD5 (ADC5), PD6 (ADC6)
enum adcNum {PT2399OUT, TAP_DIV, DELAY}; // 上記配列の添字
#define ADC_CHANNEL_PT2399OUT ADC_Channel_4 // ADC_Channel 定義
#define ADC_CHANNEL_TAP_DIV ADC_Channel_5
#define ADC_CHANNEL_DELAY ADC_Channel_6
#define ADC_MAX_VALUE 1023 // ADC 10ビット 最大値
#define TAP_DIV_THRESHOLD_HIGH (ADC_MAX_VALUE * 3 / 5) // TAPDIVスイッチ判定閾値H 3V
#define TAP_DIV_THRESHOLD_LOW (ADC_MAX_VALUE * 1 / 3)  // TAPDIVスイッチ判定閾値L 1.7V
#define DELAYED_SIGNAL_DETECT_THRESHOLD (ADC_MAX_VALUE * 3 / 5) // ディレイ信号検出閾値
#define POT_CHANGE_THRESHOLD 4 // ポットを動かしたと判定する閾値
#define ADC_ADJ_RESISTOR_KOHM 47 // ADC値補正用 ADCピンに接続している抵抗の値
#define ADC_ADJ_POT_KOHM 50 // ADC値補正用 ポットの抵抗値
#define ADC_POT_FAULT_VALUE 574 // ポットの抵抗値が異常(60kΩ)の時のADC値 1023*60/(60+47)

// プロトタイプ宣言 ==============================
void GPIO_Config(void);
void GPIO_Config_Normal_Mode(void);
void TIM1_PWM_Config(void);
void TIM2_Interrupt_Init(void);
void ADC_InitConfig(void);
void BlinkLED(uint8_t n);
uint16_t ADC_GetValue(uint8_t channel);
void TIM2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void ChangeSetting(void);
void Calibration(void);
void UpdatePWM(uint16_t Delay);
void SaveData(void);
void LoadData(void);

// main ==============================
int main(void) {
    SystemCoreClockUpdate();
    Delay_Init();

    GPIO_Config();
    TIM1_PWM_Config();
    TIM2_Interrupt_Init();
    ADC_InitConfig();
    LoadData();

    // PT2399のクロック立ち上がり待機 500ms程度
    while (!GPIO_ReadInputDataBit(PT2399_DETECT_PORT, PT2399_DETECT_PIN)) {}

    // タップスイッチを押しながら起動した時、設定変更
    if (!GPIO_ReadInputDataBit(TAP_SW_PORT, TAP_SW_PIN)) ChangeSetting();

    // 非キャリブレーション時はPC6、PD3をGPIO入力に設定
    GPIO_Config_Normal_Mode();

    while (1) {
        static int16_t last_adcDELAY = 0; // 前回のadc[DELAY]
    
        adc[DELAY] = ADC_GetValue(ADC_CHANNEL_DELAY); // ディレイタイムポット
        if (adc[DELAY] > ADC_POT_FAULT_VALUE) adc[DELAY] = ADC_POT_FAULT_VALUE; // ポット故障時を除外

        // ポットを動かした時、adc[DELAY]更新とタップテンポ解除
        if (last_adcDELAY - adc[DELAY] > POT_CHANGE_THRESHOLD || adc[DELAY] - last_adcDELAY > POT_CHANGE_THRESHOLD) {
            last_adcDELAY = adc[DELAY];
            TapDelay = 0;
        }
        else {
            adc[DELAY] = last_adcDELAY; // ポットが動いてないと判定 値を更新しない
        }

        // ディレイポットのADCの値を補正（47kΩ抵抗を入れているため）
        uint32_t corrected_adcDELAY = ADC_ADJ_RESISTOR_KOHM * ADC_MAX_VALUE * adc[DELAY] / (ADC_MAX_VALUE - adc[DELAY]) / ADC_ADJ_POT_KOHM;

        // ディレイタイム計算 ポットの抵抗値が大きいと最大ディレイタイムを超える場合あり
        PotDelay = MIN_DELAY_MS + (MAX_DELAY_MS - MIN_DELAY_MS) * corrected_adcDELAY / ADC_MAX_VALUE;
        uint16_t FinalDelay = PotDelay;

        // タップテンポ有効時 TAP_DIV反映
        if (TapDelay) {
            adc[TAP_DIV] = ADC_GetValue(ADC_CHANNEL_TAP_DIV); // TapDivスイッチ 最大1023
            if (adc[TAP_DIV] > TAP_DIV_THRESHOLD_HIGH) FinalDelay = TapDelay / 2;
            else if (adc[TAP_DIV] > TAP_DIV_THRESHOLD_LOW) FinalDelay = TapDelay * 3 / 4;
            else FinalDelay = TapDelay;
        }

        // ディレイタイムからデューティ比に変換、反映
        UpdatePWM(FinalDelay);

        // LED点滅
        uint16_t LED_BlinkTime = PotDelay; // LED点滅間隔時間
        if (TapDelay) LED_BlinkTime = TapDelay;
        if ((TapDelay || PotBlinkCtrl) && (TimerTick % (TICKS_PER_1MS * LED_BlinkTime) < LED_ON_MS * TICKS_PER_1MS)) {
            GPIO_WriteBit(LED_PORT, LED_PIN, Bit_SET);
        }
        else {
            GPIO_WriteBit(LED_PORT, LED_PIN, Bit_RESET);
        }

    }
}

// 設定変更
void ChangeSetting(void) {
    ChangeSettingFlag = 1;
    TimerTick = 0; // タイマーリセット

    // タップスイッチを押し続けている時
    while (!GPIO_ReadInputDataBit(TAP_SW_PORT, TAP_SW_PIN)) {
        // 長押しでLED点灯 → LED点滅設定変更
        if (TimerTick > LONG_PRESS_MS * TICKS_PER_1MS) GPIO_WriteBit(LED_PORT, LED_PIN, Bit_SET);
        // 長い長押しでLED点灯を暗くする → キャリブレーション
        if (TimerTick > LONG_LONG_PRESS_MS * TICKS_PER_1MS) GPIO_WriteBit(LED_PORT, LED_PIN, Bit_RESET);
    }

    // タップスイッチを離した時
    if (TimerTick > LONG_LONG_PRESS_MS * TICKS_PER_1MS) Calibration();
    else if (TimerTick > LONG_PRESS_MS * TICKS_PER_1MS) {
        PotBlinkCtrl = !PotBlinkCtrl; // LED点滅設定変更
        SaveData();
        BlinkLED(3);
        ChangeSettingFlag = 0;
    }
}

// キャリブレーション:設定したデューティ比の時のディレイタイムを測定 ==============================
void Calibration(void) {
    uint16_t DelayTicks[5] = {}; // 測定したタイマーカウント(1msあたり4カウント)

    // 14ポイントのディレイタイムを測定、記録
    for (int i = 0; i < 14; i++) {
        TIM_SetCompare1(TIM1, CalibrationDuty[i]); // キャリブレーションデューティ比を設定
        Delay_Ms(100); // 電圧安定まで待機

        // 5回測定し、中央3サンプルの平均を計算
        for (int j = 0; j < 5; j++) {
            GPIO_WriteBit(TEST_PULSE_PORT, TEST_PULSE_PIN, Bit_SET); // 試験信号出力
            TimerTick = 0;
            Delay_Ms(5);
            GPIO_WriteBit(TEST_PULSE_PORT, TEST_PULSE_PIN, Bit_RESET);
            GPIO_WriteBit(LED_PORT, LED_PIN, Bit_SET); // 測定中のLED点灯
            // ADC値取得(0.1msかかるが無視)、遅延信号検出
            while(TimerTick < MAX_MEASURE_DELAY_MS * TICKS_PER_1MS) {
                adc[PT2399OUT] = ADC_GetValue(ADC_CHANNEL_PT2399OUT);
                if (adc[PT2399OUT] < DELAYED_SIGNAL_DETECT_THRESHOLD) break;
            }
            DelayTicks[j] = TimerTick;
            GPIO_WriteBit(LED_PORT, LED_PIN, Bit_RESET); // 測定完了でLED消灯
            Delay_Ms(10);
        }
        // バブルソート
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4 - i; j++) {
                if (DelayTicks[j] > DelayTicks[j + 1]) {
                    uint16_t temp = DelayTicks[j];
                    DelayTicks[j] = DelayTicks[j + 1];
                    DelayTicks[j + 1] = temp;
                }
            }
        }
        // 測定ディレイタイム(10倍)を決定
        CalibratedDelays[i] = 10 * (DelayTicks[1] + DelayTicks[2] + DelayTicks[3]) / 3 / TICKS_PER_1MS;

        // スイッチが押し続けられていた場合、測定値をデフォルト値として登録(出荷時用)
        if (!GPIO_ReadInputDataBit(TAP_SW_PORT, TAP_SW_PIN)) {
            Delay_Ms(10);
            if (!GPIO_ReadInputDataBit(TAP_SW_PORT, TAP_SW_PIN)) DefaultDelays[i] = CalibratedDelays[i];
        }
    }

    // デフォルト値からの最大誤差(%)を計算
    uint16_t CalibrationError = 0;
    for (int i = 0; i < 14; i++) {
        int32_t ErrorPercent = 100 * (CalibratedDelays[i] - DefaultDelays[i]) / DefaultDelays[i];
        if (ErrorPercent < 0) ErrorPercent = -ErrorPercent; // 負の数は正の数へ
        if (ErrorPercent > CalibrationError) CalibrationError = ErrorPercent; // 最大値を記録
    }
    while (CalibrationError > CALIBRATION_ERROR_THRESHOLD) GPIO_WriteBit(LED_PORT, LED_PIN, Bit_SET); // 誤差が閾値を超えた場合、LED点灯のまま停止

    SaveData();
    Delay_Ms(300); // 出荷時のキャリブレーション時、ここでスイッチ押下解除
    ChangeSettingFlag = 0;
}

// 初期化関数 ==============================
// GPIO初期化
void GPIO_Config(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // PA1: 入力、内部プルアップ
    GPIO_InitStruct.GPIO_Pin = TAP_SW_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(TAP_SW_PORT, &GPIO_InitStruct);

    // PA2: 出力、プッシュプル
    GPIO_InitStruct.GPIO_Pin = LED_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(LED_PORT, &GPIO_InitStruct);

    // PC6: 出力、プッシュプル
    GPIO_InitStruct.GPIO_Pin = TEST_PULSE_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(TEST_PULSE_PORT, &GPIO_InitStruct);

    // PD2: TIM1 CH1 PWM出力、オルタネートファンクション
    GPIO_InitStruct.GPIO_Pin = PWM_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP; // オルタネートファンクション、プッシュプル
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(PWM_PORT, &GPIO_InitStruct);

    // PD3, PD5, PD6をアナログ入力に設定（なくてもOK）
    GPIO_InitStruct.GPIO_Pin = TEST_PULSE_DETECT_PIN | TAP_DIV_SW_PIN | DELAY_POT_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(DELAY_POT_PORT, &GPIO_InitStruct);

    // PD4: 入力、内部プルダウン
    GPIO_InitStruct.GPIO_Pin = PT2399_DETECT_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(PT2399_DETECT_PORT, &GPIO_InitStruct);
}

// GPIO初期化 非キャリブレーション時 PC6とPD3を変更
void GPIO_Config_Normal_Mode(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // PC6: 入力
    GPIO_InitStruct.GPIO_Pin = TEST_PULSE_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(TEST_PULSE_PORT, &GPIO_InitStruct);

    // PD3: 入力
    GPIO_InitStruct.GPIO_Pin = TEST_PULSE_DETECT_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(TEST_PULSE_DETECT_PORT, &GPIO_InitStruct);
}

// TIM1 ディレイタイム設定用PWM 初期化
void TIM1_PWM_Config(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    // TIM1基本設定
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct = {0};
    TIM_TimeBaseStruct.TIM_Prescaler = 0;
    TIM_TimeBaseStruct.TIM_Period = 1000 - 1; // 48MHz / 1000 = 48kHz
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStruct);

    // TIM1 CH1 PWM設定（PD2）
    TIM_OCInitTypeDef TIM_OCInitStruct = {0};
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse = 20; // 初期デューティ
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM1, &TIM_OCInitStruct);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

// TIM2 時間計測用割り込み 初期化
void TIM2_Interrupt_Init(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_BaseInitStruct = {0};
    NVIC_InitTypeDef NVIC_InitStruct = {0};

    // TIM2設定 0.25msごとに割り込み
    TIM_BaseInitStruct.TIM_Prescaler = 0;
    TIM_BaseInitStruct.TIM_Period = 12000 - 1; // 48MHz / 12000 = 4KHz
    TIM_BaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_BaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &TIM_BaseInitStruct);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

    // 割り込み設定
    NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

// ADC 初期化
void ADC_InitConfig(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    
    RCC_ADCCLKConfig(RCC_PCLK2_Div16); // ADCクロック設定（48MHz / 16 = 3MHz）
    ADC_InitTypeDef ADC_InitStructure = {0};
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; // 独立モード
    ADC_InitStructure.ADC_ScanConvMode = DISABLE; // スキャンモード無効（循環モードなし）
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // 連続変換無効
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // 外部トリガなし
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // 右揃え
    ADC_InitStructure.ADC_NbrOfChannel = 1; // 1チャンネルずつ変換
    ADC_Init(ADC1, &ADC_InitStructure);

    // サンプリング時間設定（なくてもOK）
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_PT2399OUT, 1, ADC_SampleTime_241Cycles); // PD3
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_TAP_DIV, 1, ADC_SampleTime_241Cycles); // PD5
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_DELAY, 1, ADC_SampleTime_241Cycles); // PD6

    // ADC有効化、キャリブレーション
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));
}

// その他関数 ==============================

// LED点滅
void BlinkLED(uint8_t n) {
    for (int i = 0; i < n; i++) {
        GPIO_WriteBit(LED_PORT, LED_PIN, Bit_SET);
        Delay_Ms(100);
        GPIO_WriteBit(LED_PORT, LED_PIN, Bit_RESET);
        Delay_Ms(100);
    }
}

// ADC値取得
uint16_t ADC_GetValue(uint8_t channel) {
    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_241Cycles); // channel設定 毎回必要
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)); // 変換完了待機
    return ADC_GetConversionValue(ADC1);
}

// TIM2割り込みハンドラ 時間測定用タイマー タップスイッチ読取
void TIM2_IRQHandler(void) {
    static uint32_t SwPressTicks = 0; // スイッチが押されているときカウントアップ
    static uint16_t TapCnt = 0; // タップ回数
    static uint16_t TapTicks[MAX_TAP_INTERVALS] = {}; // タップ間隔時間カウント数
    uint16_t TapTicksSum = 0; // TapTicksの合計 平均値算出に使用

    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        TimerTick++;

        // 設定変更、キャリブレーション時 処理なし
        if (ChangeSettingFlag) return;

        // スイッチ状態確認 タップ間隔記録 ディレイタイム計算
        if (!GPIO_ReadInputDataBit(TAP_SW_PORT, TAP_SW_PIN)) {
            SwPressTicks++;
            if (SwPressTicks == SHORT_PRESS_MS * TICKS_PER_1MS) { // スイッチ短押し検出
                if (TimerTick < MIN_DELAY_MS * TICKS_PER_1MS) {} // タップ間隔50ms未満 処理なし
                else if (TimerTick < MAX_TAP_INTERVAL_MS * TICKS_PER_1MS) { // タップ間隔1s未満
                    uint8_t TapTicksIndex = TapCnt % MAX_TAP_INTERVALS; // 添字 0～3で循環
                    TapTicks[TapTicksIndex] = TimerTick; // タップ間隔記録
                    for (int i = 0; i < MAX_TAP_INTERVALS; i++) TapTicksSum += TapTicks[i]; // 合計計算
                    if (TapCnt < MAX_TAP_INTERVALS) TapDelay = TapTicksSum / (TapCnt + 1) / TICKS_PER_1MS; // 平均計算
                    else TapDelay = TapTicksSum / MAX_TAP_INTERVALS / TICKS_PER_1MS;
                    TapCnt++;
                }
                else { // タップ間隔1s以上でリセット タップテンポ無効にせず継続
                    TapCnt = 0;
                    TapTicksSum = 0;
                    for (int i = 0; i < MAX_TAP_INTERVALS; i++) TapTicks[i] = 0;
                }
                TimerTick = 0; // TimerTickリセット
            }
        } else {
            SwPressTicks = 0;
        }
    }
}

// 実測したディレイタイム配列に基づきデューティ比を設定
void UpdatePWM(uint16_t Delay) {
    uint8_t k = 0; // 実測ディレイタイム配列の添字 12以下

    // 念のためディレイタイムを制限
    if (Delay > MAX_MEASURE_DELAY_MS) Delay = MAX_MEASURE_DELAY_MS;
    if (Delay < MIN_DELAY_MS) Delay = MIN_DELAY_MS;

    // ディレイタイムがどの範囲か判定
    for (k = 0; k < 13; k++) {
        if (10 * Delay > CalibratedDelays[k+1]) break;
    }
    if (k > 12) k = 12;

    // 線形補間でデューティ比の逆数を計算 マイナスが出るため符号ありにする
    int32_t x0 = CalibratedDelays[k];
    int32_t x1 = CalibratedDelays[k + 1];
    int32_t y0 = CalibrationDutyReciprocal[k];
    int32_t y1 = CalibrationDutyReciprocal[k + 1];
    int32_t y = y0 + (10 * Delay - x0) * (y1 - y0) / (x1 - x0);
    uint32_t duty = 1000000000 / y; // デューティ比（100000倍）

    // デューティ比=CCR/(ARR+1) (CCR: Capture/Compare Register, ARR: Auto-Reload Register)
    // デューティ比が0.2以下では精度が悪くなるため、分母のARRを減らして細かく調整(PWM周波数はわずかに変化)
    uint16_t arr = 1000 - 1;   // TIM1 初期ARR
    uint32_t ccr = duty / 100; // 下2桁切り捨て
    if (ccr < 200) {
        for (uint8_t i = 0; i < 60; i++) {
            if (100000 * ccr / (arr + 1 - i) >= duty) {
                arr = arr - i;
                break;
            }
        }
    }

    // デューティ比が大き過ぎるとディレイタイムが増える個体があるため、最大値までに制限
    if (ccr > MAX_DUTY) ccr = MAX_DUTY;

    // ARRとCCRを反映
    TIM1->ATRLR = arr;
    TIM_SetCompare1(TIM1, ccr);
}

// フラッシュ書き込み
void SaveData() {
    uint32_t addr = FLASH_ADDR;

    FLASH_Unlock();
    FLASH_ErasePage(addr);

    FLASH_ProgramHalfWord(addr, PotBlinkCtrl);
    addr += 2;
    for (int i = 0; i < 14; i++) {
        FLASH_ProgramHalfWord(addr, DefaultDelays[i]);
        addr += 2;
    }
    for (int i = 0; i < 14; i++) {
        FLASH_ProgramHalfWord(addr, CalibratedDelays[i]);
        addr += 2;
    }

    FLASH_Lock();
}

// フラッシュ読み込み 異常な値の場合はデフォルト値とする
void LoadData() {
    uint32_t addr = FLASH_ADDR;

    PotBlinkCtrl = *(uint16_t*)addr;
    addr += 2;
    for (int i = 0; i < 14; i++) {
        DefaultDelays[i] = *(uint16_t*)addr;
        if (DefaultDelays[i] > 10 * MAX_MEASURE_DELAY_MS || DefaultDelays[i] < 10 * MIN_DELAY_MS) DefaultDelays[i] = InitialDefaultDelays[i];
        addr += 2;
    }
    for (int i = 0; i < 14; i++) {
        CalibratedDelays[i] = *(uint16_t*)addr;
        if (CalibratedDelays[i] > 10 * MAX_MEASURE_DELAY_MS || CalibratedDelays[i] < 10 * MIN_DELAY_MS) CalibratedDelays[i] = DefaultDelays[i];
        addr += 2;
    }
}
