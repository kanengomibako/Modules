#ifndef _COMMON_H
#define _COMMON_H

// ピン定義 ==============================
#define R0_PORT GPIOC        // PC0 基準抵抗選択 0R
#define R0_PIN  GPIO_Pin_0
#define R1_PORT GPIOC        // PC3 基準抵抗選択 200R
#define R1_PIN  GPIO_Pin_3
#define R2_PORT GPIOC        // PC4 基準抵抗選択 2.2k
#define R2_PIN  GPIO_Pin_4
#define R3_PORT GPIOC        // PC5 基準抵抗選択 22k
#define R3_PIN  GPIO_Pin_5
#define R4_PORT GPIOC        // PC6 基準抵抗選択 220k
#define R4_PIN  GPIO_Pin_6
#define PWM_PORT GPIOD       // PD2 PWM出力 ブザー鳴動
#define PWM_PIN  GPIO_Pin_2
#define VOLT_PORT GPIOD      // PD5 電圧測定(ADC)
#define VOLT_PIN  GPIO_Pin_5
#define OPA_IN_N_PORT GPIOA  // PA1 オペアンプ-入力
#define OPA_IN_N_PIN  GPIO_Pin_1
#define OPA_IN_P_PORT GPIOA  // PA2 オペアンプ+入力
#define OPA_IN_P_PIN  GPIO_Pin_2
#define OPA_OUT_PORT GPIOD   // PD5 オペアンプ出力
#define OPA_OUT_PIN  GPIO_Pin_4
#define I2C_SDA_PORT GPIOC   // PC1 I2C SDA
#define I2C_SDA_PIN  GPIO_Pin_1
#define I2C_SCL_PORT GPIOC   // PC2 I2C SCL
#define I2C_SCL_PIN  GPIO_Pin_2
#define SW_C_PORT GPIOC      // PC7 入力ボタン 左側 C
#define SW_C_PIN  GPIO_Pin_7
#define SW_B_PORT GPIOD      // PD3 入力ボタン 中央 B
#define SW_B_PIN  GPIO_Pin_3
#define SW_A_PORT GPIOD      // PD0 入力ボタン 右側 A
#define SW_A_PIN  GPIO_Pin_0
#define LED_PORT GPIOD       // PD6 LED
#define LED_PIN  GPIO_Pin_6

// データ保存先フラッシュアドレス 末尾256バイト
#define FLASH_ADDR 0x0800F700

// ADC関係
#define ADC_CHANNEL_VOLT ADC_Channel_5 // ADCチャンネル
#define ADC_MAX_VALUE 4096.0f // ADC 10ビット最大値 float
#define ADC_SAMPLE_NUM 512 // ADC サンプリング回数
#define ADC_SAMPLE_NUM_VOLT (ADC_SAMPLE_NUM / 4) // 電圧 サンプリング回数
#define ADC_SAMPLE_NUM_OHM ADC_SAMPLE_NUM // 抵抗 サンプリング回数
#define RANGE_CHANGE_THRESHOLD_L 100 // 抵抗値レンジ切替閾値 低値側(不使用)
#define RANGE_CHANGE_THRESHOLD_H 440 // 抵抗値レンジ切替閾値 高値側
#define CALIBRATION_SAMPLE_NUM 16384 // キャリブレーション時 サンプリング回数

// 電圧 抵抗値
#define REF_VOLTAGE 3.3f // 基準電圧 V
#define OHM_CHANGE_VOLTAGE 200 // 抵抗表示に切り替える閾値電圧 mV
#define MAX_VOLTAGE 20500 // 測定可能最大電圧 mV
#define ADC_CALIBRATION_OFFSET_MAX 37.0f // ADCキャリブレーション時の異常値判定最大オフセット 30mV
#define VOLTAGE_ATTENUATE_FACTOR 6.25f // 入力電圧減衰倍率 8.6Mと1.4M
#define OPA_REF_VOLTAGE_FACTOR 0.070063694f // オペアンプ基準電圧設定値 2.2kと29.2kで22/314
#define PARALLEL_RESISTANCE 10000000.0f // 並列接続されている抵抗値 10MΩ
#define MAX_RESISTANCE 2300000 // 測定可能最大抵抗値

// スイッチ
#define SHORT_PRESS_MS 20 // 短押し判定時間
#define LONG_PRESS_MS 1250 // 長押し判定時間
#define CONTINUOUS_PRESS_MS 250 // 押し続け判定時間
#define STATUS_DISP_MS 1000 // ステータス表示時間

// OLED
#define SSD1306_HEIGHT 64
#define SSD1306_WIDTH  128
#define SSD1306_BUFFER_SIZE SSD1306_WIDTH * SSD1306_HEIGHT / 8
#define OLED_ADDR     0x78 // OLEDアドレス (0x3C << 1)
#define OLED_CMD_MODE 0x00 // コマンドモード
#define OLED_DAT_MODE 0x40 // データモード

// 導通設定、キャリブレーション画面 カーソル位置用変数
enum resister {R_0R, R_200R, R_2k2, R_22k, R_220k, R_MAX}; // 抵抗値レンジ番号用 0～3とGND接続

// 動作モード 0:通常 1:導通チェック設定 2:ADCキャリブレーション 3:抵抗値キャリブレーション
enum op_mode {MAIN, CONTINUITY_CHECK_SETTING, ADC_CALIBRATION, OHM_CALIBRATION, MODE_MAX};

#endif  // _COMMON_H