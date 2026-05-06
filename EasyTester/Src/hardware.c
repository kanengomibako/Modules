#include "common.h"
#include "hardware.h"
#include "ch32v00X.h"
#include "main.h"

// 初期化関数 ==============================
// GPIO初期化
void GPIOs_Init(void)
{
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA | RCC_PB2Periph_GPIOC | RCC_PB2Periph_GPIOD | RCC_PB2Periph_AFIO, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;

    // PC7: GPIO入力（プルアップ）
    GPIO_InitStructure.GPIO_Pin = SW_C_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(SW_C_PORT, &GPIO_InitStructure);  

    // PD0, PD3: GPIO入力（プルアップ）
    GPIO_InitStructure.GPIO_Pin = SW_B_PIN | SW_A_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(SW_B_PORT, &GPIO_InitStructure);

    // PC0, PC3, PC4, PC5, PC6: 基準抵抗設定ピン GPIO入力
    GPIO_InitStructure.GPIO_Pin = R0_PIN | R1_PIN | R2_PIN | R3_PIN | R4_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(R0_PORT, &GPIO_InitStructure);

    // PD6: LED GPIO出力
    GPIO_InitStructure.GPIO_Pin = LED_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(LED_PORT, &GPIO_InitStructure);

    // PD2: TIM1 CH1 PWM出力、オルタネートファンクション
    GPIO_InitStructure.GPIO_Pin = PWM_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // オルタネートファンクション、プッシュプル
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(PWM_PORT, &GPIO_InitStructure);

    // PD5: アナログ入力に設定（なくてもOK）
    GPIO_InitStructure.GPIO_Pin = VOLT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(VOLT_PORT, &GPIO_InitStructure);

    // PA1, PA2: アナログ入力（オペアンプ）
    GPIO_InitStructure.GPIO_Pin = OPA_IN_N_PIN | OPA_IN_P_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(OPA_IN_N_PORT, &GPIO_InitStructure);

    // PD4: オペアンプ出力、オルタネートファンクション
    GPIO_InitStructure.GPIO_Pin = OPA_OUT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 代替機能出力
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(OPA_OUT_PORT, &GPIO_InitStructure);

    // PC1, PC2: I2C
    GPIO_InitStructure.GPIO_Pin = I2C_SDA_PIN | I2C_SCL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(I2C_SDA_PORT, &GPIO_InitStructure);
}

void I2C1_Init(void)
{
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_I2C1, ENABLE);

    I2C_InitTypeDef I2C_InitStructure;

    I2C_InitStructure.I2C_ClockSpeed = 400000;
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_16_9;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitStructure);
    I2C_Cmd(I2C1, ENABLE);
}

// TIM1 PWM 初期化
void TIM1_PWM_Init(void)
{   
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_TIM1, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    TIM_TimeBaseStructure.TIM_Prescaler =  SystemCoreClock / 1000000 - 1; // 1MHz
    TIM_TimeBaseStructure.TIM_Period = UINT16_MAX;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    // TIM1 CH1 PWM設定（PD2）
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0; // 初期デューティ比 0
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, DISABLE);
    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

// TIM2 初期化（1msごとに割り込み）
void TIM2_Init(void) {
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_TIM2, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    // TIM2設定: 1msごとに割り込み
    TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock / 100000 - 1; // 100kHz
    TIM_TimeBaseStructure.TIM_Period = 99;  // 100kHz / 100 = 1kHz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // Update割り込み有効
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    // NVIC設定（優先度グループ2を想定）
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // タイマー有効
    TIM_Cmd(TIM2, ENABLE);
}

// OPA0 初期化
void OPAMP_Init(void)
{
    OPA_InitTypeDef OPA_InitStructure;

    OPA_Unlock();
    OPA_InitStructure.PSEL = CHP0;
    OPA_InitStructure.NSEL = CHN0;
    OPA_InitStructure.Mode = OUT_IO_OUT0;
    OPA_InitStructure.OPA_HS = HS_OFF;
    OPA_Init(&OPA_InitStructure);
    OPA->CFGR1 = 0x00000000; // ポーリングを確実にオフ
    OPA_Cmd(ENABLE);
}

// ADC 初期化
void ADC1_Init(void)
{
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_ADC1, ENABLE);
    
    ADC_InitTypeDef ADC_InitStructure;
    
    RCC_ADCCLKConfig(RCC_PCLK2_Div64); // ADCクロック設定（48MHz / 64 = 750kHz）
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; // 独立モード
    ADC_InitStructure.ADC_ScanConvMode = DISABLE; // スキャンモード無効（循環モードなし）
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // 連続変換無効
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // 外部トリガなし
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // 右揃え
    ADC_InitStructure.ADC_NbrOfChannel = 1; // 1チャンネルずつ変換
    ADC_Init(ADC1, &ADC_InitStructure);

    // サンプリング時間設定
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_VOLT, 1, ADC_SampleTime_CyclesMode7);

    // ADC有効化、CH32V006はキャリブレーション機能なし
    ADC_Cmd(ADC1, ENABLE);
}
