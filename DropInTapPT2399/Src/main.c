/*
DropInTap PT2399
v1.0 2025-06-17
https://github.com/kanengomibako

MCU: CH32V003F4U6
MounRiver Studio Version: 2.1.0

system_ch32v00x.c �򾎼������ڲ�����å����ä��O��
���󥳥���: #define SYSCLK_FREQ_48MHZ_HSI   48000000
�����ȥ�����: #define SYSCLK_FREQ_48MHz_HSE   48000000

Optimization -O2
*/

#include "debug.h"

// �ԥ��x ==============================
#define TAP_SW_PORT GPIOA // PA1 �ڲ��ץ륢�å� ���åץ����å�����
#define TAP_SW_PIN  GPIO_Pin_1
#define LED_PORT GPIOA // PA2 LED����
#define LED_PIN  GPIO_Pin_2
#define TEST_PULSE_PORT GPIOC // PC6 �����֥�`�����r���ǥ��쥤������Ӌ�y��ԇ�Y������ ��ͨ�������r��GPIO����
#define TEST_PULSE_PIN  GPIO_Pin_6
#define PWM_PORT GPIOD // PD2 PWM����
#define PWM_PIN  GPIO_Pin_2
#define TEST_PULSE_DETECT_PORT GPIOD // PD3 �ǥ��쥤������Ӌ�y��ԇ�Y���ʳ�(ADC) ��ͨ�������r��GPIO����
#define TEST_PULSE_DETECT_PIN  GPIO_Pin_3
#define PT2399_DETECT_PORT GPIOD // PD4 �ڲ��ץ������ PT2399���Ӵ_�J����
#define PT2399_DETECT_PIN  GPIO_Pin_4
#define TAP_DIV_SW_PORT GPIOD // PD5 TAP DIV�����å�(ADC)
#define TAP_DIV_SW_PIN  GPIO_Pin_5
#define DELAY_POT_PORT GPIOD // PD6 �ǥ��쥤�������O���ݥå�(ADC)
#define DELAY_POT_PIN  GPIO_Pin_6

// ����`�Х���� ���x ==============================
uint16_t PotBlinkCtrl = 1; // ���åץƥ��δ�O���r���ݥåȤΥǥ��쥤�������O���˺Ϥ碌��LED����礹��C��
uint8_t ChangeSettingFlag = 0; // �O������g���Хե饰
volatile uint16_t PotDelay = 0; // �ݥåȤ��O�������ǥ��쥤������
volatile uint16_t TapDelay = 0; // ���åפ��O�������ǥ��쥤������
const uint16_t CalibrationDuty[14] = {23, 25, 27, 30, 34, 38, 43, 51, 61, 75, 98, 140, 245, 700}; // �����֥�`������åǥ�`�ƥ���(1000��) �ǥ��쥤�����ब�s700ms��50ms�Ȥʤ낎
const uint32_t CalibrationDutyReciprocal[14] = {434783, 400000, 370370, 333333, 294118, 263158, 232558, 196078, 163934, 133333, 102041, 71429, 40816, 14286}; // ��ӛ�ǥ�`�ƥ��Ȥ�����
uint16_t DefaultDelays[14] = {}; // ���ɕr���O������g�y�ǥ��쥤������ǥե���Ȃ�(10��)
const uint16_t InitialDefaultDelays[14] = {7256, 6732, 6282, 5711, 5109, 4628, 4151, 3574, 3060, 2563, 2048, 1532, 1000, 534}; // �ǥ��쥤������ǥե���Ȃ��O��ǰ�γ��ڂ�
uint16_t CalibratedDelays[14] = {}; // �����֥�`�����ǌg�y�����ǥ��쥤������(10��)
#define FLASH_ADDR 0x08003FC0 // �ǩ`�������ȥե�å��奢�ɥ쥹 ĩβ64�Х���
#define CALIBRATION_ERROR_THRESHOLD 10 // �����֥�`�����ǌg�y�����ǥ��쥤������Υǥե���Ȃ�����Υ���򥨥�`�ж�����铂�(%)

// �r�g�O��
volatile uint32_t TimerTick = 10000; // �r�gӋ�y�� TIM1�θ���z��(0.25ms����)�ǥ�����ȥ��å�
#define TICKS_PER_1MS 4 // 1ms�������TimerTick���������
#define SHORT_PRESS_MS 25 // ��Ѻ���ж��r�g
#define LONG_PRESS_MS 2000 // �LѺ���ж��r�g
#define LONG_LONG_PRESS_MS 5000 // �L���LѺ���ж��r�g
#define LED_ON_MS 10 // ���åץƥ��LED���Ƥ����Ƥ����r�g
#define MAX_DELAY_MS 700 // �O���������ǥ��쥤������
#define MIN_DELAY_MS 50  // �O��������С�ǥ��쥤������
#define MAX_MEASURE_DELAY_MS 900 // �y���������ǥ��쥤������
#define MAX_DUTY 750 // ���ǥ�`�ƥ���(1000��) �� �ǥ��쥤��������С
#define MAX_TAP_INTERVAL_MS 1000 // ���å����r�g
#define MAX_TAP_INTERVALS 4 // ӛ�h���륿�å��g��������� ���å�5�أ�4�Ĥ��g��

// ADC�v�S
volatile uint16_t adc[3]; // ADCȡ�Â� PD3 (ADC4), PD5 (ADC5), PD6 (ADC6)
enum adcNum {PT2399OUT, TAP_DIV, DELAY}; // ��ӛ���Ф�����
#define ADC_CHANNEL_PT2399OUT ADC_Channel_4 // ADC_Channel ���x
#define ADC_CHANNEL_TAP_DIV ADC_Channel_5
#define ADC_CHANNEL_DELAY ADC_Channel_6
#define ADC_MAX_VALUE 1023 // ADC 10�ӥå� ���
#define TAP_DIV_THRESHOLD_HIGH (ADC_MAX_VALUE * 3 / 5) // TAPDIV�����å��ж�铂�H 3V
#define TAP_DIV_THRESHOLD_LOW (ADC_MAX_VALUE * 1 / 3)  // TAPDIV�����å��ж�铂�L 1.7V
#define DELAYED_SIGNAL_DETECT_THRESHOLD (ADC_MAX_VALUE * 3 / 5) // �ǥ��쥤�źŗʳ�铂�
#define POT_CHANGE_THRESHOLD 4 // �ݥåȤ�Ӥ��������ж�����铂�
#define ADC_ADJ_RESISTOR_KOHM 47 // ADC���a���� ADC�ԥ�˽ӾA���Ƥ���ֿ��΂�
#define ADC_ADJ_POT_KOHM 50 // ADC���a���� �ݥåȤεֿ���
#define ADC_POT_FAULT_VALUE 574 // �ݥåȤεֿ���������(60k��)�Εr��ADC�� 1023*60/(60+47)

// �ץ�ȥ��������� ==============================
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

    // PT2399�Υ���å������Ϥ�����C 500ms�̶�
    while (!GPIO_ReadInputDataBit(PT2399_DETECT_PORT, PT2399_DETECT_PIN)) {}

    // ���åץ����å���Ѻ���ʤ������Ӥ����r���O�����
    if (!GPIO_ReadInputDataBit(TAP_SW_PORT, TAP_SW_PIN)) ChangeSetting();

    // �ǥ����֥�`�����r��PC6��PD3��GPIO�������O��
    GPIO_Config_Normal_Mode();

    while (1) {
        static int16_t last_adcDELAY = 0; // ǰ�ؤ�adc[DELAY]
    
        adc[DELAY] = ADC_GetValue(ADC_CHANNEL_DELAY); // �ǥ��쥤������ݥå�
        if (adc[DELAY] > ADC_POT_FAULT_VALUE) adc[DELAY] = ADC_POT_FAULT_VALUE; // �ݥåȹ��ϕr�����

        // �ݥåȤ�Ӥ������r��adc[DELAY]���¤ȥ��åץƥ�ݽ��
        if (last_adcDELAY - adc[DELAY] > POT_CHANGE_THRESHOLD || adc[DELAY] - last_adcDELAY > POT_CHANGE_THRESHOLD) {
            last_adcDELAY = adc[DELAY];
            TapDelay = 0;
        }
        else {
            adc[DELAY] = last_adcDELAY; // �ݥåȤ��Ӥ��Ƥʤ����ж� ������¤��ʤ�
        }

        // �ǥ��쥤�ݥåȤ�ADC�΂����a����47k���ֿ������Ƥ��뤿�ᣩ
        uint32_t corrected_adcDELAY = ADC_ADJ_RESISTOR_KOHM * ADC_MAX_VALUE * adc[DELAY] / (ADC_MAX_VALUE - adc[DELAY]) / ADC_ADJ_POT_KOHM;

        // �ǥ��쥤������Ӌ�� �ݥåȤεֿ������󤭤������ǥ��쥤������򳬤�����Ϥ���
        PotDelay = MIN_DELAY_MS + (MAX_DELAY_MS - MIN_DELAY_MS) * corrected_adcDELAY / ADC_MAX_VALUE;
        uint16_t FinalDelay = PotDelay;

        // ���åץƥ���Є��r TAP_DIV��ӳ
        if (TapDelay) {
            adc[TAP_DIV] = ADC_GetValue(ADC_CHANNEL_TAP_DIV); // TapDiv�����å� ���1023
            if (adc[TAP_DIV] > TAP_DIV_THRESHOLD_HIGH) FinalDelay = TapDelay / 2;
            else if (adc[TAP_DIV] > TAP_DIV_THRESHOLD_LOW) FinalDelay = TapDelay * 3 / 4;
            else FinalDelay = TapDelay;
        }

        // �ǥ��쥤�����फ��ǥ�`�ƥ��Ȥˉ�Q����ӳ
        UpdatePWM(FinalDelay);

        // LED���
        uint16_t LED_BlinkTime = PotDelay; // LED����g���r�g
        if (TapDelay) LED_BlinkTime = TapDelay;
        if ((TapDelay || PotBlinkCtrl) && (TimerTick % (TICKS_PER_1MS * LED_BlinkTime) < LED_ON_MS * TICKS_PER_1MS)) {
            GPIO_WriteBit(LED_PORT, LED_PIN, Bit_SET);
        }
        else {
            GPIO_WriteBit(LED_PORT, LED_PIN, Bit_RESET);
        }

    }
}

// �O�����
void ChangeSetting(void) {
    ChangeSettingFlag = 1;
    TimerTick = 0; // �����ީ`�ꥻ�å�

    // ���åץ����å���Ѻ���A���Ƥ���r
    while (!GPIO_ReadInputDataBit(TAP_SW_PORT, TAP_SW_PIN)) {
        // �LѺ����LED��� �� LED����O�����
        if (TimerTick > LONG_PRESS_MS * TICKS_PER_1MS) GPIO_WriteBit(LED_PORT, LED_PIN, Bit_SET);
        // �L���LѺ����LED��Ƥ򰵤����� �� �����֥�`�����
        if (TimerTick > LONG_LONG_PRESS_MS * TICKS_PER_1MS) GPIO_WriteBit(LED_PORT, LED_PIN, Bit_RESET);
    }

    // ���åץ����å����x�����r
    if (TimerTick > LONG_LONG_PRESS_MS * TICKS_PER_1MS) Calibration();
    else if (TimerTick > LONG_PRESS_MS * TICKS_PER_1MS) {
        PotBlinkCtrl = !PotBlinkCtrl; // LED����O�����
        SaveData();
        BlinkLED(3);
        ChangeSettingFlag = 0;
    }
}

// �����֥�`�����:�O�������ǥ�`�ƥ��ȤΕr�Υǥ��쥤�������y�� ==============================
void Calibration(void) {
    uint16_t DelayTicks[5] = {}; // �y�����������ީ`�������(1ms������4�������)

    // 14�ݥ���ȤΥǥ��쥤�������y����ӛ�h
    for (int i = 0; i < 14; i++) {
        TIM_SetCompare1(TIM1, CalibrationDuty[i]); // �����֥�`�����ǥ�`�ƥ��Ȥ��O��
        Delay_Ms(100); // 늈R�����ޤǴ��C

        // 5�؜y����������3����ץ��ƽ����Ӌ��
        for (int j = 0; j < 5; j++) {
            GPIO_WriteBit(TEST_PULSE_PORT, TEST_PULSE_PIN, Bit_SET); // ԇ�Y�źų���
            TimerTick = 0;
            Delay_Ms(5);
            GPIO_WriteBit(TEST_PULSE_PORT, TEST_PULSE_PIN, Bit_RESET);
            GPIO_WriteBit(LED_PORT, LED_PIN, Bit_SET); // �y���Ф�LED���
            // ADC��ȡ��(0.1ms�����뤬�oҕ)���W���źŗʳ�
            while(TimerTick < MAX_MEASURE_DELAY_MS * TICKS_PER_1MS) {
                adc[PT2399OUT] = ADC_GetValue(ADC_CHANNEL_PT2399OUT);
                if (adc[PT2399OUT] < DELAYED_SIGNAL_DETECT_THRESHOLD) break;
            }
            DelayTicks[j] = TimerTick;
            GPIO_WriteBit(LED_PORT, LED_PIN, Bit_RESET); // �y�����ˤ�LED����
            Delay_Ms(10);
        }
        // �Х֥륽�`��
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4 - i; j++) {
                if (DelayTicks[j] > DelayTicks[j + 1]) {
                    uint16_t temp = DelayTicks[j];
                    DelayTicks[j] = DelayTicks[j + 1];
                    DelayTicks[j + 1] = temp;
                }
            }
        }
        // �y���ǥ��쥤������(10��)��Q��
        CalibratedDelays[i] = 10 * (DelayTicks[1] + DelayTicks[2] + DelayTicks[3]) / 3 / TICKS_PER_1MS;

        // �����å���Ѻ���A�����Ƥ������ϡ��y������ǥե���Ȃ��Ȥ��Ƶ��h(���ɕr��)
        if (!GPIO_ReadInputDataBit(TAP_SW_PORT, TAP_SW_PIN)) {
            Delay_Ms(10);
            if (!GPIO_ReadInputDataBit(TAP_SW_PORT, TAP_SW_PIN)) DefaultDelays[i] = CalibratedDelays[i];
        }
    }

    // �ǥե���Ȃ����������`��(%)��Ӌ��
    uint16_t CalibrationError = 0;
    for (int i = 0; i < 14; i++) {
        int32_t ErrorPercent = 100 * (CalibratedDelays[i] - DefaultDelays[i]) / DefaultDelays[i];
        if (ErrorPercent < 0) ErrorPercent = -ErrorPercent; // ؓ��������������
        if (ErrorPercent > CalibrationError) CalibrationError = ErrorPercent; // ��󂎤�ӛ�h
    }
    while (CalibrationError > CALIBRATION_ERROR_THRESHOLD) GPIO_WriteBit(LED_PORT, LED_PIN, Bit_SET); // �`�铂��򳬤������ϡ�LED��ƤΤޤ�ֹͣ

    SaveData();
    Delay_Ms(300); // ���ɕr�Υ����֥�`�����r�������ǥ����å�Ѻ�½��
    ChangeSettingFlag = 0;
}

// ���ڻ��v�� ==============================
// GPIO���ڻ�
void GPIO_Config(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // PA1: �������ڲ��ץ륢�å�
    GPIO_InitStruct.GPIO_Pin = TAP_SW_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(TAP_SW_PORT, &GPIO_InitStruct);

    // PA2: �������ץå���ץ�
    GPIO_InitStruct.GPIO_Pin = LED_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(LED_PORT, &GPIO_InitStruct);

    // PC6: �������ץå���ץ�
    GPIO_InitStruct.GPIO_Pin = TEST_PULSE_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(TEST_PULSE_PORT, &GPIO_InitStruct);

    // PD2: TIM1 CH1 PWM���������륿�ͩ`�ȥե��󥯥����
    GPIO_InitStruct.GPIO_Pin = PWM_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP; // ���륿�ͩ`�ȥե��󥯥���󡢥ץå���ץ�
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(PWM_PORT, &GPIO_InitStruct);

    // PD3, PD5, PD6�򥢥ʥ��������O�����ʤ��Ƥ�OK��
    GPIO_InitStruct.GPIO_Pin = TEST_PULSE_DETECT_PIN | TAP_DIV_SW_PIN | DELAY_POT_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(DELAY_POT_PORT, &GPIO_InitStruct);

    // PD4: �������ڲ��ץ������
    GPIO_InitStruct.GPIO_Pin = PT2399_DETECT_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(PT2399_DETECT_PORT, &GPIO_InitStruct);
}

// GPIO���ڻ� �ǥ����֥�`�����r PC6��PD3����
void GPIO_Config_Normal_Mode(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // PC6: ����
    GPIO_InitStruct.GPIO_Pin = TEST_PULSE_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(TEST_PULSE_PORT, &GPIO_InitStruct);

    // PD3: ����
    GPIO_InitStruct.GPIO_Pin = TEST_PULSE_DETECT_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(TEST_PULSE_DETECT_PORT, &GPIO_InitStruct);
}

// TIM1 �ǥ��쥤�������O����PWM ���ڻ�
void TIM1_PWM_Config(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    // TIM1�����O��
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct = {0};
    TIM_TimeBaseStruct.TIM_Prescaler = 0;
    TIM_TimeBaseStruct.TIM_Period = 1000 - 1; // 48MHz / 1000 = 48kHz
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStruct);

    // TIM1 CH1 PWM�O����PD2��
    TIM_OCInitTypeDef TIM_OCInitStruct = {0};
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse = 20; // ���ڥǥ�`�ƥ�
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM1, &TIM_OCInitStruct);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

// TIM2 �r�gӋ�y�ø���z�� ���ڻ�
void TIM2_Interrupt_Init(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_BaseInitStruct = {0};
    NVIC_InitTypeDef NVIC_InitStruct = {0};

    // TIM2�O�� 0.25ms���Ȥ˸���z��
    TIM_BaseInitStruct.TIM_Prescaler = 0;
    TIM_BaseInitStruct.TIM_Period = 12000 - 1; // 48MHz / 12000 = 4KHz
    TIM_BaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_BaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &TIM_BaseInitStruct);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

    // ����z���O��
    NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

// ADC ���ڻ�
void ADC_InitConfig(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    
    RCC_ADCCLKConfig(RCC_PCLK2_Div16); // ADC����å��O����48MHz / 16 = 3MHz��
    ADC_InitTypeDef ADC_InitStructure = {0};
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; // ������`��
    ADC_InitStructure.ADC_ScanConvMode = DISABLE; // ��������`�ɟo����ѭ�h��`�ɤʤ���
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // �B�A��Q�o��
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // �ⲿ�ȥꥬ�ʤ�
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // �ғB��
    ADC_InitStructure.ADC_NbrOfChannel = 1; // 1�����ͥ뤺�ĉ�Q
    ADC_Init(ADC1, &ADC_InitStructure);

    // ����ץ�󥰕r�g�O�����ʤ��Ƥ�OK��
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_PT2399OUT, 1, ADC_SampleTime_241Cycles); // PD3
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_TAP_DIV, 1, ADC_SampleTime_241Cycles); // PD5
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_DELAY, 1, ADC_SampleTime_241Cycles); // PD6

    // ADC�Є����������֥�`�����
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));
}

// �������v�� ==============================

// LED���
void BlinkLED(uint8_t n) {
    for (int i = 0; i < n; i++) {
        GPIO_WriteBit(LED_PORT, LED_PIN, Bit_SET);
        Delay_Ms(100);
        GPIO_WriteBit(LED_PORT, LED_PIN, Bit_RESET);
        Delay_Ms(100);
    }
}

// ADC��ȡ��
uint16_t ADC_GetValue(uint8_t channel) {
    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_241Cycles); // channel�O�� ���ر�Ҫ
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)); // ��Q���˴��C
    return ADC_GetConversionValue(ADC1);
}

// TIM2����z�ߥϥ�ɥ� �r�g�y���å����ީ` ���åץ����å��iȡ
void TIM2_IRQHandler(void) {
    static uint32_t SwPressTicks = 0; // �����å���Ѻ����Ƥ���Ȥ�������ȥ��å�
    static uint16_t TapCnt = 0; // ���å׻���
    static uint16_t TapTicks[MAX_TAP_INTERVALS] = {}; // ���å��g���r�g���������
    uint16_t TapTicksSum = 0; // TapTicks�κ�Ӌ ƽ���������ʹ��

    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        TimerTick++;

        // �O������������֥�`�����r �I��ʤ�
        if (ChangeSettingFlag) return;

        // �����å�״�B�_�J ���å��g��ӛ�h �ǥ��쥤������Ӌ��
        if (!GPIO_ReadInputDataBit(TAP_SW_PORT, TAP_SW_PIN)) {
            SwPressTicks++;
            if (SwPressTicks == SHORT_PRESS_MS * TICKS_PER_1MS) { // �����å���Ѻ���ʳ�
                if (TimerTick < MIN_DELAY_MS * TICKS_PER_1MS) {} // ���å��g��50msδ�� �I��ʤ�
                else if (TimerTick < MAX_TAP_INTERVAL_MS * TICKS_PER_1MS) { // ���å��g��1sδ��
                    uint8_t TapTicksIndex = TapCnt % MAX_TAP_INTERVALS; // ���� 0��3��ѭ�h
                    TapTicks[TapTicksIndex] = TimerTick; // ���å��g��ӛ�h
                    for (int i = 0; i < MAX_TAP_INTERVALS; i++) TapTicksSum += TapTicks[i]; // ��ӋӋ��
                    if (TapCnt < MAX_TAP_INTERVALS) TapDelay = TapTicksSum / (TapCnt + 1) / TICKS_PER_1MS; // ƽ��Ӌ��
                    else TapDelay = TapTicksSum / MAX_TAP_INTERVALS / TICKS_PER_1MS;
                    TapCnt++;
                }
                else { // ���å��g��1s���Ϥǥꥻ�å� ���åץƥ�ݟo���ˤ����@�A
                    TapCnt = 0;
                    TapTicksSum = 0;
                    for (int i = 0; i < MAX_TAP_INTERVALS; i++) TapTicks[i] = 0;
                }
                TimerTick = 0; // TimerTick�ꥻ�å�
            }
        } else {
            SwPressTicks = 0;
        }
    }
}

// �g�y�����ǥ��쥤���������Ф˻��Ť��ǥ�`�ƥ��Ȥ��O��
void UpdatePWM(uint16_t Delay) {
    uint8_t k = 0; // �g�y�ǥ��쥤���������Ф����� 12����

    // ��Τ���ǥ��쥤�����������
    if (Delay > MAX_MEASURE_DELAY_MS) Delay = MAX_MEASURE_DELAY_MS;
    if (Delay < MIN_DELAY_MS) Delay = MIN_DELAY_MS;

    // �ǥ��쥤�����ब�ɤι��줫�ж�
    for (k = 0; k < 13; k++) {
        if (10 * Delay > CalibratedDelays[k+1]) break;
    }
    if (k > 12) k = 12;

    // �����a�g�ǥǥ�`�ƥ��Ȥ�������Ӌ�� �ޥ��ʥ������뤿����Ť���ˤ���
    int32_t x0 = CalibratedDelays[k];
    int32_t x1 = CalibratedDelays[k + 1];
    int32_t y0 = CalibrationDutyReciprocal[k];
    int32_t y1 = CalibrationDutyReciprocal[k + 1];
    int32_t y = y0 + (10 * Delay - x0) * (y1 - y0) / (x1 - x0);
    uint32_t duty = 1000000000 / y; // �ǥ�`�ƥ��ȣ�100000����

    // �ǥ�`�ƥ���=CCR/(ARR+1) (CCR: Capture/Compare Register, ARR: Auto-Reload Register)
    // �ǥ�`�ƥ��Ȥ�0.2���¤ǤϾ��Ȥ������ʤ뤿�ᡢ��ĸ��ARR��p�餷�Ƽ������{��(PWM�ܲ����Ϥ鷺���ˉ仯)
    uint16_t arr = 1000 - 1;   // TIM1 ����ARR
    uint32_t ccr = duty / 100; // ��2���Ф�Τ�
    if (ccr < 200) {
        for (uint8_t i = 0; i < 60; i++) {
            if (100000 * ccr / (arr + 1 - i) >= duty) {
                arr = arr - i;
                break;
            }
        }
    }

    // �ǥ�`�ƥ��Ȥ����^����ȥǥ��쥤�����ब�����낀�夬���뤿�ᡢ��󂎤ޤǤ�����
    if (ccr > MAX_DUTY) ccr = MAX_DUTY;

    // ARR��CCR��ӳ
    TIM1->ATRLR = arr;
    TIM_SetCompare1(TIM1, ccr);
}

// �ե�å�������z��
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

// �ե�å����i���z�� �����ʂ��Έ��Ϥϥǥե���Ȃ��Ȥ���
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
