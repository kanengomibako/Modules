#ifndef _MEASURE_H
#define _MEASURE_H

uint32_t ADC_GetValue_Once(void);
uint32_t ADC_GetValue_Sum(uint32_t);
float Correct_ADC(uint32_t, uint32_t);

void Reset_Reference_Resistor(void);
void Set_Reference_Resistor(uint8_t);
void R0_Reset(void);

uint32_t Get_Volt(void);
uint32_t Get_Ohm(void);
void Continuity_Check(uint32_t);

void Run_ADC_Calibration_0V(void);
void Run_ADC_Calibration_XV(void);
void Run_Ohm_Calibration(void);

#endif  // _MEASURE_H