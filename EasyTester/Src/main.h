#ifndef _MAIN_H
#define _MAIN_H

void TIM2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void sw_process(void);
void Sw_C_Short_Push(void);
void Sw_B_Short_Push(void);
void Sw_A_Short_Push(void);
void Sw_C_Long_Push(void);
void Sw_B_Long_Push(void);
void Sw_A_Long_Push(void);
void Sw_C_Continuous_Push(void);
void Sw_B_Continuous_Push(void);
void Sw_A_Continuous_Push(void);

#endif // _MAIN_H
