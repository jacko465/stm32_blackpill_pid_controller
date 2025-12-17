#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "motor.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

// LOOP FREQUENCIES
#define PID_LOOP_FREQUENCY_HZ 1000.0f         // 1kHz PID loop
#define ENCODER_UPDATE_DIV 10                 // Update encoder reading every 10 PID loops
#define ENCODER_UPDATE_FREQUENCY_HZ (PID_LOOP_FREQUENCY_HZ / ENCODER_UPDATE_DIV)

#define PID_UPDATE_PERIOD (1.0f / PID_LOOP_FREQUENCY_HZ)
#define ENCODER_UPDATE_PERIOD (1.0f / ENCODER_UPDATE_FREQUENCY_HZ)

void Error_Handler(void);
uint16_t Encoder_GetCount(TIM_HandleTypeDef *htim);
void SET_PWM_DC(TIM_HandleTypeDef *htim, uint8_t timer_channel, float duty_cycle);

#ifdef __cplusplus
}
#endif

#endif
