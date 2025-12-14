#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

// Motor params
#define ENCODER_PPR_BASIC 7             // Encoder pulses per revolution (BASIC)
#define ENCODER_QUADRATURE_MULTIPLIER 4 // Quadrature encoding multiplier
#define GEAR_RATIO 150                  // Motor gear ratio
#define ENCODER_COUNTS_PER_REV (ENCODER_PPR_BASIC * ENCODER_QUADRATURE_MULTIPLIER * GEAR_RATIO) // Encoder counts per revolution (7*4*150)=4200 output shaft

// PID gains
#define KP 0.1f
#define KI 0.125f
#define KD 0.0f

// Motor struct definition
typedef struct 
{
    // Encoder state
    volatile uint16_t prev_count;
    volatile uint16_t curr_count;
    volatile float rpm;
    volatile int16_t delta; // Encoder ticks per update interval

    // Encoder timer pointer
    TIM_HandleTypeDef *htim_encoder;

    // PID Gains
    float kp, ki, kd;

    // PID state
    float integral;
    float prev_error;
    volatile float output;

    // Target
    int32_t target_rpm;
    float target_delta;     // Target ticks per update interval

    // Integral clamp limit (anti-windup)
    float integral_limit;

    // Output limits
    float output_min;
    float output_max;

    // PWM outputs (GPIO / TIM channel)
    TIM_HandleTypeDef *htim_pwm;
    uint32_t pwm_channel_forward;
    uint32_t pwm_channel_reverse;

} Motor_t;

// Motor methods
void PID_UpdateMotor(Motor_t *motor);
void Update_Motor_RPM(Motor_t *motor);
void Motor_SetTargetRPM(Motor_t *motor, int32_t target_rpm);

void Motor_ApplyOutput(Motor_t *motor);
void Motor_Break(Motor_t *motor);
void Motor_Coast(Motor_t *motor);
void Motor_Forward(Motor_t *motor, float duty_cycle);
void Motor_Reverse(Motor_t *motor, float duty_cycle);

// External motor instances
extern Motor_t motor1;
// extern Motor_t motor2;
// extern Motor_t motor3;
// extern Motor_t motor4;

#endif // __MOTOR_H