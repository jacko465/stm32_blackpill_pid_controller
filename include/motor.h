#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

// Motor params
#define ENCODER_PPR_BASIC 11                // Encoder pulses per revolution (BASIC)
#define ENCODER_QUADRATURE_MULTIPLIER 2     // Quadrature encoding multiplier
#define GEAR_RATIO 35.5f                    // Motor gear ratio
#define ENCODER_COUNTS_PER_REV (ENCODER_PPR_BASIC * ENCODER_QUADRATURE_MULTIPLIER * GEAR_RATIO) // Encoder counts per revolution output shaft
// #define ENCODER_COUNTS_PER_REV 1562

// PID gains
#define KP 0.005f
#define KI 0.2f
#define KD 0.0f

// PID limits
#define PID_OUTPUT_MAX 1.0f                 // Maximum output (full speed)
#define PID_OUTPUT_MIN -1.0f                // Minimum output (reverse full speed)
#define PID_INTEGRAL_LIMIT 5.0f             // Anti-windup integral limit

extern uint8_t Motors_Enabled;
extern uint8_t ESTOP;

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
    volatile float integral;
    volatile float prev_error;
    volatile float output;

    // Target
    volatile uint8_t enabled;
    volatile int32_t target_rpm;
    // volatile float target_delta;     // Target ticks per update interval
    volatile float target_delta;

    // Integral clamp limit (anti-windup)
    float integral_limit;
    // Output limits
    float output_min;
    float output_max;

    // PWM setup
    TIM_HandleTypeDef *htim_pwm;    // PWM timer
    uint32_t pwm_channel;           // PWM channel

    // DIR pin setup
    GPIO_TypeDef *dir_port;         // Direction GPIO port
    uint32_t dir_pin;               // Direction GPIO pin

} Motor_t;

// External motor instances
extern Motor_t motor1;
extern Motor_t motor2;
extern Motor_t motor3;
extern Motor_t motor4;

// Motor methods
void PID_UpdateMotor(Motor_t *motor);
void Update_Motor_RPM(Motor_t *motor);
void Motor_SetTargetRPM(Motor_t *motor, int32_t target_rpm);
void Enable_Motors(void);
void Disable_Motors(void);
void SET_ESTOP(void);

void Motor_ApplyOutput(Motor_t *motor);
void Motor_Break(Motor_t *motor);
// void Motor_Coast(Motor_t *motor);
void Motor_Forward(Motor_t *motor, float duty_cycle);
void Motor_Reverse(Motor_t *motor, float duty_cycle);
void Set_Left_RPM(int32_t rpm);
void Set_Right_RPM(int32_t rpm);
void Spin_Left(int32_t rpm);
void Spin_Right(int32_t rpm);

#endif // __MOTOR_H