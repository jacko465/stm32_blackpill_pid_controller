#include "motor.h"

// motor instances
Motor_t motor1 = {
    // Encoder state
    .prev_count = 0,
    .curr_count = 0,
    .rpm = 0.0f,
    .delta = 0,

    // Encoder timer pointer
    .htim_encoder = &htim5, // TIM5 (PA0/PA1) Motor 1 encoder

    //PID gains
    .kp = KP,
    .ki = KI,
    .kd = KD,

    // PID state
    .integral = 0.0f,
    .prev_error = 0.0f,
    .output = 0.0f,

    // Target
    .target_rpm = 0,
    .target_delta = 0,

    // Integral limit (anti-windup) 5x output clamp
    .integral_limit = PID_INTEGRAL_LIMIT,

    // Output limits
    .output_min = PID_OUTPUT_MIN,
    .output_max = PID_OUTPUT_MAX,

    // PWM output
    .htim_pwm = &htim1,                     // TIM1
    .pwm_channel = TIM_CHANNEL_1,           // PA8

    // DIR pin
    .dir_port = GPIOB,                      // PB0
    .dir_pin = GPIO_PIN_0                   // PB0
};

Motor_t motor2 = {
    // Encoder state
    .prev_count = 0,
    .curr_count = 0,
    .rpm = 0.0f,
    .delta = 0,

    // Encoder timer pointer
    .htim_encoder = &htim3, // TIM3 (PA6/PA7) Motor 2 encoder

    //PID gains
    .kp = KP,
    .ki = KI,
    .kd = KD,

    // PID state
    .integral = 0.0f,
    .prev_error = 0.0f,
    .output = 0.0f,

    // Target
    .target_rpm = 0,
    .target_delta = 0,

    // Integral limit (anti-windup) 5x output clamp
    .integral_limit = PID_INTEGRAL_LIMIT,

    // Output limits
    .output_min = PID_OUTPUT_MIN,
    .output_max = PID_OUTPUT_MAX,
    // PWM output
    .htim_pwm = &htim1,                     // TIM1
    .pwm_channel = TIM_CHANNEL_2,           // PA9

    // DIR pin
    .dir_port = GPIOB,                      // PB0
    .dir_pin = GPIO_PIN_1                   // PB1
};

Motor_t motor3 = {
    // Encoder state
    .prev_count = 0,
    .curr_count = 0,
    .rpm = 0.0f,
    .delta = 0,

    // Encoder timer pointer
    .htim_encoder = &htim4, // TIM4 (PB6/PB7) Motor 3 encoder

    //PID gains
    .kp = KP,
    .ki = KI,
    .kd = KD,

    // PID state
    .integral = 0.0f,
    .prev_error = 0.0f,
    .output = 0.0f,

    // Target
    .target_rpm = 0,
    .target_delta = 0,

    // Integral limit (anti-windup) 5x output clamp
    .integral_limit = PID_INTEGRAL_LIMIT,

    // Output limits
    .output_min = PID_OUTPUT_MIN,
    .output_max = PID_OUTPUT_MAX,
    // PWM output
    .htim_pwm = &htim1,                     // TIM1
    .pwm_channel = TIM_CHANNEL_3,           // PA10

    // DIR pin
    .dir_port = GPIOB,                      // PB0
    .dir_pin = GPIO_PIN_2                   // PB2
};

Motor_t motor4 = {
    // Encoder state
    .prev_count = 0,
    .curr_count = 0,
    .rpm = 0.0f,
    .delta = 0,

    // Encoder timer pointer
    .htim_encoder = &htim2, // TIM2 (PA0/PA1) Motor 4 encoder

    //PID gains
    .kp = KP,
    .ki = KI,
    .kd = KD,

    // PID state
    .integral = 0.0f,
    .prev_error = 0.0f,
    .output = 0.0f,

    // Target
    .target_rpm = 0,
    .target_delta = 0,

    // Integral limit (anti-windup) 5x output clamp
    .integral_limit = PID_INTEGRAL_LIMIT,

    // Output limits
    .output_min = PID_OUTPUT_MIN,
    .output_max = PID_OUTPUT_MAX,
    // PWM output
    .htim_pwm = &htim1,                     // TIM1
    .pwm_channel = TIM_CHANNEL_4,           // PA11

    // DIR pin
    .dir_port = GPIOB,                      // PB0
    .dir_pin = GPIO_PIN_10                  // PB10
};

// PID Loop function
void PID_UpdateMotor(Motor_t *motor)
{
    // Compute error terms
    float error = (float)motor->target_delta - (float)(motor->delta);
    motor->integral += error * PID_UPDATE_PERIOD;
    float derivative = (error - motor->prev_error) / PID_UPDATE_PERIOD;
    motor->prev_error = error;

    // Clamp integral to prevent windup
    if (motor->integral > motor->integral_limit) motor->integral = motor->integral_limit;
    if (motor->integral < -motor->integral_limit) motor->integral = -motor->integral_limit;

    // Compute PID
    motor->output = (motor->kp * error) + (motor->ki * motor->integral) + (motor->kd * derivative);

    // Clamp output
    if (motor->output > motor->output_max) motor->output = motor->output_max;
    if (motor->output < motor->output_min) motor->output = motor->output_min;

    // Apply output to motor
    Motor_ApplyOutput(motor);
}

// Update motor velocity (call every ~10ms)
void Update_Motor_RPM(Motor_t *motor)
{
    motor->prev_count = motor->curr_count;
    motor->curr_count = Encoder_GetCount(motor->htim_encoder);
    motor->delta = (int16_t)(motor->curr_count - motor->prev_count);
    float counts_per_sec = motor->delta * ENCODER_UPDATE_FREQUENCY_HZ;
    motor->rpm = counts_per_sec * 60.0f / ENCODER_COUNTS_PER_REV;
}

// Set motor RPM target
void Motor_SetTargetRPM(Motor_t *motor, int32_t target_rpm)
{
    motor->target_rpm = target_rpm;
    motor->target_delta = (int32_t)(target_rpm * ((float)ENCODER_COUNTS_PER_REV / 60.0f) * ENCODER_UPDATE_PERIOD);
}

// motor driving methods

// Motor apply output (set by PID)
void Motor_ApplyOutput(Motor_t *motor)
{
    float duty_cycle = motor->output;
    if (duty_cycle > 0.0f)
    {
        Motor_Forward(motor, duty_cycle);
    }
    else if (duty_cycle < 0.0f)
    {
        Motor_Reverse(motor, -duty_cycle);
    }
    else
    {
        Motor_Break(motor);
    }
}

// Motor break function
void Motor_Break(Motor_t *motor)
{
    SET_PWM_DC(motor->htim_pwm, motor->pwm_channel, 0.0f);
    HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_SET);
}

// Motor coast function
// void Motor_Coast(Motor_t *motor)
// {
//     SET_PWM_DC(motor->htim_pwm, motor->pwm_channel, 0.0f);
//     HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_RESET);
// }

// Basic motor forward function
void Motor_Forward(Motor_t *motor, float duty_cycle)
{
    SET_PWM_DC(motor->htim_pwm, motor->pwm_channel, duty_cycle);
    HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_RESET);
}

// Basic motor reverse function
void Motor_Reverse(Motor_t *motor, float duty_cycle)
{
    SET_PWM_DC(motor->htim_pwm, motor->pwm_channel, duty_cycle);
    HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_SET);
}