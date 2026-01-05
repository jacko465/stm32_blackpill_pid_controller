#include "main.h"
#include <string.h>
#include <stdbool.h>

void SystemClock_Config(void);
uint8_t rx_dma_buf[RX_BUF_LEN]; // DMA RX buffer
static volatile uint16_t rx_old_pos = 0; // Old position in RX DMA buffer

static char line_buf[LINE_MAX];
static volatile uint16_t line_len = 0;

static volatile bool msg_ready = false;
static char msg_buf[LINE_MAX];

static void ProcessBytes(const uint8_t *data, uint16_t len);

int main(void)
{
	// Init HAL
	HAL_Init();

	// Configure the system clock
	SystemClock_Config();

	// Initialize all configured peripherals
	MX_GPIO_Init();
	MX_DMA_Init();		// DMA for USART2 RX
	MX_TIM1_Init();		// PWM timer
	MX_TIM2_Init();		// Encoder timer Motor 4
	MX_TIM3_Init();		// Encoder timer Motor 2
	MX_TIM4_Init();		// Encoder timer Motor 3
	MX_TIM5_Init();		// Encoder timer Motor 1
	MX_TIM10_Init();	// Interrupt timer for PID loop
	MX_USART2_UART_Init();	// Init USART2
	HAL_UART_Receive_DMA(&huart2, rx_dma_buf, RX_BUF_LEN); // Start USART2 RX in DMA mode
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE); // Enable IDLE line interrupt for USART2

	// Start encoder timers
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL); // Motor 1 encoder
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // Motor 2 encoder
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // Motor 3 encoder
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // Motor 4 encoder

	// Start PWM timers
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // Motor 1 PWM
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // Motor 2 PWM
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // Motor 3 PWM
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // Motor 4 PWM

	// Start TIM10 interrupt timer (PID Loop)
	HAL_TIM_Base_Start_IT(&htim10);

	// Main loop
	while (1)
	{
		// __WFI();	// Wait for interrupt to save power

		// Motor_SetTargetRPM(&motor1, 50);
		// Motor_SetTargetRPM(&motor2, 50);
		// Motor_SetTargetRPM(&motor3, 50);
		// Motor_SetTargetRPM(&motor4, 50);

		// printf(">");
        // printf("motor1rpm:%.2f", motor1.rpm);
        // printf(",motor1output:%.2f", motor1.output);
		// printf(",motor2rpm:%.2f", motor2.rpm);
        // printf(",motor2output:%.2f", motor2.output);
		// printf(",motor3rpm:%.2f", motor3.rpm);
        // printf(",motor3output:%.2f", motor3.output);
		// printf(",motor4rpm:%.2f", motor4.rpm);
        // printf(",motor4output:%.2f", motor4.output);
        // printf("\r\n");
        // HAL_Delay(50);

		// Handle received USART messages
		if (msg_ready)
		{
			msg_ready = false;
			Handle_USART_Message();
		}

		// Report status over serial

	}
}

// Timer ISR callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// Counter for slower encoder updates
	static uint16_t encoder_update_counter = 0;

	if (htim->Instance == TIM10)
	{
		encoder_update_counter++;
		if (encoder_update_counter >= ENCODER_UPDATE_DIV)	// every ENCODER_UPDATE_DIV PID loops
		{
			encoder_update_counter = 0;
			// Update encoder readings
			Update_Motor_RPM(&motor1);
			Update_Motor_RPM(&motor2);
			Update_Motor_RPM(&motor3);
			Update_Motor_RPM(&motor4);
		}
		
		// PID control loop update
		PID_UpdateMotor(&motor1);
		PID_UpdateMotor(&motor2);
		PID_UpdateMotor(&motor3);
		PID_UpdateMotor(&motor4);
	}
}

// USART2 RX Idle ISR
void USART2_OnIdle(void)
{
    uint16_t new_pos = RX_BUF_LEN - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
    if (new_pos == rx_old_pos) return;

    if (new_pos > rx_old_pos)
    {
        ProcessBytes(&rx_dma_buf[rx_old_pos], new_pos - rx_old_pos);
    }
    else
    {
        ProcessBytes(&rx_dma_buf[rx_old_pos], RX_BUF_LEN - rx_old_pos);
        if (new_pos > 0) ProcessBytes(&rx_dma_buf[0], new_pos);
    }

    rx_old_pos = new_pos;
}

// Process received bytes from USART2 DMA buffer
static void ProcessBytes(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        char c = (char)data[i];
        if (c == '\r') continue;

        if (c == '\n')
        {
            // finalize line
            uint16_t n = line_len;
            if (n >= LINE_MAX) n = LINE_MAX - 1;

            memcpy(msg_buf, line_buf, n);
            msg_buf[n] = '\0';

            line_len = 0;
            msg_ready = true;   // main loop will handle it
        }
        else
        {
            if (line_len < LINE_MAX - 1)
                line_buf[line_len++] = c;
            else
                line_len = 0;   // overflow -> reset/drop
        }
    }
}

// Helper functions

// Handle USART RX
void Handle_USART_Message(void)
{
	// Example: echo back the received message
	// printf("Received: %s\r\n", msg_buf);
	HAL_UART_Transmit(&huart2, (uint8_t*)"Echo: ", 6, 100);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg_buf, strlen(msg_buf), 100);
	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 100);
}

// Get encoder count from timer
uint16_t Encoder_GetCount(TIM_HandleTypeDef *htim)
{
    return (uint16_t)__HAL_TIM_GET_COUNTER(htim);
}

// SET PWM duty cycle on TIM channels for motor control (DC 0.0 to 1.0)
void SET_PWM_DC(TIM_HandleTypeDef *htim, uint8_t timer_channel, float duty_cycle)
{
    if (duty_cycle < 0.0f) duty_cycle = 0.0f;
    if (duty_cycle > 1.0f) duty_cycle = 1.0f;

    uint32_t period = __HAL_TIM_GET_AUTORELOAD(htim);
    uint32_t ccr = (uint32_t)(duty_cycle * (period + 1));

    __HAL_TIM_SET_COMPARE(htim, timer_channel, ccr);
}

// System Clock Configuration 84 MHz SYSCLOCK with 25 MHz HSE
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
								|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

// Error handler
void Error_Handler(void)
{
	__disable_irq();
	// Fast blink PC13 so you know the clock config (or something else) died
	__HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin   = GPIO_PIN_13;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	while (1)
	{
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		for (volatile uint32_t i = 0; i < 500000; ++i) 
		{
			// crude delay
		}
	}
}