#include "main.h"
#include <string.h>
// #include <stdbool.h>
#include <stdlib.h>
// #include <stdint.h>

void SystemClock_Config(void);
uint8_t rx_dma_buf[RX_BUF_LEN]; // DMA RX buffer
static volatile uint16_t rx_old_pos = 0; // Old position in RX DMA buffer

static char line_buf[LINE_MAX];
static volatile uint16_t line_len = 0;

static volatile bool msg_ready = false;
static char msg_buf[LINE_MAX];

static uint32_t last_tel_ms = 0;
const uint32_t TEL_PERIOD_MS = 50;	// 20 Hz telemetry report rate
static uint32_t last_rx_ms = 0;

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
		// Handle received USART messages
		uint32_t now = HAL_GetTick();
		if (msg_ready)
		{
			last_rx_ms = now;
			msg_ready = false;
			Handle_USART_Message();
		}
		else if (now - last_rx_ms >= LAST_RX_TIMEOUT_MS)
		{
			// No messages received for LAST_RX_TIMEOUT_MS -> stop motors
			Set_Left_RPM(0);
			Set_Right_RPM(0);
			// Disable_Motors();
		}

		// Report telemetry over serial every TEL_PERIOD_MS
		if (now - last_tel_ms >= TEL_PERIOD_MS)
		{
			last_tel_ms = now;
			Report_Telemetry_USART();
		}
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
			if (!msg_ready)
			{
				// finalize line
				uint16_t n = line_len;
				if (n >= LINE_MAX) n = LINE_MAX - 1;

				memcpy(msg_buf, line_buf, n);
				msg_buf[n] = '\0';

				line_len = 0;
				msg_ready = true;   // main loop will handle it
			}
			// else drop line – previous message not yet processed
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

static inline bool streq(const char *a, const char *b) { return strcmp(a,b) == 0; }

// Handle USART RX
void Handle_USART_Message(void)
{
	/* List of commands:
		SET,<LEFT_RPM>,<RIGHT_RPM>		- Set target RPMs for motors
		EN,<1|0>                     	- Enable or disable motors
		BRK,<1|0>                    	- Enable or disable motor braking
		ESTOP					   		- Emergency stop (disable motors, latches until controller restart)
		PING							- Check is alive
		PID,<P>,<I>,<D>            		- Set PID parameters for all motors
	*/
	if (ESTOP) return; // ignore all commands if ESTOP is set
	
	// Create null-terminated copy of msg_buf
	char line[LINE_MAX];
	strncpy(line, msg_buf, sizeof(line)-1);
	line[sizeof(line)-1] = '\0';

	// tokenise
	char *cmd = strtok(line, ",");
	char *a1 = strtok(NULL, ",");
	char *a2 = strtok(NULL, ",");
	// char *a3 = strtok(NULL, ",");
	
	if (!cmd) return;

	// SET,<LEFT_RPM>, <RIGHT_RPM>
	if (streq(cmd, "SET"))
	{
		int32_t l, r;
		if (parse_int32(a1, &l) && parse_int32(a2, &r))
		{
			Set_Left_RPM(l);
			Set_Right_RPM(r);
			uart_print("OK,SET\r\n");
		}
		else uart_print("ERR,BADARGS\r\n");
	}

	// EN,<1|0>
	else if (streq(cmd, "EN"))
	{
		int32_t en;
		if (parse_int32(a1, &en))
		{
			if (en) Enable_Motors();
			else Disable_Motors();
			uart_print("OK,EN\r\n");
		}
		else uart_print("ERR,BADARGS\r\n");
	}

	// BRK,<1|0>
	// else if (streq(cmd, "BRK"))
	// {
	// 	int32_t brk;
	// 	if (parse_int32(a1, &brk))
	// 	{
	// 		if (brk) Enable_Motor_Braking();
	// 		else Disable_Motor_Braking();
	// 		uart_print("OK\r\n");
	// 	}
	// 	else uart_print("ERR,BADARGS\r\n");
	// }

	// ESTOP
	else if (streq(cmd, "ESTOP"))
	{
		SET_ESTOP();
		uart_print("OK,ESTOP\r\n");
	}

	// PING
	else if (streq(cmd, "PING"))
	{
		if (a1)
		{
			char response[LINE_MAX];
			snprintf(response, sizeof(response), "PONG,%s\r\n", a1);
			uart_print(response);
		}
		else uart_print("PONG\r\n");
	}

	// PID
	// else if (streq(cmd, "PID"))
	// {
	// 	float p, i, d;
	// 	if (a1 && a2 && a3)
	// 	{
	// 		// not implemented yet
	// 	}
	// }
}

bool parse_int32(const char *str, int32_t *out_value)
{
	if (!str) return false;

	char *endptr = NULL;
	long val = strtol(str, &endptr, 10);
	if (endptr == str || *endptr != '\0') return false; // not clean integer
	*out_value = (int32_t)val;
	return true;
}

// Report status over USART
void Report_Telemetry_USART(void)
{
	// Telemetry format:
	// TEL,<motors_enabled>,<ESTOP>,<motor1_rpm>,<motor2_rpm>,<motor3_rpm>,<motor4_rpm>\r\n

	char buf[160];
	int n = snprintf(buf, sizeof(buf),
		"TEL,%u,%u,%d,%d,%d,%d\r\n",
		Motors_Enabled, ESTOP,
		(int)motor1.rpm, (int)motor2.rpm, (int)motor3.rpm, (int)motor4.rpm
	);

	if (n > 0)
	{
		// HAL_UART_Transmit(&huart2, (uint8_t*)buf, (uint16_t)n, 20);
		uart_print(buf);
	}
}

void uart_print(const char *str)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)str, (uint16_t)strlen(str), 50);
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