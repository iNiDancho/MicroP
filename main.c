#include "stm32f4xx_hal.h"

TIM_HandleTypeDef htim2;

#define KEYPAD_PORT GPIOA

#define LED_PIN GPIO_PIN_13
#define BUZZER_PIN GPIO_PIN_14
#define IO_PORT GPIOC

#define SERVO_PIN GPIO_PIN_10
#define SERVO_PORT GPIOB
#define SEGMENT_PORT GPIOB

uint16_t rows[4] = { GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3 };
uint16_t cols[4] = { GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7 };

uint16_t segmentPins[7] = { GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3,
GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6 };

static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
void SystemClock_Config(void);

char Keypad_Scan(void);
void Servo_Unlock(void);
void Servo_Lock(void);
void LED_On(void);
void LED_Off(void);
void Buzzer_On(void);
void Buzzer_Off(void);
void Blink_LED(void);
void Display_Number(uint8_t num);

const char correctCode[4] = { '1', '2', '3', '4' };
int main(void) {

	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_TIM2_Init();

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

	int index = 0;

	while (1) {

		char key = Keypad_Scan();
		if (key != '\0') {
			Display_Number(key - '0');
			Buzzer_On();
			HAL_Delay(100);
			Buzzer_Off();

			if (key != correctCode[index]) {
				Blink_LED();
				Display_Number(11);
				index = 0;       // Reset index
			} else {
				index++;  // Move to next digit
			}

			if (index == 4) {
				Servo_Unlock();
				LED_On();
				HAL_Delay(500);
				Servo_Lock();
				LED_Off();
				index = 0;
			}
		}
	}
}

const uint8_t segmentMap[11] = { 
		0b00111111, // 0
		0b00000110, // 1
		0b01011011, // 2
		0b01001111, // 3
		0b01100110, // 4
		0b01101101, // 5
		0b01111101, // 6
		0b00000111, // 7
		0b01111111, // 8
		0b01101111,  // 9
		0b00000000  //
		};

void LED_On(void) {
	HAL_GPIO_WritePin(IO_PORT, LED_PIN, GPIO_PIN_SET);
}
void LED_Off(void) {
	HAL_GPIO_WritePin(IO_PORT, LED_PIN, GPIO_PIN_RESET);
}
void Buzzer_On(void) {
	HAL_GPIO_WritePin(IO_PORT, BUZZER_PIN, GPIO_PIN_SET);
}
void Buzzer_Off(void) {
	HAL_GPIO_WritePin(IO_PORT, BUZZER_PIN, GPIO_PIN_RESET);
}

void Blink_LED(void) {
	for (int var = 0; var < 4; ++var) {
		HAL_GPIO_TogglePin(IO_PORT, LED_PIN);
		HAL_Delay(100);
	}
}

char Keypad_Scan(void) {
	const char keymap[4][4] = { 
        { '1', '2', '3', 'A' }, 
        { '4', '5', '6', 'B' },
		{ '7', '8', '9', 'C' }, 
        { '*', '0', '#', 'D' } 
    };

	for (int r = 0; r < 4; r++) {
		HAL_GPIO_WritePin(KEYPAD_PORT, rows[r], GPIO_PIN_RESET);
		for (int c = 0; c < 4; c++) {
			if (HAL_GPIO_ReadPin(KEYPAD_PORT, cols[c]) == GPIO_PIN_RESET) {
				HAL_GPIO_WritePin(KEYPAD_PORT, rows[r], GPIO_PIN_SET);
				while (HAL_GPIO_ReadPin(KEYPAD_PORT, cols[c])
						== GPIO_PIN_RESET)
					;
				return keymap[r][c];
			}
		}
		HAL_GPIO_WritePin(KEYPAD_PORT, rows[r], GPIO_PIN_SET);
	}
	return '\0';
}

void Display_Number(uint8_t num) {
	uint8_t pattern = segmentMap[num];
	for (int i = 0; i < 7; i++) {
		HAL_GPIO_WritePin(SEGMENT_PORT, segmentPins[i],
				(pattern & (1 << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	}
}

void Servo_Unlock() {
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1667);
}

void Servo_Lock() {
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1250);
}

void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 84;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

static void MX_TIM2_Init(void) {

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 83;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 19999;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim2);

}

static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
			GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOB,
			GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_13 | GPIO_PIN_3
					| GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_9,
			GPIO_PIN_RESET);

	GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_13
			| GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

