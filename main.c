#include "stm32f4xx.h"

void LED_On(void)
{
	GPIOC->BSRR = (1 << 13); // PC13 ON
}

void LED_Off(void)
{
	GPIOC->BSRR = (1 << (13 + 16)); // PC13 OFF
}

void Buzzer_On(void)
{
	GPIOC->BSRR = (1 << 14); // PC14 ON
}

void Buzzer_Off(void)
{
	GPIOC->BSRR = (1 << (14 + 16)); // PC14 OFF
}

// ==== Simple delay (approximate, for 84MHz clock) ====
void delay_ms(uint32_t ms)
{
	for (uint32_t i = 0; i < ms * 84; i++)
	{
		__NOP();
	}
}

#define KEYPAD_ROW_PORT GPIOA
#define KEYPAD_COL_PORT GPIOA

// Rows = Output (PA0 - PA3), Cols = Input with Pull-Up (PA4 - PA7)
uint16_t rows[4] = {(1 << 0), (1 << 1), (1 << 2), (1 << 3)};
uint16_t cols[4] = {(1 << 4), (1 << 5), (1 << 6), (1 << 7)};

// Key mapping
const char keymap[4][4] = {
	{'1', '2', '3', 'A'},
	{'4', '5', '6', 'B'},
	{'7', '8', '9', 'C'},
	{'*', '0', '#', 'D'}};

void Keypad_GPIO_Init(void)
{
	// Enable GPIOA clock

	// Configure PA0 - PA3 as output (rows)
	for (int i = 0; i < 4; i++)
	{
		int pin = i;
		KEYPAD_ROW_PORT->MODER &= ~(3 << (pin * 2));  // Clear
		KEYPAD_ROW_PORT->MODER |= (1 << (pin * 2));	  // Output
		KEYPAD_ROW_PORT->OTYPER &= ~(1 << pin);		  // Push-pull
		KEYPAD_ROW_PORT->OSPEEDR |= (3 << (pin * 2)); // High speed
		KEYPAD_ROW_PORT->PUPDR &= ~(3 << (pin * 2));  // No pull
	}

	// Configure PA4 - PA7 as input with pull-up (columns)
	for (int i = 0; i < 4; i++)
	{
		int pin = i + 4;
		KEYPAD_COL_PORT->MODER &= ~(3 << (pin * 2)); // Input mode
		KEYPAD_COL_PORT->PUPDR &= ~(3 << (pin * 2)); // Clear
		KEYPAD_COL_PORT->PUPDR |= (1 << (pin * 2));	 // Pull-up
	}
}

char Keypad_Scan(void)
{
	for (int r = 0; r < 4; r++)
	{
		// Drive current row LOW
		KEYPAD_ROW_PORT->BSRR = (rows[r] << 16); // Reset

		// Check each column
		for (int c = 0; c < 4; c++)
		{
			if ((KEYPAD_COL_PORT->IDR & cols[c]) == 0)
			{
				// Debounce (optional small delay)
				for (volatile int d = 0; d < 20000; d++)
					;

				// Wait for release
				while ((KEYPAD_COL_PORT->IDR & cols[c]) == 0)
					;

				// Restore row HIGH
				KEYPAD_ROW_PORT->BSRR = rows[r];

				return keymap[r][c];
			}
		}

		// Restore row HIGH
		KEYPAD_ROW_PORT->BSRR = rows[r];
	}

	return '\0'; // No key pressed
}

#define SEGMENT_PORT GPIOB
uint16_t segmentPins[7] = {
	(1 << 0), // PB0
	(1 << 1), // PB1
	(1 << 2), // PB2
	(1 << 3), // PB3
	(1 << 4), // PB4
	(1 << 5), // PB5
	(1 << 6)  // PB6
};

// Segment bit map for 0-9 and blank
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
	0b01101111, // 9
	0b01000000	// blank (just middle bar)
};

// === GPIOB Initialization ===
void Segment_GPIO_Init(void)
{
	// Enable GPIOB clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

	// Set PB0 - PB6 as General Purpose Output
	for (int i = 0; i < 7; i++)
	{
		uint32_t pin = i;
		SEGMENT_PORT->MODER &= ~(3 << (pin * 2));  // Clear
		SEGMENT_PORT->MODER |= (1 << (pin * 2));   // Output mode
		SEGMENT_PORT->OTYPER &= ~(1 << pin);	   // Push-pull
		SEGMENT_PORT->OSPEEDR |= (3 << (pin * 2)); // High speed
		SEGMENT_PORT->PUPDR &= ~(3 << (pin * 2));  // No pull-up/down
	}
}

// === Display number on 7-segment ===
void Display_Number(uint8_t num)
{
	if (num > 10)
		return; // Out of range
	uint8_t pattern = segmentMap[num];
	for (int i = 0; i < 7; i++)
	{
		if (pattern & (1 << i))
		{
			SEGMENT_PORT->BSRR = segmentPins[i]; // Set pin
		}
		else
		{
			SEGMENT_PORT->BSRR = (segmentPins[i] << 16); // Reset pin
		}
	}
}

// ==== GPIO Configuration ====
void GPIO_Config(void)
{
	// Enable GPIOC, GPIOA clocks
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	// PC13 and PC14 as output
	GPIOC->MODER &= ~((3 << (13 * 2)) | (3 << (14 * 2))); // Clear mode bits
	GPIOC->MODER |= (1 << (13 * 2)) | (1 << (14 * 2));	  // Output mode

	GPIOC->OTYPER &= ~((1 << 13) | (1 << 14));			  // Push-pull
	GPIOC->OSPEEDR |= (3 << (13 * 2)) | (3 << (14 * 2));  // High speed
	GPIOC->PUPDR &= ~((3 << (13 * 2)) | (3 << (14 * 2))); // No pull-up/pull-down
	
    GPIOA->MODER |= (2 << (15 * 2));              // Alternate function mode for PA5
    GPIOA->AFR[1] |= (1 << ((15 - 8) * 4));        // AF1 (TIM2_CH1) for PA15
}



void TIM2_PWM_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;          // Enable TIM2 clock

   TIM2->PSC = 84;     // Assuming 84 MHz ? gives 100 kHz
		TIM2->ARR = 19999;   // 100 kHz / 20000 = 50 Hz


    TIM2->CCR1 = 0;        // Default pulse width = 1.5 ms (center)

    // PWM mode 1 on CH1: active while CNT < CCR1
    TIM2->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM2->CCMR1 |=  (6 << TIM_CCMR1_OC1M_Pos);  // PWM mode 1
    TIM2->CCMR1 |=  TIM_CCMR1_OC1PE;            // Preload enable

    TIM2->CCER |= TIM_CCER_CC1E;        // Enable output on CH1
    TIM2->CR1 |= TIM_CR1_ARPE;          // Enable auto-reload preload

    TIM2->EGR |= TIM_EGR_UG;            // Generate update event
    TIM2->CR1 |= TIM_CR1_CEN;           // Enable timer
}


void GPIOA_Init(void) {

    GPIOA->MODER &= ~(3 << (15 * 2));             // Clear mode bits for PA15
    GPIOA->MODER |= (2 << (15 * 2));              // Alternate function mode for PA15

    GPIOA->AFR[1] &= ~(0xF << ((15 - 8) * 4));     // Clear AF bits
    GPIOA->AFR[1] |= (1 << ((15 - 8) * 4));        // Set AF1 for TIM2_CH1
}

void delay_timer2(uint32_t us) {
    uint32_t start = TIM2->CNT;
    while ((TIM2->CNT - start) < us);  // Wait for specified microseconds
}

void Servo_Set_Angle(uint16_t pulse) {
    TIM2->CCR1 = pulse;                          // Set duty cycle (1000 - 2000)
}

// ==== Main ====
const char correctCode[4] = { '1', '2', '3', '4' };

int main(void)
{
	GPIO_Config();
	GPIOA_Init();
	TIM2_PWM_Init();
	Segment_GPIO_Init();
	Keypad_GPIO_Init();
	int index = 0;
	while (1)
	{
		
		char key = Keypad_Scan();
		if (key != '\0') {
			Display_Number(key - '0');
			Buzzer_On();
			delay_ms(100);
			Buzzer_Off();

			if (key != correctCode[index]) {
				LED_On();
				Display_Number(11);
				index = 0;       // Reset index
			}
			else{
				index++;  // Move to next digit
			}

			if (index == 4) {
				TIM2->CCR1 = 2510;        // Default pulse width = 1.5 ms (center)//193-383
				LED_On();
				Buzzer_On();

				delay_ms(5000);
				TIM2->CCR1 = 1000;        // Default pulse width = 1.5 ms (center)//193-383
				LED_Off();
				Buzzer_Off();
				index = 0;  // Reset for next attempt
			}

		}

	}
}
