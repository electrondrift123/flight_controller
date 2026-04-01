#include "pin_config.h"
#include <Arduino.h>

// uses active buzzer on PC13
void Blink_Init(void) {
  RCC->AHB1ENR |= (1 << 2); // Enable GPIOC clock
  GPIOC->MODER &= ~(3 << (BUILTIN_LED_PIN * 2)); // Clear the setup bits (reset state)
  GPIOC->MODER |= (1 << (BUILTIN_LED_PIN * 2)); // Set it as output [01] at [26:25]
  GPIOC->BSRR = (1 << BUILTIN_LED_PIN ); // Set LED OFF (active low)
}

void Buzzer_Init(void){
  RCC->AHB1ENR |= (1 << 1); // enable GPIOB clock at pos[1]
  BUZZER_PORT->MODER &= ~(3 << (BUZZER_PIN * 2)); // clear the setup bits (reset state)
  BUZZER_PORT->MODER |= (1 << (BUZZER_PIN * 2)); // set it as output [01] at [14:13]

  BUZZER_PORT->BSRR = (1 << (BUZZER_PIN + 16)); // HIGH = OFF LED
}

void Vbat_init(void){
  // 1. Enable GPIOB clock 
  RCC->AHB1ENR |= (1 << 1); // Enable GPIOB clock
  
  // 2. Configure PB0 as analog mode (NOT just input!)
  VBAT_PORT->MODER &= ~(3 << (VBAT_PIN * 2));
  VBAT_PORT->MODER |= (3 << (VBAT_PIN * 2));  // [11] = Analog mode!
  
  // 3. Enable ADC1 clock (CRITICAL!)
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  
  // 4. Configure ADC1
  ADC1->CR2 = 0;  // Reset
  ADC1->SQR3 = 8; // Channel 8 = PB0 (ADC1_IN8)
  
  // 5. Optional: Set sample time
  ADC1->SMPR2 |= (7 << 24); // Channel 8 sample time (bits 24-26)
  
  // 6. Enable ADC
  ADC1->CR2 |= ADC_CR2_ADON; // Enable ADC
  delayMicroseconds(10);      // Wait for stabilization
}

uint16_t readVbat(void) {
  // Start conversion
  ADC1->CR2 |= ADC_CR2_SWSTART;
  
  // Wait for completion
  while(!(ADC1->SR & ADC_SR_EOC));
  
  // Read 12-bit result (0-4095)
  return (uint16_t)ADC1->DR;
}

void MotorPWM_TIM2_Init(void) {
  // Enable Clocks 
  RCC->AHB1ENR |= (1 << 0);     // Enable GPIOA clock
  RCC->APB1ENR |= (1 << 0);     // Enable TIM2 clock

  // Reset TIM2 registers for clean configuration
  TIM2->CR1 = 0;
  TIM2->CR2 = 0;
  TIM2->SMCR = 0;
  TIM2->DIER = 0;
  TIM2->SR = 0;
  TIM2->EGR = 0;
  TIM2->CCMR1 = 0;
  TIM2->CCMR2 = 0;
  TIM2->CCER = 0;
  TIM2->CNT = 0;
  TIM2->PSC = 0;
  TIM2->ARR = 0;

  // Set MODER[0–3] = 10 (AF)
  GPIOA->MODER &= ~((3 << (0 * 2)) | (3 << (1 * 2)) | (3 << (2 * 2)) | (3 << (3 * 2)));
  GPIOA->MODER |=  ((2 << (0 * 2)) | (2 << (1 * 2)) | (2 << (2 * 2)) | (2 << (3 * 2)));

  // Set AFRL[0–3] = AF1 (TIM2)
  GPIOA->AFR[0] &= ~((0xF << (0 * 4)) | (0xF << (1 * 4)) | (0xF << (2 * 4)) | (0xF << (3 * 4)));
  GPIOA->AFR[0] |=  ((1 << (0 * 4)) | (1 << (1 * 4)) | (1 << (2 * 4)) | (1 << (3 * 4)));

  // Configure TIM2 for PWM
  TIM2->PSC = 83;          // Prescaler: 84MHz / (83 + 1) = 1 MHz
  TIM2->ARR = 19999;       // ARR: 1 MHz / 50 Hz = 20000 → 20ms period

  /*** Configure PWM Mode on All 4 Channels ***/
  // CH1 (PA0) - TIM2_CH1
  TIM2->CCMR1 &= ~(0xFF << 0);  // Clear CCMR1 bits for CH1 and CH2
  TIM2->CCMR1 |= (6 << 4);       // PWM Mode 1 for CH1
  TIM2->CCMR1 |= TIM_CCMR1_OC1PE;  // Enable preload for CH1
  TIM2->CCER &= ~TIM_CCER_CC1NP;   // Clear polarity (active high)
  TIM2->CCER &= ~TIM_CCER_CC1E;    // Disable CH1 first
  TIM2->CCER |= TIM_CCER_CC1E;     // Enable CH1 output

  // CH2 (PA1) - TIM2_CH2
  TIM2->CCMR1 |= (6 << 12);       // PWM Mode 1 for CH2
  TIM2->CCMR1 |= TIM_CCMR1_OC2PE; // Enable preload for CH2
  TIM2->CCER &= ~TIM_CCER_CC2NP;   // Clear polarity (active high)
  TIM2->CCER &= ~TIM_CCER_CC2E;    // Disable CH2 first
  TIM2->CCER |= TIM_CCER_CC2E;     // Enable CH2 output

  // CH3 (PA2) - TIM2_CH3
  TIM2->CCMR2 &= ~(0xFF << 0);  // Clear CCMR2 bits for CH3
  TIM2->CCMR2 |= (6 << 4);       // PWM Mode 1 for CH3
  TIM2->CCMR2 |= TIM_CCMR2_OC3PE; // Enable preload for CH3
  TIM2->CCER &= ~TIM_CCER_CC3NP;   // Clear polarity (active high)
  TIM2->CCER &= ~TIM_CCER_CC3E;    // Disable CH3 first
  TIM2->CCER |= TIM_CCER_CC3E;     // Enable CH3 output

  // CH4 (PA3) - TIM2_CH4
  TIM2->CCMR2 |= (6 << 12);       // PWM Mode 1 for CH4
  TIM2->CCMR2 |= TIM_CCMR2_OC4PE; // Enable preload for CH4
  TIM2->CCER &= ~TIM_CCER_CC4NP;   // Clear polarity (active high)
  TIM2->CCER &= ~TIM_CCER_CC4E;    // Disable CH4 first
  TIM2->CCER |= TIM_CCER_CC4E;     // Enable CH4 output

  // Enable timer with auto-reload preload
  TIM2->CR1 = TIM_CR1_ARPE;   // Auto-reload preload enable
  TIM2->EGR = TIM_EGR_UG;     // Update generation (load all registers)
  TIM2->CR1 |= TIM_CR1_CEN;   // Enable counter

  // 1 ms min throttle : 2 ms max throttle
  TIM2->CCR1 = 1020;
  TIM2->CCR2 = 1020;
  TIM2->CCR3 = 1020;
  TIM2->CCR4 = 1020;
}

// void MotorPWM_TIM2_Init(void) {
//   // Enable Clocks 
//   RCC->AHB1ENR |= (1 << 0);     // Enable GPIOA clock
//   RCC->APB1ENR |= (1 << 0);     // Enable TIM2 clock

//   // Reset TIM2 registers for clean configuration
//   TIM2->CR1 = 0;
//   TIM2->CR2 = 0;
//   TIM2->SMCR = 0;
//   TIM2->DIER = 0;
//   TIM2->SR = 0;
//   TIM2->EGR = 0;
//   TIM2->CCMR1 = 0;
//   TIM2->CCMR2 = 0;
//   TIM2->CCER = 0;
//   TIM2->CNT = 0;
//   TIM2->PSC = 0;
//   TIM2->ARR = 0;

//   // Set MODER[0–3] = 10 (AF)
//   GPIOA->MODER &= ~((3 << (0 * 2)) | (3 << (1 * 2)) | (3 << (2 * 2)) | (3 << (3 * 2)));
//   GPIOA->MODER |=  ((2 << (0 * 2)) | (2 << (1 * 2)) | (2 << (2 * 2)) | (2 << (3 * 2)));

//   // Set AFRL[0–3] = AF1 (TIM2)
//   GPIOA->AFR[0] &= ~((0xF << (0 * 4)) | (0xF << (1 * 4)) | (0xF << (2 * 4)) | (0xF << (3 * 4)));
//   GPIOA->AFR[0] |=  ((1 << (0 * 4)) | (1 << (1 * 4)) | (1 << (2 * 4)) | (1 << (3 * 4)));

//   // Configure TIM2 for PWM
//   TIM2->PSC = 83;          // Prescaler: 84MHz / (83 + 1) = 1 MHz
//   TIM2->ARR = 19999;       // ARR: 1 MHz / 50 Hz = 20000 → 20ms period

//   /*** Configure PWM Mode on All 4 Channels ***/
//   // CH1 (PA0) - TIM2_CH1
//   TIM2->CCMR1 |= (6 << 4);       // PWM Mode 1 for CH1
//   TIM2->CCMR1 |= TIM_CCMR1_OC1PE;  // Enable preload for CH1

//   // CH2 (PA1) - TIM2_CH2
//   TIM2->CCMR1 |= (6 << 12);       // PWM Mode 1 for CH2
//   TIM2->CCMR1 |= TIM_CCMR1_OC2PE; // Enable preload for CH2

//   // CH3 (PA2) - TIM2_CH3
//   TIM2->CCMR2 |= (6 << 4);       // PWM Mode 1 for CH3
//   TIM2->CCMR2 |= TIM_CCMR2_OC3PE; // Enable preload for CH3

//   // CH4 (PA3) - TIM2_CH4
//   TIM2->CCMR2 |= (6 << 12);       // PWM Mode 1 for CH4
//   TIM2->CCMR2 |= TIM_CCMR2_OC4PE; // Enable preload for CH4

//   // Configure CCER - ALL CHANNELS AT ONCE
//   TIM2->CCER = 0;  // Start fresh
//   TIM2->CCER |= TIM_CCER_CC1E;  // Enable CH1 (active high)
//   TIM2->CCER |= TIM_CCER_CC2E;  // Enable CH2
//   TIM2->CCER |= TIM_CCER_CC3E;  // Enable CH3
//   TIM2->CCER |= TIM_CCER_CC4E;  // Enable CH4
//   // (CC1P, etc default to 0 = active high)

//   // Enable timer with auto-reload preload
//   TIM2->CR1 = TIM_CR1_ARPE;   // Auto-reload preload enable
//   TIM2->EGR = TIM_EGR_UG;     // Update generation (load all registers)
//   TIM2->CR1 |= TIM_CR1_CEN;   // Enable counter

//   // 1 ms min throttle : 2 ms max throttle
//   TIM2->CCR1 = 1000;
//   TIM2->CCR2 = 1000;
//   TIM2->CCR3 = 1000;
//   TIM2->CCR4 = 1000;
// }


void used_gpio_init(void){
    Blink_Init(); // Initialize the built-in LED on PC13
    Buzzer_Init(); // Initialize buzzer on PC13
    Vbat_init(); // Initialize VBAT pin on PB0
    MotorPWM_TIM2_Init(); // Initialize TIM2 for motor control PWM on PA0-
}