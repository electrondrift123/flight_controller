#include "pin_config.h"
#include <Arduino.h>

// uses active buzzer on PC13
void Buzzer_Init(void){
  RCC->AHB1ENR |= (1 << 2); // enable GPIOC clock at pos[2]
  GPIOC->MODER &= ~(3 << (BUZZER_PIN * 2)); // clear the setup bits (reset state)
  GPIOC->MODER |= (1 << (BUZZER_PIN * 2)); // set it as output [01] at [14:13]

  GPIOC->BSRR = (1 << (BUZZER_PIN + 16)); // HIGH = OFF LED
}

void MotorPWM_TIM2_Init(void) {
  /*** 1. Enable Clocks ***/
  RCC->AHB1ENR |= (1 << 0);     // Enable GPIOA clock
  RCC->APB1ENR |= (1 << 0);     // Enable TIM2 clock

  /*** 2. Configure GPIOA Pins PA0–PA3 to Alternate Function Mode ***/
  // Set MODER[0–3] = 10 (AF)
  GPIOA->MODER &= ~((3 << (0 * 2)) | (3 << (1 * 2)) | (3 << (2 * 2)) | (3 << (3 * 2)));
  GPIOA->MODER |=  ((2 << (0 * 2)) | (2 << (1 * 2)) | (2 << (2 * 2)) | (2 << (3 * 2)));

  // Set AFRL[0–3] = AF1 (TIM2)
  GPIOA->AFR[0] &= ~((0xF << (0 * 4)) | (0xF << (1 * 4)) | (0xF << (2 * 4)) | (0xF << (3 * 4)));
  GPIOA->AFR[0] |=  ((1 << (0 * 4)) | (1 << (1 * 4)) | (1 << (2 * 4)) | (1 << (3 * 4)));

  /*** 3. Configure TIM2 for PWM ***/
  TIM2->PSC = 83;          // Prescaler: 84MHz / (83 + 1) = 1 MHz
  TIM2->ARR = 19999;       // ARR: 1 MHz / 50 Hz = 20000 → 20ms period

  /*** 4. Configure PWM Mode on All 4 Channels ***/
  // CH1 (PA0)
  TIM2->CCMR1 |= (6 << 4);             // PWM Mode 1
  TIM2->CCMR1 |= TIM_CCMR1_OC1PE;      // Enable preload
  TIM2->CCER  |= TIM_CCER_CC1E;        // Enable output

  // CH2 (PA1)
  TIM2->CCMR1 |= (6 << 12);
  TIM2->CCMR1 |= TIM_CCMR1_OC2PE;
  TIM2->CCER  |= TIM_CCER_CC2E;

  // CH3 (PA2)
  TIM2->CCMR2 |= (6 << 4);
  TIM2->CCMR2 |= TIM_CCMR2_OC3PE;
  TIM2->CCER  |= TIM_CCER_CC3E;

  // CH4 (PA3)
  TIM2->CCMR2 |= (6 << 12);
  TIM2->CCMR2 |= TIM_CCMR2_OC4PE;
  TIM2->CCER  |= TIM_CCER_CC4E;

  // enable timer
  TIM2->CR1 |= TIM_CR1_ARPE;   // Auto-reload preload
  TIM2->EGR  |= TIM_EGR_UG;    // Update generation (load registers)
  TIM2->CR1 |= TIM_CR1_CEN;    // Enable timer

  // 1 ms min throttle : 2 ms max throttle
  TIM2->CCR1 = 1000;
  TIM2->CCR2 = 1000;
  TIM2->CCR3 = 1000;
  TIM2->CCR4 = 1000;
}

void used_gpio_init(void){
    Buzzer_Init(); // Initialize buzzer on PC13
    MotorPWM_TIM2_Init(); // Initialize TIM2 for motor control PWM on PA0-
}