#include "buzzer.h"
#include "pin_config.h"
#include <Arduino.h>

void buzzer_init(void) {
    // Initialize the buzzer pin
    RCC->AHB1ENR |= (1 << 1); // enable GPIOB clock at pos [1]
    BUZZER_PORT->MODER &= ~(3 << (BUZZER_PIN * 2)); // Clear mode bits for BUZZER_PIN
    BUZZER_PORT->MODER |= (1 << (BUZZER_PIN * 2)); // Set BUZZER_PIN as output '01'

    BUZZER_PORT->BSRR = (1 << (BUZZER_PIN + 16)); // Set BUZZER_PIN low (active low)
}

void buzz_on(void){
    // Turn on the buzzer
    BUZZER_PORT->BSRR = (1 << (BUZZER_PIN)); // Set BUZZER_PIN high
}

void buzz_off(void){
    BUZZER_PORT->BSRR = (1 << (BUZZER_PIN + 16)); // Set BUZZER_PIN low
}