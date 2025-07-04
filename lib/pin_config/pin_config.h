#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// ===== BUZZER (PC13) =====
#define BUZZER_PORT       GPIOC
#define BUZZER_PIN        13

// ===== MOTORS (PA0–PA3) =====
#define MOTOR_PWM_PORT    GPIOA
#define MOTOR1_PWM_PIN    0  // TIM2_CH1
#define MOTOR2_PWM_PIN    1  // TIM2_CH2
#define MOTOR3_PWM_PIN    2  // TIM2_CH3
#define MOTOR4_PWM_PIN    3  // TIM2_CH4

// ===== GPIO Modes (optional) =====
#define GPIO_MODE_INPUT   0x0
#define GPIO_MODE_OUTPUT  0x1
#define GPIO_MODE_ALT     0x2
#define GPIO_MODE_ANALOG  0x3

// ===== Function Prototypes =====
void Buzzer_Init(void);
void MotorPWM_TIM2_Init(void);
void used_gpio_init(void);

#endif // PIN_CONFIG_H
