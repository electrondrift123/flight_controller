#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// ==== i2c default pins =====
#define I2C_SCL1_PIN       6   // PB6
#define I2C_SDA1_PIN       7   // PB7

// ==== SPI default pins =====
#define SPI_SCK_PIN       5   // PA5
#define SPI_MISO_PIN      6   // PA6
#define SPI_MOSI_PIN      7   // PA7

// ==== SX1278 pins =====
// #define LORA_NSS_PIN       
// #define LORA_DIO0_PIN       
// #define LORA_RST_PIN       

// ===== nRF24 pins =====
#define NRF_CE_PIN    PB13
#define NRF_CSN_PIN   PA4
#define NRF_IRQ_PIN   PB1  

// ===== LED PIN ======
#define BUILTIN_LED_PORT GPIOC
#define BUILTIN_LED_PIN  13

// ===== BUZZER (PC13) =====
#define BUZZER_PORT       GPIOB
#define BUZZER_PIN        12

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
