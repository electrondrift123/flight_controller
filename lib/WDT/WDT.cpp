#include "WDT.h"
#include "stm32f4xx.h"
#include "sync.h"
#include "shared_data.h"

void IWDG_Init(void) {
    // Enable write access to IWDG registers
    IWDG->KR = 0x5555;

    // Set prescaler (divides LSI clock)
    IWDG->PR = IWDG_PR_PR_2;  // Prescaler 64

    // Set reload value (max 0x0FFF) → how long until reset
    // Timeout = (reload + 1) * prescaler / LSI
    // Ex: ~1s = (1000 * 64) / 32000
    IWDG->RLR = 199;

    // Reload the counter
    IWDG->KR = 0xAAAA;

    // Start the watchdog
    IWDG->KR = 0xCCCC;
}

void WDT_setSafe(bool state) {
    taskENTER_CRITICAL();
    SAFE_WDT = state;
    taskEXIT_CRITICAL();
}

bool WDT_isSafe() {
    bool value;
    taskENTER_CRITICAL();
    value = SAFE_WDT;
    taskEXIT_CRITICAL();
    return value;
}
