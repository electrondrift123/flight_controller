#ifndef WTD_H
#define WTD_H

void IWDG_Init(void);

void WDT_setSafe(bool state);
bool WDT_isSafe();


#endif // WTD_H