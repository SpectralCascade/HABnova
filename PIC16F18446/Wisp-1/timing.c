#include "mcc_generated_files/mcc.h"
#include "timing.h"

uint32_t ticks = 0;

void TimerISR()
{
    ticks++;
}

void InitTiming()
{
    INTERRUPT_GlobalInterruptEnable();
    TMR0_SetInterruptHandler(TimerISR);
}

void Sleep(uint32_t ms)
{
    uint32_t start = millis();
    while (millis() - start < ms)
    {
        __delay_ms(1);
    }
}

#ifdef __global_timeout
uint32_t __global_timeout;
#endif // __global_timeout
