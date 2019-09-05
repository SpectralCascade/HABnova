#include "mcc_generated_files/mcc.h"
#include "timing.h"

unsigned long ticks = 0;

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
    unsigned long start = millis();
    while (millis() - start < ms)
    {
        __delay_ms(1);
    }
}

#ifdef __global_timeout
unsigned long __global_timeout;
#endif // __global_timeout
