#include "mcc_generated_files/mcc.h"
#include "timing.h"

void DebugAlert(unsigned int delay_high, unsigned int delay_low, uint8_t num_flashes)
{
#ifdef LED_DEBUG_TRIS
    for (int i = 0; i < num_flashes; i++)
    {
        LED_DEBUG_SetHigh();
        Sleep(delay_high);
        LED_DEBUG_SetLow();
        Sleep(delay_low);
    }
#endif // LED_DEBUG_TRIS
}
