extern unsigned long ticks;

void TimerISR(void);

#define millis() ticks

void InitTiming(void);

void Sleep(uint32_t ms);

#define __global_timeout __g_timeout_start

#ifdef __global_timeout

extern unsigned long __global_timeout;

#define await_timeout(condition, wait_time, on_timeout)                             \
        __global_timeout = millis();                                                \
        while (!(condition))                                                        \
        {                                                                           \
            if (millis() - __global_timeout > wait_time)                            \
            {                                                                       \
                on_timeout();                                                       \
                break;                                                              \
            }                                                                       \
        }
#else
#warning "__global_timeout has not been defined, so await_timeout() will not work!"
#define await_timeout(dummya, dummyb, dummyc)
#endif
