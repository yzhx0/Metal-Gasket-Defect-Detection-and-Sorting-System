#include "watchdog.h"
#include "main.h"

/*
 * LSI frequency is nominally 40 kHz (30..60 kHz across parts).
 *   40 kHz / 256 = 156.25 Hz
 *   156 ticks  -> ~998 ms timeout.
 * A slower part stretches the timeout up to ~1.3 s, which is still well
 * below the 5 s fail-safe rejection horizon of the line controller.
 */
#define IWDG_KEY_UNLOCK    0x5555U
#define IWDG_KEY_REFRESH   0xAAAAU
#define IWDG_KEY_START     0xCCCCU
#define IWDG_PR_DIV256     0x06U
#define IWDG_RELOAD_VALUE  156U
#define IWDG_FLAG_TIMEOUT_MS 100U

void Watchdog_Init(void)
{
    uint32_t deadline;

    /* LSI must be running before the watchdog can be started. */
    __HAL_RCC_LSI_ENABLE();
    deadline = HAL_GetTick() + IWDG_FLAG_TIMEOUT_MS;
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY) == RESET)
    {
        if (HAL_GetTick() > deadline)
        {
            Error_Handler();
        }
    }

    /* Unlock the IWDG registers and wait for any pending update to finish. */
    IWDG->KR = IWDG_KEY_UNLOCK;
    deadline = HAL_GetTick() + IWDG_FLAG_TIMEOUT_MS;
    while ((IWDG->SR & IWDG_SR_PVU) != 0U)
    {
        if (HAL_GetTick() > deadline)
        {
            Error_Handler();
        }
    }
    IWDG->PR = IWDG_PR_DIV256;

    deadline = HAL_GetTick() + IWDG_FLAG_TIMEOUT_MS;
    while ((IWDG->SR & IWDG_SR_RVU) != 0U)
    {
        if (HAL_GetTick() > deadline)
        {
            Error_Handler();
        }
    }
    IWDG->RLR = IWDG_RELOAD_VALUE;

    /* Start the watchdog and issue the first refresh.  From this point on
       the Monitor task must service it at least once per timeout period. */
    IWDG->KR = IWDG_KEY_START;
    IWDG->KR = IWDG_KEY_REFRESH;
}

void Watchdog_Refresh(void)
{
    IWDG->KR = IWDG_KEY_REFRESH;
}
