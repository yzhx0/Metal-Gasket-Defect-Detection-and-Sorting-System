#ifndef WATCHDOG_H
#define WATCHDOG_H

/**
 * @brief  Independent watchdog (IWDG) driver, register level.
 *
 * The STM32F103 watchdog runs from the LSI oscillator (~40 kHz).  With the
 * /256 prescaler and a reload value of 156 the timeout is roughly one
 * second.  The watchdog is started before the RTOS scheduler runs and is
 * serviced exclusively by the Monitor task, which only refreshes it while
 * every supervised task is provably alive.
 */

#ifdef __cplusplus
extern "C" {
#endif

void Watchdog_Init(void);
void Watchdog_Refresh(void);

#ifdef __cplusplus
}
#endif

#endif /* WATCHDOG_H */
