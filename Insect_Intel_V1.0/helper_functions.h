#ifndef POWER_HELPERS_H
#define POWER_HELPERS_H

#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

/* ─────────────────────────────────────────────────────────────────────────────
 * Power Profiles
 *
 *  MINIMUM   – Only RTC alive. Used during IDLE and CHARGING sleep windows.
 *              Wake sources: RTC minute tick, Hall GPIO interrupt.
 *
 *  MEASURE   – RTC + I2C_0. Used for periodic safety / charging reads inside
 *              IDLE and CHARGING states before returning to sleep.
 *
 *  ACTIVE    – Everything up. Used for the entire POWER_STM state.
 * ───────────────────────────────────────────────────────────────────────────*/

/* ── Clock gating ───────────────────────────────────────────────────────────*/
void PWR_BlockFastClocks(void);
void PWR_UnblockFastClocks(void);

/* ── Individual peripheral power gates ─────────────────────────────────────*/
void PWR_EnableI2C0(void);
void PWR_DisableI2C0(void);

void PWR_EnableI2C1(void);
void PWR_DisableI2C1(void);

void PWR_EnableUART0(void);
void PWR_DisableUART0(void);

void PWR_EnableSPI1(void);
void PWR_DisableSPI1(void);

/* ── Compound profile helpers ───────────────────────────────────────────────*/

/**
 * @brief Enter MINIMUM profile.
 *        Disables I2C_0, I2C_1, UART_0, SPI_1, DMA.
 *        Blocks fast clocks. RTC stays alive.
 */
void PWR_EnterMinimumProfile(void);

/**
 * @brief Enter MEASURE profile from MINIMUM.
 *        Unblocks fast clocks and enables I2C_0 only.
 *        Call at the start of a safety / charging check window.
 */
void PWR_EnterMeasureProfile(void);

/**
 * @brief Return to MINIMUM profile from MEASURE.
 *        Disables I2C_0 and re-blocks fast clocks.
 *        Call after I2C reads are complete.
 */
void PWR_ExitMeasureProfile(void);

/**
 * @brief Enter ACTIVE profile.
 *        Unblocks fast clocks, enables UART_0, I2C_0, SPI_1, DMA.
 *        Call at entry of POWER_STM state.
 */
void PWR_EnterActiveProfile(void);

/**
 * @brief Exit ACTIVE profile back to MINIMUM.
 *        Disables SPI_1, DMA, I2C_0, UART_0. Re-blocks fast clocks.
 *        Call when leaving POWER_STM state.
 */
void PWR_ExitActiveProfile(void);

void hall_init(void);
void gauge_init(void);

void RTC_EnablePrescaler(void);
void RTC_DisablePrescaler(void);

void PWR_EnableCoreInterrupts(void);
#endif /* POWER_HELPERS_H */