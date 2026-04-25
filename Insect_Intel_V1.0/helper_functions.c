#include "helper_functions.h"
#include "ti_msp_dl_config.h"
#include "HAL/spi_master.h"
#include "HAL/uart.h"
#include "HAL/i2c.h"
/* ═════════════════════════════════════════════════════════════════════════════
 * Clock gating
 * ═══════════════════════════════════════════════════════════════════════════*/

void PWR_BlockFastClocks(void)
{
    DL_SYSCTL_blockAllAsyncFastClockRequests();
    DL_SYSCTL_disableSYSPLL();
}

void PWR_UnblockFastClocks(void)
{
    DL_SYSCTL_allowAllAsyncFastClockRequests();
}

/* ═════════════════════════════════════════════════════════════════════════════
 * I2C_0
 * ═══════════════════════════════════════════════════════════════════════════*/


void PWR_EnableI2C0(void)
{
    DL_I2C_reset(I2C_0_INST);
    DL_I2C_enablePower(I2C_0_INST);
    delay_cycles(POWER_STARTUP_DELAY);
    SYSCFG_DL_I2C_0_init();
    i2c_init();
}

void PWR_DisableI2C0(void)
{
    DL_I2C_disableController(I2C_0_INST);
    DL_I2C_disablePower(I2C_0_INST);
}

/* ═════════════════════════════════════════════════════════════════════════════
 * I2C_1
 * ═══════════════════════════════════════════════════════════════════════════*/

void PWR_EnableI2C1(void)
{
    DL_I2C_reset(I2C_1_INST);
    DL_I2C_enablePower(I2C_1_INST);
    delay_cycles(POWER_STARTUP_DELAY);
    SYSCFG_DL_I2C_1_init();
    i2c_init();
}

void PWR_DisableI2C1(void)
{
    DL_I2C_disableController(I2C_1_INST);
    DL_I2C_disablePower(I2C_1_INST);
}

/* ═════════════════════════════════════════════════════════════════════════════
 * UART_0
 * ═══════════════════════════════════════════════════════════════════════════*/

void PWR_EnableUART0(void)
{
    DL_UART_Main_reset(UART_0_INST);
    DL_UART_Main_enablePower(UART_0_INST);
    delay_cycles(POWER_STARTUP_DELAY);
    SYSCFG_DL_UART_0_init();
    uart_init();
}

void PWR_DisableUART0(void)
{
    DL_UART_Main_disable(UART_0_INST);
    DL_UART_Main_disablePower(UART_0_INST);
}

/* ═════════════════════════════════════════════════════════════════════════════
 * SPI_1 + DMA
 * ═══════════════════════════════════════════════════════════════════════════*/

void PWR_EnableSPI1(void)
{
    DL_SPI_reset(SPI_1_INST);
    DL_SPI_enablePower(SPI_1_INST);
    delay_cycles(POWER_STARTUP_DELAY);
    SYSCFG_DL_SPI_1_init();
    SYSCFG_DL_DMA_init();
    spi_init();
}

void PWR_DisableSPI1(void)
{
    DL_SPI_disable(SPI_1_INST);
    DL_SPI_disablePower(SPI_1_INST);
}

/* ═════════════════════════════════════════════════════════════════════════════
 * Compound profile helpers
 * ═══════════════════════════════════════════════════════════════════════════*/

void PWR_EnterMinimumProfile(void)
{
    PWR_DisableSPI1();
    PWR_DisableI2C1();
}

void PWR_EnterMeasureProfile(void)
{
    PWR_EnableI2C0();
    PWR_UnblockFastClocks();
    PWR_EnableUART0();
    delay_cycles(3200);
}

void PWR_ExitMeasureProfile(void)
{
    PWR_EnableI2C0();
    PWR_DisableUART0();
    PWR_BlockFastClocks();
    delay_cycles(3200);
}

void PWR_EnterActiveProfile(void)
{
    PWR_EnableSPI1();
    delay_cycles(3200);
}

void PWR_ExitActiveProfile(void)
{
    PWR_DisableSPI1();
    delay_cycles(3200);
}

void hall_init(void) {
    DL_GPIO_setPins(DIGITAL_OUTPUT_PORTA_PORT, DIGITAL_OUTPUT_PORTA_HALL_3V_PIN);
    delay_cycles(1000);
    DL_GPIO_clearInterruptStatus(GPIOB, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
}

void gauge_init(void) {
    DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_GAUGE_EN_PIN);
    delay_cycles(320000);
    DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_GAUGE_EN_PIN);
}

void PWR_EnableCoreInterrupts(void)
{
    uart_init();
    i2c_init();
    spi_init();
    NVIC_EnableIRQ(RTC_INT_IRQn);
    NVIC_EnableIRQ(EXTERNAL_INTERRUPT_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(EXTERNAL_INTERRUPT_GPIOA_INT_IRQN);
}

void RTC_EnablePrescaler(void) {
    DL_RTC_enableInterrupt(RTC, DL_RTC_INTERRUPT_PRESCALER1);
}

void RTC_DisablePrescaler(void) {
    DL_RTC_disableInterrupt(RTC, DL_RTC_INTERRUPT_PRESCALER1);
}