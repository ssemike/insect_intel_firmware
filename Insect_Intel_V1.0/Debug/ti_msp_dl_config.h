/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define GPIO_LFXT_PORT                                                     GPIOA
#define GPIO_LFXIN_PIN                                             DL_GPIO_PIN_3
#define GPIO_LFXIN_IOMUX                                          (IOMUX_PINCM8)
#define GPIO_LFXOUT_PIN                                            DL_GPIO_PIN_4
#define GPIO_LFXOUT_IOMUX                                         (IOMUX_PINCM9)
#define CPUCLK_FREQ                                                      4000000




/* Defines for I2C_0 */
#define I2C_0_INST                                                          I2C0
#define I2C_0_INST_IRQHandler                                    I2C0_IRQHandler
#define I2C_0_INST_INT_IRQN                                        I2C0_INT_IRQn
#define I2C_0_BUS_SPEED_HZ                                                100000
#define GPIO_I2C_0_SDA_PORT                                                GPIOA
#define GPIO_I2C_0_SDA_PIN                                        DL_GPIO_PIN_28
#define GPIO_I2C_0_IOMUX_SDA                                      (IOMUX_PINCM3)
#define GPIO_I2C_0_IOMUX_SDA_FUNC                       IOMUX_PINCM3_PF_I2C0_SDA
#define GPIO_I2C_0_SCL_PORT                                                GPIOA
#define GPIO_I2C_0_SCL_PIN                                        DL_GPIO_PIN_31
#define GPIO_I2C_0_IOMUX_SCL                                      (IOMUX_PINCM6)
#define GPIO_I2C_0_IOMUX_SCL_FUNC                       IOMUX_PINCM6_PF_I2C0_SCL

/* Defines for I2C_1 */
#define I2C_1_INST                                                          I2C1
#define I2C_1_INST_IRQHandler                                    I2C1_IRQHandler
#define I2C_1_INST_INT_IRQN                                        I2C1_INT_IRQn
#define I2C_1_BUS_SPEED_HZ                                                100000
#define GPIO_I2C_1_SDA_PORT                                                GPIOA
#define GPIO_I2C_1_SDA_PIN                                        DL_GPIO_PIN_30
#define GPIO_I2C_1_IOMUX_SDA                                      (IOMUX_PINCM5)
#define GPIO_I2C_1_IOMUX_SDA_FUNC                       IOMUX_PINCM5_PF_I2C1_SDA
#define GPIO_I2C_1_SCL_PORT                                                GPIOA
#define GPIO_I2C_1_SCL_PIN                                        DL_GPIO_PIN_29
#define GPIO_I2C_1_IOMUX_SCL                                      (IOMUX_PINCM4)
#define GPIO_I2C_1_IOMUX_SCL_FUNC                       IOMUX_PINCM4_PF_I2C1_SCL


/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_FREQUENCY                                              32768
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                                  (9600)
#define UART_0_IBRD_33_kHZ_9600_BAUD                                         (1)
#define UART_0_FBRD_33_kHZ_9600_BAUD                                         (9)




/* Defines for SPI_1 */
#define SPI_1_INST                                                         SPI1
#define SPI_1_INST_IRQHandler                                   SPI1_IRQHandler
#define SPI_1_INST_INT_IRQN                                       SPI1_INT_IRQn
#define GPIO_SPI_1_PICO_PORT                                              GPIOB
#define GPIO_SPI_1_PICO_PIN                                      DL_GPIO_PIN_22
#define GPIO_SPI_1_IOMUX_PICO                                   (IOMUX_PINCM50)
#define GPIO_SPI_1_IOMUX_PICO_FUNC                   IOMUX_PINCM50_PF_SPI1_PICO
#define GPIO_SPI_1_POCI_PORT                                              GPIOB
#define GPIO_SPI_1_POCI_PIN                                      DL_GPIO_PIN_21
#define GPIO_SPI_1_IOMUX_POCI                                   (IOMUX_PINCM49)
#define GPIO_SPI_1_IOMUX_POCI_FUNC                   IOMUX_PINCM49_PF_SPI1_POCI
/* GPIO configuration for SPI_1 */
#define GPIO_SPI_1_SCLK_PORT                                              GPIOB
#define GPIO_SPI_1_SCLK_PIN                                      DL_GPIO_PIN_23
#define GPIO_SPI_1_IOMUX_SCLK                                   (IOMUX_PINCM51)
#define GPIO_SPI_1_IOMUX_SCLK_FUNC                   IOMUX_PINCM51_PF_SPI1_SCLK
#define GPIO_SPI_1_CS1_PORT                                               GPIOB
#define GPIO_SPI_1_CS1_PIN                                       DL_GPIO_PIN_27
#define GPIO_SPI_1_IOMUX_CS1                                    (IOMUX_PINCM58)
#define GPIO_SPI_1_IOMUX_CS1_FUNC               IOMUX_PINCM58_PF_SPI1_CS1_POCI1



/* Defines for DMA_CH1 */
#define DMA_CH1_CHAN_ID                                                      (1)
#define SPI_1_INST_DMA_TRIGGER_0                              (DMA_SPI1_RX_TRIG)
/* Defines for DMA_CH0 */
#define DMA_CH0_CHAN_ID                                                      (0)
#define SPI_1_INST_DMA_TRIGGER_1                              (DMA_SPI1_TX_TRIG)


/* Defines for CHARGER_INT: GPIOB.1 with pinCMx 13 on package pin 48 */
#define EXTERNAL_INTERRUPT_CHARGER_INT_PORT                              (GPIOB)
#define EXTERNAL_INTERRUPT_CHARGER_INT_PIN                       (DL_GPIO_PIN_1)
#define EXTERNAL_INTERRUPT_CHARGER_INT_IOMUX                     (IOMUX_PINCM13)
/* Defines for SETUP_INT: GPIOB.20 with pinCMx 48 on package pin 19 */
#define EXTERNAL_INTERRUPT_SETUP_INT_PORT                                (GPIOB)
// pins affected by this interrupt request:["SETUP_INT"]
#define EXTERNAL_INTERRUPT_GPIOB_INT_IRQN                       (GPIOB_INT_IRQn)
#define EXTERNAL_INTERRUPT_GPIOB_INT_IIDX       (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define EXTERNAL_INTERRUPT_SETUP_INT_IIDX                   (DL_GPIO_IIDX_DIO20)
#define EXTERNAL_INTERRUPT_SETUP_INT_PIN                        (DL_GPIO_PIN_20)
#define EXTERNAL_INTERRUPT_SETUP_INT_IOMUX                       (IOMUX_PINCM48)
/* Defines for STM_MCU_IO2: GPIOA.26 with pinCMx 59 on package pin 30 */
#define EXTERNAL_INTERRUPT_STM_MCU_IO2_PORT                              (GPIOA)
// pins affected by this interrupt request:["STM_MCU_IO2"]
#define EXTERNAL_INTERRUPT_GPIOA_INT_IRQN                       (GPIOA_INT_IRQn)
#define EXTERNAL_INTERRUPT_GPIOA_INT_IIDX       (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define EXTERNAL_INTERRUPT_STM_MCU_IO2_IIDX                 (DL_GPIO_IIDX_DIO26)
#define EXTERNAL_INTERRUPT_STM_MCU_IO2_PIN                      (DL_GPIO_PIN_26)
#define EXTERNAL_INTERRUPT_STM_MCU_IO2_IOMUX                     (IOMUX_PINCM59)
/* Port definition for Pin Group DIGITAL_OUTPUT_PORTB */
#define DIGITAL_OUTPUT_PORTB_PORT                                        (GPIOB)

/* Defines for CHARGER_EN: GPIOB.0 with pinCMx 12 on package pin 47 */
#define DIGITAL_OUTPUT_PORTB_CHARGER_EN_PIN                      (DL_GPIO_PIN_0)
#define DIGITAL_OUTPUT_PORTB_CHARGER_EN_IOMUX                    (IOMUX_PINCM12)
/* Defines for CHARGER_QON: GPIOB.2 with pinCMx 15 on package pin 50 */
#define DIGITAL_OUTPUT_PORTB_CHARGER_QON_PIN                     (DL_GPIO_PIN_2)
#define DIGITAL_OUTPUT_PORTB_CHARGER_QON_IOMUX                   (IOMUX_PINCM15)
/* Defines for EN3V8: GPIOB.11 with pinCMx 28 on package pin 63 */
#define DIGITAL_OUTPUT_PORTB_EN3V8_PIN                          (DL_GPIO_PIN_11)
#define DIGITAL_OUTPUT_PORTB_EN3V8_IOMUX                         (IOMUX_PINCM28)
/* Defines for MCU_LTE_PON: GPIOB.13 with pinCMx 30 on package pin 1 */
#define DIGITAL_OUTPUT_PORTB_MCU_LTE_PON_PIN                    (DL_GPIO_PIN_13)
#define DIGITAL_OUTPUT_PORTB_MCU_LTE_PON_IOMUX                   (IOMUX_PINCM30)
/* Defines for STM_PON: GPIOB.14 with pinCMx 31 on package pin 2 */
#define DIGITAL_OUTPUT_PORTB_STM_PON_PIN                        (DL_GPIO_PIN_14)
#define DIGITAL_OUTPUT_PORTB_STM_PON_IOMUX                       (IOMUX_PINCM31)
/* Defines for MCU_WIFI_PON: GPIOB.15 with pinCMx 32 on package pin 3 */
#define DIGITAL_OUTPUT_PORTB_MCU_WIFI_PON_PIN                   (DL_GPIO_PIN_15)
#define DIGITAL_OUTPUT_PORTB_MCU_WIFI_PON_IOMUX                  (IOMUX_PINCM32)
/* Defines for CAM_SYNC: GPIOB.10 with pinCMx 27 on package pin 62 */
#define DIGITAL_OUTPUT_PORTB_CAM_SYNC_PIN                       (DL_GPIO_PIN_10)
#define DIGITAL_OUTPUT_PORTB_CAM_SYNC_IOMUX                      (IOMUX_PINCM27)
/* Defines for GAUGE_EN: GPIOB.16 with pinCMx 33 on package pin 4 */
#define DIGITAL_OUTPUT_PORTB_GAUGE_EN_PIN                       (DL_GPIO_PIN_16)
#define DIGITAL_OUTPUT_PORTB_GAUGE_EN_IOMUX                      (IOMUX_PINCM33)
/* Port definition for Pin Group DIGITAL_OUTPUT_PORTA */
#define DIGITAL_OUTPUT_PORTA_PORT                                        (GPIOA)

/* Defines for HALL_3V: GPIOA.7 with pinCMx 14 on package pin 49 */
#define DIGITAL_OUTPUT_PORTA_HALL_3V_PIN                         (DL_GPIO_PIN_7)
#define DIGITAL_OUTPUT_PORTA_HALL_3V_IOMUX                       (IOMUX_PINCM14)
/* Defines for STM_MCU_IO1: GPIOA.27 with pinCMx 60 on package pin 31 */
#define DIGITAL_OUTPUT_PORTA_STM_MCU_IO1_PIN                    (DL_GPIO_PIN_27)
#define DIGITAL_OUTPUT_PORTA_STM_MCU_IO1_IOMUX                   (IOMUX_PINCM60)





/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_I2C_0_init(void);
void SYSCFG_DL_I2C_1_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_SPI_1_init(void);
void SYSCFG_DL_DMA_init(void);

void SYSCFG_DL_RTC_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
