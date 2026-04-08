#include "sm.h"
#include "ti_msp_dl_config.h"
#include "HAL/uart.h"
#include "HAL/i2c.h"
#include "HAL/spi_master.h"
#include "ics/BQ25628/BQ25628_functions.h"
#include "ics/BQ27Z7/BQ27Z7_functions.h"
#include <stdio.h>
#include <string.h>

/* ── Timing Constants ────────────────────────────────────── */
#define SM_STM_TIMEOUT_MINUTES    1    // max time to wait for STM32
#define SM_SLEEP_WAKEUP_MINUTES   15   // full system wake interval
#define SM_HALL_ON_MINUTES        5    // Hall sensor ON window
#define SM_HALL_OFF_MINUTES       1    // Hall sensor OFF window
#define SM_FAULT_RETRY_MINUTES    1    // re-check fault interval
#define SM_VBAT_LOW_MV            3000 // low battery threshold
#define SM_SAFETY_POLL_MS         500
#define SM_ADAPTER_DEBOUNCE_MS    5000

/* ── Global Context ──────────────────────────────────────── */
SM_Context_t sm_context;
extern volatile bool rtc_minute_tick;
extern volatile bool hall_wakeup_flag;
extern volatile bool stm_io2_flag;
extern SPI_Controller_Handle stm32Spi;
extern volatile uint32_t systick_ms;
extern volatile bool adapter_check_flag;

/* ── Static variables ────────────────────────────────────── */
static uint32_t last_safety_status = 0;
static char json_buf[256];

/* ── Internal Prototypes ─────────────────────────────────── */
static void SM_Handle_RTC_Tick(void);
static void SM_DecodeSafetyStatus(uint32_t status);

void SM_Init(void) {
    memset(&sm_context, 0, sizeof(SM_Context_t));
    sm_context.current = SM_STATE_INIT;
    sm_context.sm_paused = false;
    sm_context.entry_done = false;
}

void SM_Transition(SM_State_t new_state) {
    const char* old_name = SM_GetStateString();
    sm_context.previous = sm_context.current;
    sm_context.current = new_state;
    sm_context.entry_done = false;
    
    uart_printf("[SM] %s -> %s\n", old_name, SM_GetStateString());
}

const char* SM_GetStateString(void) {
    switch (sm_context.current) {
        case SM_STATE_INIT:           return "INIT";
        case SM_STATE_CHARGING:       return "CHARGING";
        case SM_STATE_POWER_STM:      return "POWER_STM";
        case SM_STATE_SLEEP:          return "SLEEP";
        case SM_STATE_CRITICAL_FAULT: return "CRITICAL_FAULT";
        default:                      return "UNKNOWN";
    }
}

SM_State_t SM_GetState(void) {
    return sm_context.current;
}

static void SM_PostWake_Branch(void) {
    BQ27Z746_UpdateTelemetry(I2C_0_INST);
    uint16_t vbat_mv = BQ27Z746_Get_Voltage_mV();
    uint8_t stat1 = BQ25628E_ReadReg8(BQ25628E_REG_STAT1);
    bool adapter_present = (stat1 & BQ25628E_VBUS_STAT_MASK) != 0;

    if (vbat_mv < SM_VBAT_LOW_MV) {
        if (adapter_present) {
            SM_Transition(SM_STATE_CHARGING);
        } else {
            sm_context.fault_source = SM_FAULT_LOW_BATTERY;
            SM_Transition(SM_STATE_CRITICAL_FAULT);
        }
    } else if (adapter_present && vbat_mv < 3600) {
        SM_Transition(SM_STATE_CHARGING);
    } else {
        SM_Transition(SM_STATE_POWER_STM);
    }
}

void SM_SafetyCheck(void) {
    // Don't run during sleep or fault — sleep is low power, fault handles itself
    if (sm_context.current == SM_STATE_SLEEP ||
        sm_context.current == SM_STATE_CRITICAL_FAULT ||
        sm_context.current == SM_STATE_INIT) return;

    if ((systick_ms - sm_context.last_safety_check_ms) < SM_SAFETY_POLL_MS) return;
    sm_context.last_safety_check_ms = systick_ms;

    // Safety status check
    uint32_t safety = 0;
    BQ27Z746_GetSafetyStatus(I2C_0_INST, &safety);
    if (safety != 0) {
        sm_context.fault_source = SM_FAULT_GAUGE;
        SM_Transition(SM_STATE_CRITICAL_FAULT);
        return;
    }

    // Adapter presence — only relevant during CHARGING
    if (sm_context.current == SM_STATE_CHARGING) {
        uint8_t stat1 = BQ25628E_ReadReg8(BQ25628E_REG_STAT1);
        bool adapter_present = (stat1 & BQ25628E_VBUS_STAT_MASK) != 0;

        if (!adapter_present) {
            if (!sm_context.adapter_missing) {
                sm_context.adapter_missing = true;
                sm_context.adapter_missing_ms = systick_ms;
                uart_printf("[SM] Adapter removed — 5s debounce started\n");
            } else if ((systick_ms - sm_context.adapter_missing_ms) >= SM_ADAPTER_DEBOUNCE_MS) {
                DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_CHARGER_EN_PIN);
                uart_printf("[SM] Adapter absent 5s — stopping charge\n");
                sm_context.adapter_missing = false;
                SM_PostWake_Branch();
            }
        } else {
            sm_context.adapter_missing = false;
        }
    }
}


static void SM_Handle_RTC_Tick(void) {
    if (!rtc_minute_tick) return;
    rtc_minute_tick = false;
    sm_context.minute_counter++;

    // Hall sensor duty cycle — ON x min, OFF x min
    uint32_t hall_phase = sm_context.minute_counter
                          % (SM_HALL_ON_MINUTES + SM_HALL_OFF_MINUTES);
    if (hall_phase < SM_HALL_ON_MINUTES) {
        if (!sm_context.hall_powered) {
            DL_GPIO_setPins(DIGITAL_OUTPUT_PORTA_PORT,
                            DIGITAL_OUTPUT_PORTA_HALL_3V_PIN);
            sm_context.hall_powered = true;
        }
    } else {
        if (sm_context.hall_powered) {
            DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTA_PORT,
                              DIGITAL_OUTPUT_PORTA_HALL_3V_PIN);
            sm_context.hall_powered = false;
        }
    }
    // 15-minute relative sleep wake
    if (sm_context.current == SM_STATE_SLEEP) {
        uint32_t elapsed = sm_context.minute_counter
                           - sm_context.sleep_entry_tick;
        if (elapsed >= SM_SLEEP_WAKEUP_MINUTES) {
            SM_PostWake_Branch();
        }
    }
}

void SM_Run(void) {
    SM_Handle_RTC_Tick();
    switch (sm_context.current) {
        case SM_STATE_INIT: {
            if (!sm_context.entry_done) {
                DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_GAUGE_EN_PIN);
                if (!I2C_TryAddress(I2C_0_INST, GAUGE_I2C_ADDR) ||
                    !I2C_TryAddress(I2C_0_INST, BQ25628E_I2C_ADDR)) {
                    DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_GAUGE_EN_PIN);
                    DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_GAUGE_EN_PIN);
                    if (!I2C_TryAddress(I2C_0_INST, GAUGE_I2C_ADDR) ||
                        !I2C_TryAddress(I2C_0_INST, BQ25628E_I2C_ADDR)) {
                        sm_context.fault_source = SM_FAULT_I2C_BUS;
                        SM_Transition(SM_STATE_CRITICAL_FAULT);
                        return;
                    }
                }
                bool gauge_ok   = BQ27Z746_Init(I2C_0_INST);
                bool charger_ok = BQ25628E_Init_Default();
                if (!gauge_ok || !charger_ok) {
                    sm_context.fault_source = SM_FAULT_INIT_FAILED;
                    SM_Transition(SM_STATE_CRITICAL_FAULT);
                }else {
                    SM_PostWake_Branch();
                }
            }
            break;
        }
        case SM_STATE_CHARGING: {
            if (!sm_context.entry_done) {
                if (last_safety_status != 0) {
                    uart_printf("[SM] Charging blocked — gauge fault active\n");
                    sm_context.fault_source = SM_FAULT_GAUGE;
                    SM_Transition(SM_STATE_CRITICAL_FAULT);
                    return;
                }
                DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_CHARGER_EN_PIN);
                uart_printf("[SM] Charging started\n");
                sm_context.adapter_missing = false;
                sm_context.entry_done = true;
            }

            static uint32_t last_charging_tick = 0;
            if (sm_context.minute_counter != last_charging_tick) {
                last_charging_tick = sm_context.minute_counter;

                BQ25628E_UpdateTelemetry();
                BQ27Z746_UpdateTelemetry(I2C_0_INST);
                BQ27Z746_GetSafetyStatus(I2C_0_INST, &last_safety_status);

                uint8_t stat1         = BQ25628E_ReadReg8(BQ25628E_REG_STAT1);
                bool adapter_present  = (stat1 & BQ25628E_VBUS_STAT_MASK) != 0;
                uint8_t chg_stat      = (stat1 >> 3) & 0x03;  // CHG_STAT[4:3]

            if (last_safety_status != 0) {
                sm_context.fault_source = SM_FAULT_GAUGE;
                SM_Transition(SM_STATE_CRITICAL_FAULT);
                break;
            }

            // Low battery check — if adapter gone and still under 3V, back to fault
            if (!adapter_present) {
                uint16_t vbat = BQ27Z746_Get_Voltage_mV();
                if (vbat < 3000) {
                    sm_context.fault_source = SM_FAULT_LOW_BATTERY;
                    SM_Transition(SM_STATE_CRITICAL_FAULT);
                    break;
                }
            }

            // Charging complete: voltage at or above 3600mV, gauge full, charger done
            bool gauge_full   = BQ27Z746_IsFullyCharged();
            bool charger_done = (chg_stat == 0 || chg_stat == 3);
            uint16_t vbat     = BQ27Z746_Get_Voltage_mV();
            if ((gauge_full && charger_done) || vbat >= 3600) {
                DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_CHARGER_EN_PIN);
                uart_printf("[SM] Charging complete\n");
                SM_PostWake_Branch();
            } else {
                uart_printf("[SM] CHARGING | VBUS:%4dmV VBAT:%4dmV IBUS:%4dmA IBAT:%4dmA SOC:%3d%% CHG_STAT:%d\n",
                    BQ25628E_Get_VBUS_mV(),
                    BQ27Z746_Get_Voltage_mV(),
                    BQ25628E_Get_IBUS_mA(),
                    BQ27Z746_Get_Current_mA(),
                    BQ27Z746_Get_SOC_pct(),
                    chg_stat);
            }
           
            }
             break;
        }

        case SM_STATE_POWER_STM: {
            if (!sm_context.entry_done) {
                if (sm_context.wake_reason == SM_WAKE_SETUP) {
                    DL_GPIO_setPins(DIGITAL_OUTPUT_PORTA_PORT, DIGITAL_OUTPUT_PORTA_STM_MCU_IO1_PIN);
                } else {
                    DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTA_PORT, DIGITAL_OUTPUT_PORTA_STM_MCU_IO1_PIN);
                }
                DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_EN3V8_PIN);
                DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_STM_PON_PIN);
                sm_context.stm_power_on_ms = systick_ms;
                uart_printf("[SM] STM32 powered — reason: %s\n", 
                            (sm_context.wake_reason == SM_WAKE_SETUP) ? "SETUP" : "NORMAL");
                stm_io2_flag = false;
                sm_context.entry_done = true;
                DL_GPIO_disableInterrupt(EXTERNAL_INTERRUPT_CHARGER_INT_PORT, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
                DL_GPIO_enableInterrupt(EXTERNAL_INTERRUPT_STM_MCU_IO2_PORT, EXTERNAL_INTERRUPT_STM_MCU_IO2_PIN);
            }

            if (stm_io2_flag) {
                uint8_t stat1_json = BQ25628E_ReadReg8(BQ25628E_REG_STAT1);
                uint8_t chg_stat   = (stat1_json >> 3) & 0x03;

                snprintf(json_buf, sizeof(json_buf),
                    "{\"soc\":%d,\"soh\":%d,\"vchg\":%d,\"ichg\":%d,\"vbat\":%d,\"ibat\":%d,\"temp\":%d,\"prot\":\"0x%08X\",\"chg\":%d}",
                    BQ27Z746_Get_SOC_pct(),
                    BQ27Z746_Get_StateOfHealth_pct(),
                    BQ25628E_Get_VBUS_mV(),
                    BQ25628E_Get_IBUS_mA(),
                    BQ27Z746_Get_Voltage_mV(),
                    BQ27Z746_Get_Current_mA(),
                    BQ27Z746_Get_InternalTemp_C(),
                    (unsigned int)last_safety_status,
                    chg_stat
                );     
                size_t len = strlen(json_buf);
                memset(stm32Spi.txBuf, 0, stm32Spi.size);
                memcpy(stm32Spi.txBuf, json_buf, (len < stm32Spi.size) ? len : stm32Spi.size);
                
                SPI_Controller_Arm(&stm32Spi);
                
                // Non-blocking wait for IO2 to go low
                while (DL_GPIO_readPins(EXTERNAL_INTERRUPT_STM_MCU_IO2_PORT, EXTERNAL_INTERRUPT_STM_MCU_IO2_PIN)) {
                    // Just wait for pin to go low. In a real system we might want a small timeout here 
                    // or let it happen in next SM_Run call, but instructions say poll pin state.
                    // To keep it non-blocking relative to main loop, we should ideally check it and return.
                    // But "Wait for STM_MCU_IO2 to return LOW" usually implies a tight loop if it's short.
                    // Given the constraint "SM_Run() is non-blocking", I will check and return if not low.
                    return; 
                }
                stm_io2_flag = false;
                DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_STM_PON_PIN);
                DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTA_PORT, DIGITAL_OUTPUT_PORTA_STM_MCU_IO1_PIN);
                SM_Transition(SM_STATE_SLEEP);
            }

            if ((systick_ms - sm_context.stm_power_on_ms) >= 60000UL) {
                uart_printf("[SM] STM32 timeout — forcing sleep\n");
                DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_STM_PON_PIN);
                DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTA_PORT, DIGITAL_OUTPUT_PORTA_STM_MCU_IO1_PIN);
                SM_Transition(SM_STATE_SLEEP);
            }
            break;
        }

        case SM_STATE_SLEEP: {
            if (!sm_context.entry_done) {
                sm_context.wake_reason = SM_WAKE_NORMAL;
                DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_STM_PON_PIN);
                DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_EN3V8_PIN);
                sm_context.sleep_entry_tick = sm_context.minute_counter;
                
                DL_GPIO_disableInterrupt(EXTERNAL_INTERRUPT_STM_MCU_IO2_PORT, EXTERNAL_INTERRUPT_STM_MCU_IO2_PIN);
                DL_GPIO_enableInterrupt(EXTERNAL_INTERRUPT_CHARGER_INT_PORT, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
                hall_wakeup_flag = false;
                uart_printf("[SM] Entering sleep\n");
                sm_context.entry_done = true;
            }

            if (hall_wakeup_flag) {
                hall_wakeup_flag = false;
                sm_context.wake_reason = SM_WAKE_SETUP;
                SM_PostWake_Branch();
            }
            else {
                __WFI();
            }
            break;
        }

        case SM_STATE_CRITICAL_FAULT: {
            if (!sm_context.entry_done) {
                DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_STM_PON_PIN);
                DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_EN3V8_PIN);
                
                const char* fault_str = "UNKNOWN";
                switch (sm_context.fault_source) {
                    case SM_FAULT_I2C_BUS:      fault_str = "I2C_BUS"; break;
                    case SM_FAULT_GAUGE:        fault_str = "GAUGE"; break;
                    case SM_FAULT_LOW_BATTERY:  fault_str = "LOW_BATTERY"; break;
                    case SM_FAULT_INIT_FAILED:  fault_str = "INIT_FAILED"; break;
                    default: break;
                }
                uart_printf("[SM] CRITICAL FAULT, source: %s\n", fault_str);
                sm_context.fault_retry_ms = systick_ms;
                if (sm_context.fault_source == SM_FAULT_I2C_BUS) {
                    uart_printf("[SM] I2C bus failure can't initialize system(Power cycle system)\n");
                } else {
                    SM_DecodeSafetyStatus(last_safety_status);
                }
                sm_context.entry_done = true;
            }

            if (sm_context.fault_source != SM_FAULT_I2C_BUS &&
                (systick_ms - sm_context.fault_retry_ms) >= 5000UL) {
                sm_context.fault_retry_ms = systick_ms;
                BQ27Z746_GetSafetyStatus(I2C_0_INST, &last_safety_status);

                if (sm_context.fault_source == SM_FAULT_LOW_BATTERY) {
                    uint8_t stat1 = BQ25628E_ReadReg8(BQ25628E_REG_STAT1);
                    bool adapter_present = (stat1 & BQ25628E_VBUS_STAT_MASK) != 0;
                    if (adapter_present) {
                        uart_printf("[SM] Adapter detected — resuming charge\n");
                        SM_PostWake_Branch();
                    } else {
                        uart_printf("[SM] Low battery fault — still waiting for adapter\n");
                    }
                } else if (last_safety_status != 0) {
                    uart_printf("[SM] Fault still active: 0x%08X\n", (unsigned int)last_safety_status);
                    SM_DecodeSafetyStatus(last_safety_status);
                } else {
                    SM_PostWake_Branch();
                }
            }
            break;
            }

        }
}

static void SM_DecodeSafetyStatus(uint32_t status) {
    if (status & BQ27Z746_SAFETY_CUV)  uart_printf("[FAULT] CUV  — Cell Undervoltage\n");
    if (status & BQ27Z746_SAFETY_OVP)  uart_printf("[FAULT] COV  — Cell Overvoltage\n");
    if (status & BQ27Z746_SAFETY_OCC)  uart_printf("[FAULT] OCC  — Overcurrent During Charge\n");
    if (status & BQ27Z746_SAFETY_OCD)  uart_printf("[FAULT] OCD  — Overcurrent During Discharge\n");
    if (status & BQ27Z746_SAFETY_HOCD) uart_printf("[FAULT] HOCD — Overload During Discharge\n");
    if (status & BQ27Z746_SAFETY_HOCC) uart_printf("[FAULT] HOCC — Short-Circuit During Charge\n");
    if (status & BQ27Z746_SAFETY_SCD)  uart_printf("[FAULT] HSCD — Hardware Short-Circuit Discharge\n");
    if (status & BQ27Z746_SAFETY_OTC)  uart_printf("[FAULT] OTC  — Over-Temperature During Charge\n");
    if (status & BQ27Z746_SAFETY_OTD)  uart_printf("[FAULT] OTD  — Over-Temperature During Discharge\n");
    if (status & BQ27Z746_SAFETY_OTF)  uart_printf("[FAULT] OTF  — Over-Temperature FET\n");
    if (status & BQ27Z746_SAFETY_PTO)  uart_printf("[FAULT] PTO  — Precharge Timeout\n");
    if (status & BQ27Z746_SAFETY_CTO)  uart_printf("[FAULT] CTO  — Charge Timeout\n");
    if (status & BQ27Z746_SAFETY_UTC)  uart_printf("[FAULT] UTC  — Under-Temperature During Charge\n");
    if (status & BQ27Z746_SAFETY_UTD)  uart_printf("[FAULT] UTD  — Under-Temperature During Discharge\n");
    if (status & BQ27Z746_SAFETY_HCOV) uart_printf("[FAULT] HCOV — Hardware Cell Overvoltage\n");
    if (status & BQ27Z746_SAFETY_HCUV) uart_printf("[FAULT] HCUV — Hardware Cell Undervoltage\n");
}
