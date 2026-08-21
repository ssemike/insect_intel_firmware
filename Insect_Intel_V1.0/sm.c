#include "sm.h"
#include "ti_msp_dl_config.h"
#include "HAL/uart.h"
#include "HAL/i2c.h"
#include "HAL/spi_master.h"
#include "ics/BQ25628/BQ25628_functions.h"
#include "ics/BQ27Z7/BQ27Z7_functions.h"
#include <stdio.h>
#include <string.h>
#include "helper_functions.h"

/* ── Timing Constants ────────────────────────────────────── */
#define SM_VBAT_LOW_MV            3000
#define SM_VBAT_FULL_MV           3650
#define SM_VBAT_CHARGE_START_MV   3400
#define SM_SAFETY_POLL_S          5
#define SM_INACTIVITY_TIMEOUT_S         600
#define SM_I2C_RETRY_S   5
#define SM_FAULT_RETRY_S 5

/* ── Global Context ──────────────────────────────────────── */
SM_Context_t sm_context;
extern volatile bool rtc_minute_tick;
extern volatile bool rtc_second_tick;
extern volatile bool hall_wakeup_flag;
extern volatile bool stm_io2_flag;
extern SPI_Controller_Handle stm32Spi;

/* ── Static variables ────────────────────────────────────── */
static uint32_t last_safety_status = 0;
static uint8_t last_charger_status = 0;
static char json_buf[512];

/* ── Internal Prototypes ─────────────────────────────────── */
static void SM_Handle_RTC_Tick(void);
static void SM_DecodeBatterySafetyStatus(uint32_t status);
static void SM_DecodeChargingSafetyStatus(uint8_t status);
static bool SM_NeedsPeriodicSTMWake(SM_PowerContext_t pwr);
static void SM_ResumeSystemContext(void);
static SM_PowerContext_t SM_FetchPowerContext(void);
static void SM_SetSTMPower(bool enable);
static void SM_SendOffer(void);
static void SM_DispatchIncomingPacket(void);
static bool SM_ProcessFault(uint32_t gauge_safety, uint8_t charger_fault);
static void SM_PrepareTelemetryResponse(void);
static void SM_PrepareSTMConfigResponse(void);
static void SM_PrepareSTMCredentialsResponse(void);

static void SM_Heartbeat(void);

static void SM_HandleState_INIT(void);
static void SM_HandleState_CHARGING(void);
static void SM_HandleState_POWER_STM(void);
static void SM_HandleState_IDLE(void);
static void SM_HandleState_CRITICAL_FAULT(void);


/* ── Heartbeat ───────────────────────────────────────────── */

/*
 * Printed once per minute, on the minute-tick branch of IDLE and CHARGING —
 * i.e. immediately after PWR_EnterMeasureProfile() has brought the UART back
 * up, which is the only window in those states where a print is possible.
 *
 * Its real job is liveness: if these stop arriving, the main loop has stalled,
 * and the timestamp of the last one tells you when.
 */
static void SM_Heartbeat(void)
{
    uint32_t period      = sm_context.stm_wake_period.wake_interval_minutes;
    uint32_t since_wake  = sm_context.minute_counter - sm_context.last_stm_periodic_minute;
    uint32_t next_wake   = (since_wake >= period) ? 0U : (period - since_wake);

    uart_printf("[HB] %s | up %lum | %lum since STM wake | next in %lum | wakes:%lu | i2c t/o:%lu rec:%lu\n",
        SM_GetStateString(),
        (unsigned long)sm_context.minute_counter,
        (unsigned long)since_wake,
        (unsigned long)next_wake,
        (unsigned long)sm_context.total_wakes,
        (unsigned long)I2C_GetTimeoutCount(),
        (unsigned long)I2C_GetRecoveryCount());
}

/* ── Hardware Abstraction Helpers ────────────────────────── */

// Replaces repetitive GPIO calls for STM32 power
static void SM_SetSTMPower(bool enable) {
    if (enable) {
        PWR_EnterActiveProfile();
        DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_EN3V8_PIN | DIGITAL_OUTPUT_PORTB_STM_PON_PIN);
    } else {
        DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_EN3V8_PIN | DIGITAL_OUTPUT_PORTB_STM_PON_PIN);
        DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTA_PORT, DIGITAL_OUTPUT_PORTA_STM_MCU_IO1_PIN);
        PWR_ExitActiveProfile();
    }
}

// Single source of truth for current power state
static SM_PowerContext_t SM_FetchPowerContext(void) {
    SM_PowerContext_t ctx;
    BQ25628E_ADC_Control(true);
    BQ27Z746_UpdateTelemetry(I2C_0_INST);
    BQ25628E_UpdateTelemetry();   
    uint16_t batt_status = BQ27Z746_Get_BatteryStatus();
    ctx.vbus_mv = BQ25628E_Get_VBUS_mV();
    ctx.vbat_mv = BQ27Z746_Get_Voltage_mV();
    ctx.stat1 = BQ25628E_ReadReg8(BQ25628E_REG_STAT1);
    ctx.adapter_present = (ctx.stat1 & BQ25628E_VBUS_STAT_MASK) != 0;
    ctx.chg_stat = (ctx.stat1 >> 3) & 0x03;
    ctx.is_critical_low = (ctx.vbat_mv < SM_VBAT_LOW_MV) && !ctx.adapter_present;
    bool gauge_charging = (batt_status & BQ27Z746_STATUS_DSG) == 0;
    bool charger_active = (ctx.chg_stat == 1 || ctx.chg_stat == 2);   
    ctx.is_charging = gauge_charging || charger_active;
    ctx.charger_done = (ctx.chg_stat == 0 || ctx.chg_stat == 3);
    BQ25628E_ADC_Control(false);
    return ctx;
}

/* ── Core State Machine Functions ────────────────────────── */

void SM_Init(void)
{
    memset(&sm_context, 0, sizeof(SM_Context_t));
    sm_context.current = SM_STATE_INIT;
    sm_context.first_boot = true;
    SM_LoadCharger();
    SM_LoadPeriod();
    SM_LoadCredentials();
    SM_LoadSTMConfig();
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
        case SM_STATE_IDLE:           return "IDLE";
        case SM_STATE_CRITICAL_FAULT: return "CRITICAL_FAULT";
        default:                      return "UNKNOWN";
    }
}

const char* SM_GetChargeString(uint8_t chg_stat) {
    switch (chg_stat) {
        case 0: return "Not Charging / Terminated";
        case 1: return "Pre/Trickle/Fast (CC)"; 
        case 2: return "Taper (CV)";
        case 3: return "Top-Off";
        default: return "Unknown";
    }
}

SM_State_t SM_GetState(void) {
    return sm_context.current;
}

static void SM_PostWake_Branch(void) {
    SM_PowerContext_t pwr = SM_FetchPowerContext();
    if (pwr.is_critical_low) {
        uart_printf("[SM] Critical low battery detected, entering IDLE to conserve power\n");
        SM_Transition(SM_STATE_IDLE);      
    } else if (pwr.vbat_mv < SM_VBAT_CHARGE_START_MV && pwr.adapter_present) {
        SM_Transition(SM_STATE_CHARGING);
    } else {
        sm_context.wake_reason = SM_WAKE_NORMAL;
        SM_Transition(SM_STATE_POWER_STM);  
    }
}

static void SM_Handle_RTC_Tick(void) {
    if (!rtc_minute_tick && !rtc_second_tick) return;
    
    if (rtc_minute_tick) {
        rtc_minute_tick = false;
        sm_context.minute_counter++;
    }
    if (rtc_second_tick) {
        rtc_second_tick = false;
        sm_context.second_counter++;
        if ((sm_context.second_counter - sm_context.last_safety_check_s) >= SM_SAFETY_POLL_S) {
            sm_context.last_safety_check_s = sm_context.second_counter;
            if (SM_SafetyCheck()) return;
        }
    }  
}

static void SM_HandleState_INIT(void) {

    if (!sm_context.entry_done) {
        PWR_EnterMeasureProfile();
        sm_context.entry_done = true;

        if (!I2C_TryAddress(I2C_0_INST, GAUGE_I2C_ADDR) || !I2C_TryAddress(I2C_0_INST, BQ25628E_I2C_ADDR)) {
            sm_context.fault_source = SM_FAULT_I2C_BUS;
            SM_Transition(SM_STATE_CRITICAL_FAULT);
            return;
        }
        bool gauge_ok   = BQ27Z746_Init(I2C_0_INST);
        BQ25628E_HardwareInit();
        BQ25628E_ApplyProfile(&sm_context.sm_charger_config);
        bool charger_ok = true;
                        
        if (!gauge_ok || !charger_ok) {
            sm_context.fault_source = SM_FAULT_INIT_FAILED;
            SM_Transition(SM_STATE_CRITICAL_FAULT);
        } else {
            SM_PostWake_Branch();
        }
    }
}

static void SM_HandleState_CHARGING(void) {

    if (!sm_context.entry_done) {
        /* This block prints before it powers the bus down, so establish the
         * profile rather than trusting the transition we arrived through. */
        PWR_EnterMeasureProfile();
        DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_CHARGER_EN_PIN);
        uart_printf("[SM] Charging started\n");
        sm_context.critical_msg_sent = false;
        sm_context.entry_done = true;
        sm_context.last_charging_tick = sm_context.minute_counter; 
        RTC_DisablePrescaler();
        DL_GPIO_clearInterruptStatus(GPIOB, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
        DL_GPIO_enableInterrupt(GPIOB, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
        sm_context.wake_reason = SM_WAKE_NORMAL;
        PWR_ExitMeasureProfile();
    }
    if (hall_wakeup_flag) {
        hall_wakeup_flag = false;
        sm_context.wake_reason = SM_WAKE_SETUP;
        RTC_EnablePrescaler();
        PWR_EnterMeasureProfile();
        SM_Transition(SM_STATE_POWER_STM);
        return;
    }
    if (sm_context.minute_counter != sm_context.last_charging_tick) {
        sm_context.last_charging_tick = sm_context.minute_counter;
        PWR_EnterMeasureProfile();
        SM_Heartbeat();
        if (SM_SafetyCheck()) return;
        if (SM_ChargingSafetyCheck()) return;
        SM_PowerContext_t pwr = SM_FetchPowerContext();
        if (pwr.vbat_mv >= SM_VBAT_FULL_MV || pwr.charger_done) {
            DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_CHARGER_EN_PIN);
            uart_printf("[SM] Charging complete (VBAT %dmV >= %dmV) / Charging terminated: \n", pwr.vbat_mv, SM_VBAT_FULL_MV);
            SM_Transition(SM_STATE_IDLE);
            return;
        } else {
            uart_printf("[SM] CHARGING | VBUS:%4dmV VBAT:%4dmV IBAT:%4dmA SOC:%3d%% TBAT:%3.1fC TDIE:%3dC CHG_STAT:%s\n",
                BQ25628E_Get_VBUS_mV(), pwr.vbat_mv, BQ27Z746_Get_Current_mA(),
                BQ27Z746_Get_SOC_pct(), BQ25628E_Get_TBAT_C(), BQ25628E_Get_TDIE_C(),
                SM_GetChargeString(pwr.chg_stat));
        }            
        if (SM_NeedsPeriodicSTMWake(pwr)) {
            RTC_EnablePrescaler();
            sm_context.last_stm_periodic_minute = sm_context.minute_counter;
            SM_Transition(SM_STATE_POWER_STM);
            return;
        }
    }
    PWR_ExitMeasureProfile();
    __WFI();
}

/* 32 MHz core: 32000 cycles per millisecond. Same basis as the 200 ms delays
 * in functions.c and the 10 ms one in gauge_init(). */
#define SM_MS_CYCLES(ms)   ((uint32_t)(ms) * 32000UL)

/* Long enough for the 3V8 rail to actually collapse and for the STM32's
 * external NOR to see a genuine power-on reset. Too short and the flash keeps
 * its octal-DTR mode across the "cycle", which reproduces exactly the failure
 * this whole mechanism exists to avoid. */
#define SM_POWER_CYCLE_OFF_MS   500U

/* Small gap after the acknowledgement transfer completes, so the STM32 has a
 * moment to stop touching the SD card. It has already committed everything
 * that matters before it asks, so this is politeness rather than safety. */
#define SM_POWER_CYCLE_SETTLE_MS 50U

/**
 * Remove the STM32's power and give it straight back, WITHOUT going through
 * SM_ResumeSystemContext()/IDLE.
 *
 * Two reasons that routing matters, and both are easy to get wrong:
 *
 *   1. SM_HandleState_IDLE's entry sets wake_reason = SM_WAKE_NORMAL. The
 *      power-up path only asserts STM_MCU_IO1 when the reason is SM_WAKE_SETUP,
 *      and that pin is what tells the STM32 it is in Setup Mode. Lose it and
 *      the device comes back with no access point, so the operator who just
 *      pressed the update button never sees the result.
 *
 *   2. IDLE waits for SM_NeedsPeriodicSTMWake(), i.e. wake_interval_minutes.
 *      That is minutes of darkness for what should be under a second.
 *
 * So: clear entry_done and let SM_HandleState_POWER_STM re-run its own entry
 * block, which re-asserts IO1 from the wake reason we are deliberately leaving
 * untouched, powers the rail back up and re-arms the IO2 interrupt.
 */
static void SM_DoPowerCycle(void) {
    sm_context.power_cycle_pending = false;
    sm_context.power_cycle_armed   = false;

    uart_printf("[SM] Power-cycling STM32 (reason kept: %s)\n",
        (sm_context.wake_reason == SM_WAKE_SETUP) ? "SETUP" : "NORMAL");

    delay_cycles(SM_MS_CYCLES(SM_POWER_CYCLE_SETTLE_MS));

    SM_SetSTMPower(false);
    delay_cycles(SM_MS_CYCLES(SM_POWER_CYCLE_OFF_MS));

    /* Re-run the POWER_STM entry block on the next tick of the state machine.
     * Deliberately not SM_Transition(): we are already in this state and want
     * its entry actions repeated, not a state change. */
    sm_context.entry_done = false;
}

static void SM_HandleState_POWER_STM(void) {

    if (!sm_context.entry_done) {
        /* Clear any power-cycle request left over from a previous session in
         * this state. Without this, a request that was accepted and then
         * overtaken by an inactivity timeout or a shutdown would still be
         * pending next time the STM32 came up, and would fire on the first
         * staged response — a spurious power cycle with no one asking for it. */
        sm_context.power_cycle_pending = false;
        sm_context.power_cycle_armed   = false;

        PWR_EnterMeasureProfile();
        sm_context.total_wakes++;
        if (sm_context.wake_reason == SM_WAKE_SETUP) {
            DL_GPIO_setPins(DIGITAL_OUTPUT_PORTA_PORT, DIGITAL_OUTPUT_PORTA_STM_MCU_IO1_PIN);
        } else {
            DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTA_PORT, DIGITAL_OUTPUT_PORTA_STM_MCU_IO1_PIN);
        }
        RTC_EnablePrescaler();
        SM_SetSTMPower(true);
        sm_context.stm_power_on_s = sm_context.second_counter;
        sm_context.last_io2_activity_s = sm_context.second_counter;
        uart_printf("[SM] STM32 powered : reason: %s (wake #%lu)\n",
            (sm_context.wake_reason == SM_WAKE_SETUP) ? "SETUP" : "NORMAL",
            (unsigned long)sm_context.total_wakes);
        sm_context.entry_done = true;
        sm_context.stm_data_sent = false;
        DL_GPIO_disableInterrupt(EXTERNAL_INTERRUPT_CHARGER_INT_PORT, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
        DL_GPIO_enableInterrupt(EXTERNAL_INTERRUPT_STM_MCU_IO2_PORT, EXTERNAL_INTERRUPT_STM_MCU_IO2_PIN);
        stm_io2_flag = false;
    }
    if (stm_io2_flag) {
        sm_context.last_io2_activity_s = sm_context.second_counter;
        stm_io2_flag = false;
        stm32Spi.rxDone = false;
        if (sm_context.has_pending_response) {
            SPI_Controller_Arm(&stm32Spi);
            sm_context.has_pending_response = false;
            /* If that staged reply is the acknowledgement for a power-cycle
             * request, it is now on the wire. SPI_Controller_Arm() has just
             * cleared rxDone, so rxDone going true again means this transfer
             * finished and the STM32 has the answer. Only then may the rail
             * drop. */
            if (sm_context.power_cycle_pending) {
                sm_context.power_cycle_armed = true;
            }
        } else {
            sm_context.stm_data_sent = true;
            SM_SendOffer();
        }
    }
    /* Process STM32 reply and send next packet immediately */
    if (sm_context.stm_data_sent && stm32Spi.rxDone) {
        stm32Spi.rxDone = false;
        sm_context.stm_data_sent = false;
        uart_printf("\n Received from STM32:\n");
        for (int i = 0; i < 20; i++) {
            uart_printf("%02X ", stm32Spi.rxBuf[i]);
        }
        // delay_cycles(320000);
        SM_DispatchIncomingPacket();
     }

    /* Deferred power cycle. Reached only once the acknowledgement transfer
     * armed above has completed, so the STM32 knows the cut is coming.
     *
     * This sits after the dispatch block on purpose: that block only consumes
     * rxDone when stm_data_sent is set, which it is not for a staged response,
     * so the flag survives for us to test here. */
    if (sm_context.power_cycle_armed && stm32Spi.rxDone) {
        stm32Spi.rxDone = false;
        SM_DoPowerCycle();
        return;
    }

    /* Inactivity timeout — resets each time IO2 fires */
    if ((sm_context.second_counter - sm_context.last_io2_activity_s) >= SM_INACTIVITY_TIMEOUT_S) {
        sm_context.inactivity_timeouts++;
        uart_printf("[SM] STM32 inactivity timeout (#%lu)\n", (unsigned long)sm_context.inactivity_timeouts);
        SM_SetSTMPower(false);
        SM_ResumeSystemContext();
    }
}

static void SM_HandleState_IDLE(void) {

    if (!sm_context.entry_done) {
        /* This block does a full I2C read (SM_FetchPowerContext) and three
         * prints before it powers the bus down, so establish the profile
         * rather than trusting the transition we arrived through. */
        PWR_EnterMeasureProfile();
        sm_context.wake_reason = SM_WAKE_NORMAL;
        SM_SetSTMPower(false);
        sm_context.sleep_entry_minute = sm_context.minute_counter;
        DL_GPIO_disableInterrupt(EXTERNAL_INTERRUPT_STM_MCU_IO2_PORT, EXTERNAL_INTERRUPT_STM_MCU_IO2_PIN);
        DL_GPIO_clearInterruptStatus(GPIOB, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
        DL_GPIO_enableInterrupt(GPIOB, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
        hall_wakeup_flag = false;
        RTC_DisablePrescaler();
        SM_PowerContext_t pwr = SM_FetchPowerContext();
        if (pwr.vbat_mv < SM_VBAT_CHARGE_START_MV) {
            DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_CHARGER_EN_PIN);
            uart_printf("[SM] Charging in IDLE enabled\n");
        } else {
            DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_CHARGER_EN_PIN);
            uart_printf("[SM] Charging in IDLE disabled\n");
        }      
        uart_printf("[SM] Entering IDLE\n");
        sm_context.entry_done = true;
        PWR_ExitMeasureProfile();
    }
    if (sm_context.sleep_entry_minute != sm_context.minute_counter) {
        sm_context.sleep_entry_minute = sm_context.minute_counter;
        PWR_EnterMeasureProfile();
        SM_Heartbeat();
        if (SM_SafetyCheck()) return;
        SM_PowerContext_t pwr = SM_FetchPowerContext();
        if (pwr.is_charging) {
            SM_Transition(SM_STATE_CHARGING);
            return;
        }
        if (SM_NeedsPeriodicSTMWake(pwr)) {
            RTC_EnablePrescaler();
            sm_context.last_stm_periodic_minute = sm_context.minute_counter;
            SM_Transition(SM_STATE_POWER_STM);
            return;
        }
    }
    if (hall_wakeup_flag) {
        hall_wakeup_flag = false;
        sm_context.wake_reason = SM_WAKE_SETUP;
        RTC_EnablePrescaler();
        PWR_EnterMeasureProfile();
        SM_Transition(SM_STATE_POWER_STM);
        return;
    }
    PWR_ExitMeasureProfile();
    __WFI();
}

static void SM_HandleState_CRITICAL_FAULT(void) {
    if (!sm_context.entry_done) {
        PWR_EnterMeasureProfile();
        SM_SetSTMPower(false);
        DL_GPIO_disableInterrupt(GPIOB, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
        DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_CHARGER_EN_PIN);
        RTC_EnablePrescaler();
        const char* fault_str = "UNKNOWN";
        switch (sm_context.fault_source) {
            case SM_FAULT_I2C_BUS:      fault_str = "I2C_BUS"; break;
            case SM_FAULT_GAUGE:        fault_str = "GAUGE"; break;
            case SM_FAULT_CHARGER:      fault_str = "CHARGER"; break;
            case SM_FAULT_INIT_FAILED:  fault_str = "INIT_FAILED"; break;
            default: break;
        }
        uart_printf("[SM] CRITICAL FAULT, source: %s\n", fault_str);
        sm_context.fault_retry_s = sm_context.second_counter;

        if (sm_context.fault_source == SM_FAULT_GAUGE) {
            SM_DecodeBatterySafetyStatus(last_safety_status);
        } else if (sm_context.fault_source == SM_FAULT_CHARGER) {
            SM_DecodeChargingSafetyStatus(last_charger_status);
        } else if (sm_context.fault_source == SM_FAULT_I2C_BUS) {
            uart_printf("[SM] I2C bus fault. Retrying every %ds\n", SM_I2C_RETRY_S);
        }else if (sm_context.fault_source == SM_FAULT_INIT_FAILED) {
            uart_printf("[SM] Initialization failed, power cycle board\n");
        }
        sm_context.entry_done = true;
    }

    /* INIT_FAILED is unrecoverable, nothing to retry */
    if (sm_context.fault_source == SM_FAULT_INIT_FAILED) {
        PWR_ExitMeasureProfile();
        __WFI();
        return;
    }

    uint32_t elapsed = sm_context.second_counter - sm_context.fault_retry_s;
    uint32_t interval = (sm_context.fault_source == SM_FAULT_I2C_BUS) ? SM_I2C_RETRY_S : SM_FAULT_RETRY_S;

    if (elapsed >= interval) {
        sm_context.fault_retry_s = sm_context.second_counter;
        PWR_EnterMeasureProfile();

        /* I2C bus fault: just probe the addresses, go back to INIT if found */
        if (sm_context.fault_source == SM_FAULT_I2C_BUS) {
            uart_printf("[SM] Retrying I2C bus...\n");
            gauge_init();
            if (I2C_TryAddress(I2C_0_INST, GAUGE_I2C_ADDR) &&
                I2C_TryAddress(I2C_0_INST, BQ25628E_I2C_ADDR)) {
                uart_printf("[SM] I2C devices found, returning to INIT\n");
                SM_Transition(SM_STATE_INIT);
                return;     /* leave the bus powered — INIT needs it */
            }
            uart_printf("[SM] I2C still unavailable\n");
            /* Fall through to power down and sleep until the next retry.
             * Returning here would leave I2C0, UART0 and the fast clocks up
             * and skip the __WFI(), so a board with a dead I2C bus would spin
             * at full power until the battery died. */
        } else {
            /* Gauge / charger faults: read live status and clear if clean */
            BQ27Z746_GetSafetyStatus(I2C_0_INST, &last_safety_status);
            last_charger_status = BQ25628E_GetFaultFlags();

            if (last_safety_status == 0 && last_charger_status == 0) {
                uart_printf("[SM] All safety flags clear\n");
                SM_Transition(SM_STATE_IDLE);
                return;     /* leave the bus powered — IDLE entry reads it */
            }
            if (last_safety_status != 0) {
                uart_printf("[SM] Battery Fault Active: 0x%08X\n", (unsigned int)last_safety_status);
                SM_DecodeBatterySafetyStatus(last_safety_status);
            }
            if (last_charger_status != 0) {
                uart_printf("[SM] Charger Fault Active: 0x%02X\n", last_charger_status);
                SM_DecodeChargingSafetyStatus(last_charger_status);
            }
        }
    }

    PWR_ExitMeasureProfile();
    __WFI();
}

void SM_Run(void) {
    SM_Handle_RTC_Tick();
    switch (sm_context.current) {
        case SM_STATE_INIT:           SM_HandleState_INIT();           break;
        case SM_STATE_CHARGING:       SM_HandleState_CHARGING();       break;
        case SM_STATE_POWER_STM:      SM_HandleState_POWER_STM();      break;
        case SM_STATE_IDLE:           SM_HandleState_IDLE();           break;
        case SM_STATE_CRITICAL_FAULT: SM_HandleState_CRITICAL_FAULT(); break;
    }
}

bool SM_SafetyCheck(void) {
    if (sm_context.current == SM_STATE_CRITICAL_FAULT ||
        sm_context.current == SM_STATE_INIT) return false;

    /* The gauge lives on I2C0, which is gated off outside the MEASURE profile.
     * A stale rtc_second_tick can land us here with the bus unpowered; skip the
     * check rather than issue a transaction that cannot complete. */
    if (!g_i2c0_powered) return false;

    uint32_t safety = 0;
    BQ27Z746_GetSafetyStatus(I2C_0_INST, &safety);
    if (safety != 0) {
        last_safety_status = safety; 
        if (SM_ProcessFault(safety, 0)) {
            sm_context.fault_source = SM_FAULT_GAUGE;
            SM_Transition(SM_STATE_CRITICAL_FAULT);
            return true; 
        }
        return false;  
    }
    return false;
}

bool SM_ChargingSafetyCheck(void) {
    /* Charger is on the same gated I2C0 bus — see SM_SafetyCheck. */
    if (!g_i2c0_powered) return false;

    uint8_t fault_flags = BQ25628E_GetFaultFlags();
    if (fault_flags != 0U) {
        last_charger_status = fault_flags;
        if (SM_ProcessFault(0, fault_flags)) {
            sm_context.fault_source = SM_FAULT_CHARGER;
            SM_Transition(SM_STATE_CRITICAL_FAULT);
            return true; 
        }
        return false; 
    }
    return false;
}

static bool SM_NeedsPeriodicSTMWake(SM_PowerContext_t pwr)
{
    if (pwr.is_critical_low && sm_context.critical_msg_sent) return false;
    return (sm_context.minute_counter - sm_context.last_stm_periodic_minute) >= 
           sm_context.stm_wake_period.wake_interval_minutes;
}
static void SM_ResumeSystemContext(void) {
    SM_PowerContext_t pwr = SM_FetchPowerContext();
    if (pwr.is_charging) {
        uart_printf("[SM] Continuing to charge\n");
        SM_Transition(SM_STATE_CHARGING);
    } else if (pwr.is_critical_low) {
        uart_printf("[SM] Critical low battery detected, entering IDLE to conserve power\n");
        SM_Transition(SM_STATE_IDLE);
    } else {
        SM_Transition(SM_STATE_IDLE);
    }
}
/* ── Protocol helpers ───────────────────────────────────────────────────── */
static void SM_PrepareSimpleMsg(SM_MsgType_t type)
{
    SM_SpiPacket_t *pkt = (SM_SpiPacket_t *)stm32Spi.txBuf;
    memset(stm32Spi.txBuf, 0, stm32Spi.size);

    pkt->pkt.header.msg_type   = type;
    pkt->pkt.header.payload_id = 0;
    pkt->pkt.header.length     = 0;
}

static void SM_SendOffer(void) { 
    SM_PrepareSimpleMsg(MSG_OFFER);
    SPI_Controller_Arm(&stm32Spi);
}

static void SM_PrepareAck(void) { 
    SM_PrepareSimpleMsg(MSG_ACK); 
    sm_context.has_pending_response = true;
}

static void SM_PrepareNack(void) { 
    SM_PrepareSimpleMsg(MSG_NACK);  
    sm_context.has_pending_response = true;
}
static SM_SpiPacket_t* SM_InitDataPacket(SM_PayloadId_t pid) {
    SM_SpiPacket_t *pkt = (SM_SpiPacket_t *)stm32Spi.txBuf;
    memset(stm32Spi.txBuf, 0, stm32Spi.size);
    pkt->pkt.header.msg_type   = MSG_DATA;
    pkt->pkt.header.payload_id = pid;
    sm_context.has_pending_response = true;
    return pkt;
}

static void SM_PrepareTelemetryResponse(void)
{
    SM_PowerContext_t pwr = SM_FetchPowerContext();
    BQ27Z746_GetSafetyStatus(I2C_0_INST, &last_safety_status);

    uint8_t  chg_flags   = BQ25628E_ReadReg8(BQ25628E_REG_CHG_FLAG0);
    uint8_t  fault_flags = BQ25628E_ReadReg8(BQ25628E_REG_FAULT_FLAG0);

    uint16_t batt_status = BQ27Z746_Get_BatteryStatus();

    int btmp_dC = (int)(BQ25628E_Get_TBAT_C() * 10.0f);

    /* Report the state we came from, not POWER_STM which is always current here */
    const char* prev_state_str;
    switch (sm_context.previous) {
        case SM_STATE_INIT:           prev_state_str = "INIT";           break;
        case SM_STATE_CHARGING:       prev_state_str = "CHARGING";       break;
        case SM_STATE_POWER_STM:      prev_state_str = "POWER_STM";      break;
        case SM_STATE_IDLE:           prev_state_str = "IDLE";           break;
        case SM_STATE_CRITICAL_FAULT: prev_state_str = "CRITICAL_FAULT"; break;
        default:                      prev_state_str = "UNKNOWN";        break;
    }

    snprintf(json_buf, sizeof(json_buf),
        "{\"soc\":%d,\"soh\":%d,"
        "\"vbat\":%d,\"ibat\":%d,\"vchg\":%d,\"vsys\":%d,\"ichg\":%d,\"avgi\":%d,\"avgpwr\":%d,"
        "\"gtmp\":%d,\"ctmp\":%d,\"btmp\":%d,"
        "\"cycles\":%d,"
        "\"adapter\":%d,"
        "\"state\":\"%s\","
        "\"safety\":\"0x%08X\",\"battstat\":\"0x%04X\","
        "\"chgflags\":\"0x%02X\",\"faultflags\":\"0x%02X\","
        "\"chgstat\":\"%s\","
        "\"lowbattery\":%d,"
        "\"wake_interval\":%d,"
        "\"wakes\":%lu,\"inactivity_timeouts\":%lu,"
        "\"first_boot\":%d,"
        "\"vreg\":%d,\"cfg_ichg\":%d,\"iindpm\":%d,"
        "\"vindpm\":%d,\"vsysmin\":%d,\"iprechg\":%d,\"iterm\":%d}",
        BQ27Z746_Get_SOC_pct(), BQ27Z746_Get_StateOfHealth_pct(),
        pwr.vbat_mv, BQ27Z746_Get_Current_mA(), BQ25628E_Get_VBUS_mV(),
        BQ25628E_Get_VSYS_mV(), BQ25628E_Get_IBUS_mA(), BQ27Z746_Get_AvgCurrent_mA(),
        BQ27Z746_Get_AvgPower_mW(), (int)BQ27Z746_Get_InternalTemp_C(),
        BQ25628E_Get_TDIE_C(), btmp_dC,
        BQ27Z746_Get_CycleCount(),
        pwr.adapter_present ? 1 : 0, prev_state_str,
        (unsigned int)last_safety_status, batt_status, chg_flags, fault_flags,
        SM_GetChargeString(pwr.chg_stat), pwr.is_critical_low ? 1 : 0,
        sm_context.stm_wake_period.wake_interval_minutes,
        (unsigned long)sm_context.total_wakes,
        (unsigned long)sm_context.inactivity_timeouts,
        sm_context.first_boot ? 1 : 0,
        sm_context.sm_charger_config.vreg_mV,
        sm_context.sm_charger_config.ichg_mA,
        sm_context.sm_charger_config.iindpm_mA,
        sm_context.sm_charger_config.vindpm_mV,
        sm_context.sm_charger_config.vsysmin_mV,
        sm_context.sm_charger_config.iprechg_mA,
        sm_context.sm_charger_config.iterm_mA);

    sm_context.first_boot = false;

    SM_SpiPacket_t *pkt = SM_InitDataPacket(PID_TELEMETRY);
    size_t len = strlen(json_buf);
    if (len > sizeof(pkt->pkt.payload.telemetry.json))
        len = sizeof(pkt->pkt.payload.telemetry.json);
    pkt->pkt.header.length     = (uint16_t)len;
    // uart_printf("[SM] Telemetry response: %s\n", json_buf);
    memcpy(pkt->pkt.payload.telemetry.json, json_buf, len);

    if(pwr.is_critical_low) {
        sm_context.critical_msg_sent = true;
        uart_printf("[SM] Critical low battery detected, sending last packet till charge\n");
    }
}

static void SM_PrepareRTCResponse(void)
{
    RTC_GetTime(&sm_context.sm_rtc_config);
    SM_SpiPacket_t *pkt = SM_InitDataPacket(PID_RTC_GET);
    pkt->pkt.header.length     = sizeof(SM_RTCConfig_t);
    pkt->pkt.payload.rtc_data = sm_context.sm_rtc_config;
}

static void SM_HandleRequest(uint8_t pid)
{
    if (pid == PID_TELEMETRY) {
        SM_PrepareTelemetryResponse();
    } else if (pid == PID_RTC_GET) {
        SM_PrepareRTCResponse();
    } else if (pid == PID_STM_CFG) {
        SM_PrepareSTMConfigResponse();
    } else if (pid == PID_STM_CREDENTIALS) {
        SM_PrepareSTMCredentialsResponse();
    } else {
        SM_PrepareNack();
    }
}

static void SM_HandleConfig(uint8_t pid, const void *payload)
{
    if (pid == PID_CHARGER_CFG) {
        const SM_ChargerConfig_t *cfg = (const SM_ChargerConfig_t *)payload;
        BQ25628E_ApplyProfile(cfg);
        sm_context.sm_charger_config = *cfg;
        sm_context.charger_configured = true;
        SM_PrepareAck();
    } else if (pid == PID_RTC_SET) {
        const SM_RTCConfig_t *cfg = (const SM_RTCConfig_t *)payload;
        if (RTC_SetTime(cfg)) {
            sm_context.sm_rtc_config = *cfg;
            SM_PrepareAck();
        } else {
            SM_PrepareNack();
        }
    } else if (pid == PID_PERIOD_SET){
        const SM_PeriodConfig_t *cfg = (const SM_PeriodConfig_t *)payload;
        sm_context.stm_wake_period = *cfg;
        sm_context.wake_interval_configured = true;
        SM_PrepareAck();
    } else if (pid == PID_KEEP_ALIVE) {
        sm_context.last_io2_activity_s = sm_context.second_counter;
        SM_PrepareAck();
        uart_printf("[SM] Keep-alive received: resetting inactivity timer\n");
    } else if (pid == PID_POWER_CYCLE) {
        /* Do NOT cut power here. SM_PrepareAck() only stages the reply; it is
         * transmitted on the next transfer the STM32 initiates. Dropping the
         * rail now would kill it mid-request, and it would never learn that we
         * agreed. SM_HandleState_POWER_STM performs the cut once the reply has
         * actually gone out — see SM_DoPowerCycle(). */
        sm_context.last_io2_activity_s = sm_context.second_counter;
        sm_context.power_cycle_pending = true;
        SM_PrepareAck();
        uart_printf("[SM] Power-cycle requested by STM32 (deferred until ack sent)\n");
    }
    else if (pid == PID_STM_CFG) {
        const SM_STMConfig_t *cfg = (const SM_STMConfig_t *)payload;
        sm_context.stm_config = *cfg;
        sm_context.stm_config_received = true;
        uart_printf("[SM] STM config updated: conn=%d lte_baud=%d cam_res=%d\n",
            cfg->connectivity.mode,
            cfg->lte.baudrate_index,
            cfg->camera.resolution);
        SM_PrepareAck();
    } else if (pid == PID_STM_CREDENTIALS) {
        const SM_STMCredentials_t *creds = (const SM_STMCredentials_t *)payload;
        memcpy(&sm_context.stm_credentials, creds, sizeof(SM_STMCredentials_t));
        sm_context.stm_credentials_received = true;
        uart_printf("[SM] Credentials updated: ssid=%s device=%s\n",
            sm_context.stm_credentials.ap_ssid,
            sm_context.stm_credentials.device_name);
        SM_PrepareAck();
    } else {
        SM_PrepareNack();
    }
}

static void SM_PrepareSTMConfigResponse(void)
{
    SM_SpiPacket_t *pkt = SM_InitDataPacket(PID_STM_CFG);
    pkt->pkt.header.length     = sizeof(SM_STMConfig_t);
    pkt->pkt.payload.stm_config = sm_context.stm_config;

    uart_printf("[SM] STM config response sent (defaults: %s)\n",
        sm_context.stm_config_received ? "no" : "yes");
}

static void SM_PrepareSTMCredentialsResponse(void)
{
    SM_SpiPacket_t *pkt = SM_InitDataPacket(PID_STM_CREDENTIALS);
    pkt->pkt.header.length     = sizeof(SM_STMCredentials_t);
    pkt->pkt.payload.stm_credentials = sm_context.stm_credentials;

    uart_printf("[SM] Credentials response sent (defaults: %s)\n",
        sm_context.stm_credentials_received ? "no" : "yes");
}

static void SM_DispatchIncomingPacket(void)
{
    SM_MsgHeader_t *hdr = (SM_MsgHeader_t *)stm32Spi.rxBuf;

    switch (hdr->msg_type) {
        case MSG_REQUEST:
            SM_HandleRequest(hdr->payload_id);
            break;

        case MSG_CONFIG:
            SM_HandleConfig(hdr->payload_id, stm32Spi.rxBuf + sizeof(SM_MsgHeader_t));
            break;

        case MSG_SHUTDOWN:
            uart_printf("[SM] STM32 requested shutdown\n");
            SM_SetSTMPower(false);
            SM_ResumeSystemContext();
            return;   

        default:
            SM_PrepareNack();
            break;
    }
}

static void SM_DecodeBatterySafetyStatus(uint32_t status) {
    if (status & BQ27Z746_SAFETY_CUV)  uart_printf("[FAULT] CUV  : Cell Undervoltage\n");
    if (status & BQ27Z746_SAFETY_OVP)  uart_printf("[FAULT] COV  : Cell Overvoltage\n");
    if (status & BQ27Z746_SAFETY_OCC)  uart_printf("[FAULT] OCC  : Overcurrent During Charge\n");
    if (status & BQ27Z746_SAFETY_OCD)  uart_printf("[FAULT] OCD  : Overcurrent During Discharge\n");
    if (status & BQ27Z746_SAFETY_HOCD) uart_printf("[FAULT] HOCD : Overload During Discharge\n");
    if (status & BQ27Z746_SAFETY_HOCC) uart_printf("[FAULT] HOCC : Short-Circuit During Charge\n");
    if (status & BQ27Z746_SAFETY_SCD)  uart_printf("[FAULT] HSCD : Hardware Short-Circuit Discharge\n");
    if (status & BQ27Z746_SAFETY_OTC)  uart_printf("[FAULT] OTC  : Over-Temperature During Charge\n");
    if (status & BQ27Z746_SAFETY_OTD)  uart_printf("[FAULT] OTD  : Over-Temperature During Discharge\n");
    if (status & BQ27Z746_SAFETY_OTF)  uart_printf("[FAULT] OTF  : Over-Temperature FET\n");
    if (status & BQ27Z746_SAFETY_PTO)  uart_printf("[FAULT] PTO  : Precharge Timeout\n");
    if (status & BQ27Z746_SAFETY_CTO)  uart_printf("[FAULT] CTO  : Charge Timeout\n");
    if (status & BQ27Z746_SAFETY_UTC)  uart_printf("[FAULT] UTC  : Under-Temperature During Charge\n");
    if (status & BQ27Z746_SAFETY_UTD)  uart_printf("[FAULT] UTD  : Under-Temperature During Discharge\n");
    if (status & BQ27Z746_SAFETY_HCOV) uart_printf("[FAULT] HCOV : Hardware Cell Overvoltage\n");
    if (status & BQ27Z746_SAFETY_HCUV) uart_printf("[FAULT] HCUV : Hardware Cell Undervoltage\n");
}

static void SM_DecodeChargingSafetyStatus(uint8_t status) {
    if (status & BQ25628E_VBUS_FAULT_FLAG) uart_printf("  [!] VBUS: Over-Voltage or Sleep detected\n");
    if (status & BQ25628E_BAT_FAULT_FLAG)  uart_printf("  [!] BAT: Discharge OCP or VBAT OVP\n");
    if (status & BQ25628E_SYS_FAULT_FLAG) uart_printf("  [!] SYS: System Over-Voltage or Short Circuit\n");
    if (status & BQ25628E_TSHUT_FLAG) uart_printf(" [!] THERMAL: IC Thermal Shutdown triggered\n");
    if (status & BQ25628E_TS_FLAG) uart_printf("  [i] TS: Temperature status change detected\n");
}


void RTC_GetTime(SM_RTCConfig_t *out)
{
    DL_RTC_Calendar calendar;

    calendar = DL_RTC_getCalendarTime(RTC);

    out->second = calendar.seconds;
    out->minute = calendar.minutes;
    out->hour   = calendar.hours;
    out->day    = calendar.dayOfMonth;
    out->month  = calendar.month;
    out->year = calendar.year; 

}

bool RTC_SetTime(const SM_RTCConfig_t *in)
{
    DL_RTC_Calendar calendar;

    if (in->second > 59 || in->minute > 59 || in->hour > 23 ||
        in->month < 1 || in->month > 12 ||
        in->day < 1 || in->day > 31 ||
        in->year < 2000 || in->year > 2099)
    {
        return false;
    }

    calendar.seconds    = in->second;
    calendar.minutes    = in->minute;
    calendar.hours      = in->hour;
    calendar.dayOfMonth = in->day;
    calendar.month      = in->month;
    calendar.year       = in->year;
    calendar.dayOfWeek  = 1;

    DL_RTC_initCalendar(RTC, calendar, DL_RTC_FORMAT_BINARY);
    return true;
}

static bool SM_ProcessFault(uint32_t gauge_safety, uint8_t charger_fault)
{
    bool is_critical = false;
    bool disable_charging = false;
    if (gauge_safety != 0) {
        SM_DecodeBatterySafetyStatus(gauge_safety);
        if (gauge_safety & (BQ27Z746_SAFETY_HOCC  |
                            BQ27Z746_SAFETY_SCD   |
                            BQ27Z746_SAFETY_HOCD)) {
            is_critical = true;
        }
        /* Recoverable faults : just disable charging, stay in current state */
        if (gauge_safety & (BQ27Z746_SAFETY_OVP   | BQ27Z746_SAFETY_HCOV |   // Overvoltage
                                 BQ27Z746_SAFETY_OCC                           |   // Overcurrent charge
                                 BQ27Z746_SAFETY_PTO  | BQ27Z746_SAFETY_CTO    |   // Timeouts
                                 BQ27Z746_SAFETY_OTC  | BQ27Z746_SAFETY_UTC)) {    // Temperature charge
            disable_charging = true;
        }
        /* CUV/HCUV, OCD/HOCD, OTD, UTC/UTD : log only (CUV already prevented by SM_VBAT_LOW_MV) */
    }

    /* ── Charger faults (using your exact defines) ──────────── */
    if (charger_fault != 0) {
        SM_DecodeChargingSafetyStatus(charger_fault);

        /* Critical faults */
        if (charger_fault & (BQ25628E_SYS_FAULT_FLAG | BQ25628E_TSHUT_FLAG)) {
            is_critical = true;
        }
        /* VBUS_FAULT_FLAG : only real fault if adapter is actually present */
        if (charger_fault & BQ25628E_VBUS_FAULT_FLAG) {
            SM_PowerContext_t pwr = SM_FetchPowerContext();
            if (pwr.adapter_present) {
                disable_charging = true;
                uart_printf("[SM] VBUS fault while adapter present : charging disabled\n");
            } else {
                uart_printf("[SM] VBUS sleep (no adapter) : ignored\n");
            }
        }
        /* BAT_FAULT_FLAG : recoverable */
        if (charger_fault & BQ25628E_BAT_FAULT_FLAG) {
            disable_charging = true;
        }
        /* TS_FLAG : completely ignored (as agreed) */
    }

    /* Apply the recoverable action */
    if (disable_charging && !is_critical) {
        DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_CHARGER_EN_PIN);
        uart_printf("[SM] Recoverable fault : charging disabled (system continues running)\n");
    }

    return is_critical;
}