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
#define SM_SLEEP_WAKEUP_MINUTES   15   
#define SM_VBAT_LOW_MV            3000 
#define SM_VBAT_FULL_MV           3650
#define SM_VBAT_CHARGE_START_MV   3500
#define SM_SAFETY_POLL_S                1
#define SM_ADAPTER_DEBOUNCE_S           10
#define SM_SETUP_INACTIVITY_TIMEOUT_S   120
#define SM_NORMAL_INACTIVITY_TIMEOUT_S  60
/* ── Global Context ──────────────────────────────────────── */
SM_Context_t sm_context;
extern volatile bool rtc_minute_tick;
extern volatile bool rtc_second_tick;
extern volatile bool hall_wakeup_flag;
extern volatile bool stm_io2_flag;
extern SPI_Controller_Handle stm32Spi;
extern volatile bool adapter_check_flag;

/* ── Static variables ────────────────────────────────────── */
static uint32_t last_safety_status = 0;
static char json_buf[512];

/* ── Internal Prototypes ─────────────────────────────────── */
static void SM_Handle_RTC_Tick(void);
static void SM_DecodeSafetyStatus(uint32_t status);
static bool SM_NeedsPeriodicSTMWake(void);
static void SM_ReturnFromPowerSTM(void) ;
void SM_SafetyCheck(void);

void SM_Init(void) {
    memset(&sm_context, 0, sizeof(SM_Context_t));
    sm_context.current = SM_STATE_INIT;
    sm_context.sm_paused = false;
    sm_context.entry_done = false;
     sm_context.last_io2_activity_s = 0;
     sm_context.last_stm_periodic_minute = 0;
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
        case SM_STATE_IDLE:          return "IDLE";
        case SM_STATE_CRITICAL_FAULT: return "CRITICAL_FAULT";
        default:                      return "UNKNOWN";
    }
}

const char* SM_GetChargeString( uint8_t chg_stat) {
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

static bool SM_IsCriticalLowBattery(void) {
    BQ27Z746_UpdateTelemetry(I2C_0_INST);
    uint16_t vbat_mv = BQ27Z746_Get_Voltage_mV();
    uint8_t stat1 = BQ25628E_ReadReg8(BQ25628E_REG_STAT1);
    bool adapter_present = (stat1 & BQ25628E_VBUS_STAT_MASK) != 0;
    return (vbat_mv < SM_VBAT_LOW_MV) && !adapter_present;
}

static void SM_PostWake_Branch(void) {
    if (SM_IsCriticalLowBattery()) {
        SM_Transition(SM_STATE_IDLE);      
        return;
    }

    BQ27Z746_UpdateTelemetry(I2C_0_INST);
    uint16_t vbat_mv = BQ27Z746_Get_Voltage_mV();
    uint8_t stat1 = BQ25628E_ReadReg8(BQ25628E_REG_STAT1);
    bool adapter_present = (stat1 & BQ25628E_VBUS_STAT_MASK) != 0;

    if (vbat_mv < SM_VBAT_CHARGE_START_MV) {
        SM_Transition(SM_STATE_CHARGING);
    } else {
        SM_Transition(SM_STATE_POWER_STM);
    }
}
void hall_init(void) {
    DL_GPIO_setPins(DIGITAL_OUTPUT_PORTA_PORT, DIGITAL_OUTPUT_PORTA_HALL_3V_PIN);
    delay_cycles(1000);
    DL_GPIO_clearInterruptStatus(GPIOB, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
    DL_GPIO_enableInterrupt(GPIOB, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
}

void SM_EnablePrescaler(void) {
    DL_RTC_enableInterrupt(RTC, DL_RTC_INTERRUPT_PRESCALER1);
}

void SM_DisablePrescaler(void) {
    DL_RTC_disableInterrupt(RTC, DL_RTC_INTERRUPT_PRESCALER1);
}

static void SM_Handle_RTC_Tick(void) {
    if (!rtc_minute_tick && !rtc_second_tick) return;
    if (rtc_second_tick) {
        rtc_second_tick = false;
        sm_context.second_counter++;
        if ((sm_context.second_counter - sm_context.last_safety_check_s) >= SM_SAFETY_POLL_S){
            sm_context.last_safety_check_s = sm_context.second_counter;
            SM_SafetyCheck();
            }
    }
    if (rtc_minute_tick) {
        rtc_minute_tick = false;
        sm_context.minute_counter++;
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
                    delay_cycles(3200);
                    DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_GAUGE_EN_PIN);
                    delay_cycles(3200);
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
                DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_CHARGER_EN_PIN);
                uart_printf("[SM] Charging started\n");
                sm_context.entry_done = true;
                SM_DisablePrescaler();
            }
            if (hall_wakeup_flag) {
                hall_wakeup_flag = false;
                sm_context.wake_reason = SM_WAKE_SETUP;
                SM_EnablePrescaler();
                SM_Transition(SM_STATE_POWER_STM);
                break;
            }
            static uint32_t last_charging_tick = 0;
            if (sm_context.minute_counter != last_charging_tick) {
                last_charging_tick = sm_context.minute_counter;

                BQ25628E_UpdateTelemetry();
                BQ27Z746_UpdateTelemetry(I2C_0_INST);
                BQ27Z746_GetSafetyStatus(I2C_0_INST, &last_safety_status);
                uint16_t vbat_mv = BQ27Z746_Get_Voltage_mV();

                uint8_t stat1         = BQ25628E_ReadReg8(BQ25628E_REG_STAT1);
                bool adapter_present  = (stat1 & BQ25628E_VBUS_STAT_MASK) != 0;
                uint8_t chg_stat      = (stat1 >> 3) & 0x03;  // CHG_STAT[4:3]

                bool charger_done = (chg_stat == 0 || chg_stat == 3);
                if (vbat_mv >= SM_VBAT_FULL_MV || charger_done) {
                    DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_CHARGER_EN_PIN);
                    uart_printf("[SM] Charging complete: VBAT %dmV >= %dmV\n", vbat_mv, SM_VBAT_FULL_MV);
                    SM_Transition(SM_STATE_IDLE);
                    break;
                }else {
                    const char* desc = SM_GetChargeString(chg_stat);
                    uart_printf("[SM] CHARGING | VBUS:%4dmV VBAT:%4dmV IBAT:%4dmA SOC:%3d%% TBAT:%3.1fC TDIE:%3dC CHG_STAT:%s\n",
                        BQ25628E_Get_VBUS_mV(),
                        BQ27Z746_Get_Voltage_mV(),
                        BQ27Z746_Get_Current_mA(),
                        BQ27Z746_Get_SOC_pct(),
                        BQ25628E_Get_TBAT_C(), 
                        BQ25628E_Get_TDIE_C(),
                        desc);
                }
                if (SM_NeedsPeriodicSTMWake()) {
                    SM_EnablePrescaler();
                    SM_Transition(SM_STATE_POWER_STM);
                    break;
                }
            }
            __WFI();
             break;
        }

        case SM_STATE_POWER_STM: {
            if (!sm_context.entry_done) {
                // Set IO1 based on wake reason
                if (sm_context.wake_reason == SM_WAKE_SETUP) {
                    DL_GPIO_setPins(DIGITAL_OUTPUT_PORTA_PORT, DIGITAL_OUTPUT_PORTA_STM_MCU_IO1_PIN);
                    sm_context.last_io2_activity_s = sm_context.second_counter;
                } else {
                    DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTA_PORT, DIGITAL_OUTPUT_PORTA_STM_MCU_IO1_PIN);
                }
                SM_EnablePrescaler();
                sm_context.last_stm_periodic_minute = sm_context.minute_counter;
                // Power on STM32
                DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_EN3V8_PIN);
                DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_STM_PON_PIN);
                sm_context.stm_power_on_s = sm_context.second_counter;
                
                uart_printf("[SM] STM32 powered : reason: %s\n", 
                            (sm_context.wake_reason == SM_WAKE_SETUP) ? "SETUP" : "NORMAL");
                
                sm_context.entry_done = true;
                sm_context.stm_data_sent = false;
                stm_io2_flag = false;
                
                DL_GPIO_disableInterrupt(EXTERNAL_INTERRUPT_CHARGER_INT_PORT, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
                DL_GPIO_enableInterrupt(EXTERNAL_INTERRUPT_STM_MCU_IO2_PORT, EXTERNAL_INTERRUPT_STM_MCU_IO2_PIN);
            }
            
            // Handle IO2 data request
            if (stm_io2_flag && !sm_context.stm_data_sent) {
                SM_SendTelemetryToSTM32();
                
                if (sm_context.wake_reason == SM_WAKE_SETUP) {
                    sm_context.last_io2_activity_s = sm_context.second_counter;  // Refresh inactivity timer
                }
                
                stm_io2_flag = false;
                sm_context.stm_data_sent = true;
            }
            
            // Check SPI completion and STM32 command
            if (sm_context.stm_data_sent && stm32Spi.rxDone) {
                stm32Spi.rxDone = false;
                uint8_t stm32_cmd = stm32Spi.rxBuf[0];
                
                if (stm32_cmd == STM32_CMD_SHUTDOWN) {
                    uart_printf("[SM] STM32 requested shutdown\n");
                    DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_STM_PON_PIN);
                    DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTA_PORT, DIGITAL_OUTPUT_PORTA_STM_MCU_IO1_PIN);
                    SM_ReturnFromPowerSTM();
                    break;
                }
                sm_context.stm_data_sent = false;
            }
            
            if (sm_context.wake_reason == SM_WAKE_NORMAL) {
                if ((sm_context.second_counter - sm_context.stm_power_on_s) >= SM_NORMAL_INACTIVITY_TIMEOUT_S) {
                    uart_printf("[SM] STM32 timeout (NORMAL mode)\n");
                    SM_ReturnFromPowerSTM();
                }
            } else {
                uint32_t inactive_s = sm_context.second_counter - sm_context.last_io2_activity_s;
                if (inactive_s >= SM_SETUP_INACTIVITY_TIMEOUT_S) {
                    uart_printf("[SM] STM32 inactivity timeout (SETUP mode)\n");
                    DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTA_PORT, DIGITAL_OUTPUT_PORTA_STM_MCU_IO1_PIN);
                    SM_ReturnFromPowerSTM();
                }
            }
            break;
        }

        case SM_STATE_IDLE: {
            if (!sm_context.entry_done) {
                sm_context.wake_reason = SM_WAKE_NORMAL;
                DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_STM_PON_PIN);
                DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_EN3V8_PIN);
                sm_context.sleep_entry_tick = sm_context.minute_counter;

                DL_GPIO_disableInterrupt(EXTERNAL_INTERRUPT_STM_MCU_IO2_PORT, EXTERNAL_INTERRUPT_STM_MCU_IO2_PIN);
                DL_GPIO_clearInterruptStatus(GPIOB, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
                DL_GPIO_enableInterrupt(GPIOB, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
                hall_wakeup_flag = false;
                SM_DisablePrescaler();
                uart_printf("[SM] Entering IDLE\n");
                sm_context.entry_done = true;
            }
            if (sm_context.sleep_entry_tick != sm_context.minute_counter) {
                BQ27Z746_UpdateTelemetry(I2C_0_INST);
                uint16_t vbat_mv = BQ27Z746_Get_Voltage_mV();
                SM_SafetyCheck();

                if (vbat_mv < SM_VBAT_CHARGE_START_MV) {
                    SM_EnablePrescaler();
                    SM_Transition(SM_STATE_CHARGING);
                    break;
                }

                if (SM_NeedsPeriodicSTMWake()) {
                    SM_EnablePrescaler();
                    SM_Transition(SM_STATE_POWER_STM);
                    break;
                }
            }

            if (hall_wakeup_flag) {
                hall_wakeup_flag = false;
                sm_context.wake_reason = SM_WAKE_SETUP;
                SM_EnablePrescaler();
                SM_PostWake_Branch();
                break;
            }

            __WFI();
            break;
        }

        case SM_STATE_CRITICAL_FAULT: {
            if (!sm_context.entry_done) {
                DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_STM_PON_PIN);
                DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_EN3V8_PIN);
                DL_GPIO_disableInterrupt(GPIOB, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
                SM_EnablePrescaler();
                const char* fault_str = "UNKNOWN";
                switch (sm_context.fault_source) {
                    case SM_FAULT_I2C_BUS:      fault_str = "I2C_BUS"; break;
                    case SM_FAULT_GAUGE:        fault_str = "GAUGE"; break;
                    case SM_FAULT_LOW_BATTERY:  fault_str = "LOW_BATTERY"; break;
                    case SM_FAULT_INIT_FAILED:  fault_str = "INIT_FAILED"; break;
                    default: break;
                }
                uart_printf("[SM] CRITICAL FAULT, source: %s\n", fault_str);
                sm_context.fault_retry_s = sm_context.second_counter;
                if (sm_context.fault_source == SM_FAULT_I2C_BUS) {
                    uart_printf("[SM] I2C bus failure can't initialize system(Power cycle system)\n");
                } else {
                    SM_DecodeSafetyStatus(last_safety_status);
                }
                sm_context.entry_done = true;
            }

            if (sm_context.fault_source != SM_FAULT_I2C_BUS &&
                (sm_context.second_counter - sm_context.fault_retry_s) >= 5UL) {
                sm_context.fault_retry_s = sm_context.second_counter;
                BQ27Z746_GetSafetyStatus(I2C_0_INST, &last_safety_status);

                if (sm_context.fault_source == SM_FAULT_LOW_BATTERY) {
                    uint8_t stat1 = BQ25628E_ReadReg8(BQ25628E_REG_STAT1);
                    bool adapter_present = (stat1 & BQ25628E_VBUS_STAT_MASK) != 0;
                    if (adapter_present) {
                        uart_printf("[SM] Adapter detected : resuming charge\n");
                        SM_PostWake_Branch();
                    } else {
                        uart_printf("[SM] Low battery : Please connect an adapter\n");
                        SM_Transition(SM_STATE_IDLE);
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


void SM_SafetyCheck(void) {
    if (sm_context.current == SM_STATE_CRITICAL_FAULT ||
        sm_context.current == SM_STATE_INIT) return;
    uint32_t safety = 0;
    BQ27Z746_GetSafetyStatus(I2C_0_INST, &safety);
    if (safety != 0) {
        sm_context.fault_source = SM_FAULT_GAUGE;
        SM_Transition(SM_STATE_CRITICAL_FAULT);
        return;
    }
}

static bool SM_NeedsPeriodicSTMWake(void) {
    return (sm_context.minute_counter - sm_context.last_stm_periodic_minute) >= SM_SLEEP_WAKEUP_MINUTES;
}

static void SM_ReturnFromPowerSTM(void) {
    uart_printf("[SM] STM32 session done — resuming background state\n");

    if (SM_IsCriticalLowBattery()) {
        uart_printf("[SM] Critical low battery detected — entering stable IDLE\n");
        SM_Transition(SM_STATE_IDLE);
        return;
    }

    BQ27Z746_UpdateTelemetry(I2C_0_INST);
    uint16_t vbat_mv = BQ27Z746_Get_Voltage_mV();
    uint8_t stat1 = BQ25628E_ReadReg8(BQ25628E_REG_STAT1);
    bool adapter_present = (stat1 & BQ25628E_VBUS_STAT_MASK) != 0;
    uint8_t chg_stat = (stat1 >> 3) & 0x03;
    bool charger_done = (chg_stat == 0 || chg_stat == 3);

    if (vbat_mv < SM_VBAT_CHARGE_START_MV || !charger_done) {
        SM_Transition(SM_STATE_CHARGING);
    } else {
        SM_Transition(SM_STATE_IDLE);
    }
}

void SM_SendTelemetryToSTM32(void) {
    BQ25628E_UpdateTelemetry();
    BQ27Z746_UpdateTelemetry(I2C_0_INST);
    BQ27Z746_GetSafetyStatus(I2C_0_INST, &last_safety_status);

    uint8_t  stat1       = BQ25628E_ReadReg8(BQ25628E_REG_STAT1); 
    bool     adapter     = (stat1 & BQ25628E_VBUS_STAT_MASK) != 0;
    uint8_t  chg_stat    = (stat1 >> 3) & 0x03;
    const char* desc     = SM_GetChargeString(chg_stat);
    uint8_t  chg_flags   = BQ25628E_ReadReg8(BQ25628E_REG_CHG_FLAG0);
    uint8_t  fault_flags = BQ25628E_ReadReg8(BQ25628E_REG_FAULT_FLAG0);

    uint16_t batt_status = BQ27Z746_Get_BatteryStatus();
    uint16_t tte         = BQ27Z746_Get_TimeToEmpty_min();
    uint16_t ttf         = BQ27Z746_Get_TimeToFull_min();
    uint16_t cycles      = BQ27Z746_Get_CycleCount();
    uint16_t vsys        = BQ25628E_Get_VSYS_mV();
    int16_t  avg_i       = BQ27Z746_Get_AvgCurrent_mA();
    int16_t  avg_pwr     = BQ27Z746_Get_AvgPower_mW(); 

    int tte_json = (tte == 0xFFFFu) ? -1 : (int)tte;
    int ttf_json = (ttf == 0xFFFFu) ? -1 : (int)ttf;

    const char* sm_state_str = SM_GetStateString();
    const char* wake_str     = (sm_context.wake_reason == SM_WAKE_SETUP) ? "SETUP" : "NORMAL";

    snprintf(json_buf, sizeof(json_buf),
        "{\"soc\":%d,\"soh\":%d,"
        "\"vbat\":%d,\"ibat\":%d,\"vchg\":%d,\"vsys\":%d,\"ichg\":%d,\"avgi\":%d,\"avgpwr\":%d,"
        "\"gtmp\":%d,\"ctmp\":%d,\"btmp\":%.1f,"
        "\"tte\":%d,\"ttf\":%d,\"cycles\":%d,"
        "\"adapter\":%d,"
        "\"state\":\"%s\",\"wake\":\"%s\","
        "\"safety\":\"0x%08X\",\"battstat\":\"0x%04X\","
        "\"chgflags\":\"0x%02X\",\"faultflags\":\"0x%02X\","
        "\"chgstat\":\"%s\","
        "\"lowbattery\":%d}",                     // ← NEW FIELD
        BQ27Z746_Get_SOC_pct(),
        BQ27Z746_Get_StateOfHealth_pct(),
        BQ27Z746_Get_Voltage_mV(),
        BQ27Z746_Get_Current_mA(),
        BQ25628E_Get_VBUS_mV(),
        vsys,
        BQ25628E_Get_IBUS_mA(),
        avg_i,
        avg_pwr,  
        (int)BQ27Z746_Get_InternalTemp_C(), 
        BQ25628E_Get_TDIE_C(),
        BQ25628E_Get_TBAT_C(),      
        tte_json,
        ttf_json,
        cycles,
        adapter ? 1 : 0,
        sm_state_str,
        wake_str,
        (unsigned int)last_safety_status,
        batt_status,
        chg_flags,
        fault_flags,
        desc,
        SM_IsCriticalLowBattery() ? 1 : 0        // ← NEW VALUE
    );
    
    size_t len = strlen(json_buf);
    memset(stm32Spi.txBuf, 0, stm32Spi.size);
    memcpy(stm32Spi.txBuf, json_buf, (len < stm32Spi.size) ? len : stm32Spi.size);
    
    SPI_Controller_Arm(&stm32Spi);
}

static void SM_DecodeSafetyStatus(uint32_t status) {
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
