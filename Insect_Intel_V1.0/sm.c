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
#define SM_VBAT_CHARGE_START_MV   3400
#define SM_SAFETY_POLL_S          5
#define SM_ADAPTER_DEBOUNCE_S     10
#define SM_SETUP_INACTIVITY_TIMEOUT_S   120
#define SM_NORMAL_INACTIVITY_TIMEOUT_S  60
#define SM_I2C_RETRY_S   10

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


/* ── Hardware Abstraction Helpers ────────────────────────── */

// Replaces repetitive GPIO calls for STM32 power
static void SM_SetSTMPower(bool enable) {
    if (enable) {
        DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_EN3V8_PIN | DIGITAL_OUTPUT_PORTB_STM_PON_PIN);
    } else {
        DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_EN3V8_PIN | DIGITAL_OUTPUT_PORTB_STM_PON_PIN);
        DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTA_PORT, DIGITAL_OUTPUT_PORTA_STM_MCU_IO1_PIN);
    }
}

// Single source of truth for current power state
static SM_PowerContext_t SM_FetchPowerContext(void) {
    BQ25628E_UpdateTelemetry();
    BQ25628E_ADC_Control(true);
    SM_PowerContext_t ctx;
    BQ27Z746_UpdateTelemetry(I2C_0_INST);
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

    /* Charger defaults */
    sm_context.sm_charger_config = (SM_ChargerConfig_t){
        .vreg_mV     = BQ_INIT_VREG_MV,
        .ichg_mA     = BQ_INIT_ICHG_MA,
        .iindpm_mA   = BQ_INIT_IINDPM_MA,
        .vindpm_mV   = BQ_INIT_VINDPM_MV,
        .vsysmin_mV  = BQ_INIT_VSYSMIN_MV,
        .iprechg_mA  = BQ_INIT_IPRECHG_MA,
        .iterm_mA    = BQ_INIT_ITERM_MA
    };
    RTC_GetTime(&sm_context.sm_rtc_config);  
    sm_context.sm_rtc_config.wake_interval_minutes = SM_SLEEP_WAKEUP_MINUTES;
    sm_context.charger_configured = false;
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
        SM_Transition(SM_STATE_POWER_STM);
    }
}

void hall_init(void) {
    DL_GPIO_setPins(DIGITAL_OUTPUT_PORTA_PORT, DIGITAL_OUTPUT_PORTA_HALL_3V_PIN);
    delay_cycles(1000);
    DL_GPIO_clearInterruptStatus(GPIOB, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
}

void gauge_init(void) {
    DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_GAUGE_EN_PIN);
    delay_cycles(320000);
}

void SM_EnablePrescaler(void) {
    DL_RTC_enableInterrupt(RTC, DL_RTC_INTERRUPT_PRESCALER1);
}

void SM_DisablePrescaler(void) {
    DL_RTC_disableInterrupt(RTC, DL_RTC_INTERRUPT_PRESCALER1);
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

void SM_Run(void) {
    SM_Handle_RTC_Tick();
    switch (sm_context.current) {
        case SM_STATE_INIT: {
            if (!sm_context.entry_done) {
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
            break;
        }
        case SM_STATE_CHARGING: {
            if (!sm_context.entry_done) {
                DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_CHARGER_EN_PIN);
                uart_printf("[SM] Charging started\n");
                sm_context.critical_msg_sent = false;
                sm_context.entry_done = true;
                sm_context.last_charging_tick = sm_context.minute_counter+1; 
                SM_DisablePrescaler();
                DL_GPIO_enableInterrupt(GPIOB, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
            }
            if (hall_wakeup_flag) {
                hall_wakeup_flag = false;
                sm_context.wake_reason = SM_WAKE_SETUP;
                SM_EnablePrescaler();
                SM_Transition(SM_STATE_POWER_STM);
                break;
            }
            if (sm_context.minute_counter != sm_context.last_charging_tick) {
                sm_context.last_charging_tick = sm_context.minute_counter; 
                if (SM_SafetyCheck()) return;
                if (SM_ChargingSafetyCheck()) return;
                SM_PowerContext_t pwr = SM_FetchPowerContext();
                if (pwr.vbat_mv >= SM_VBAT_FULL_MV || pwr.charger_done) {
                    DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_CHARGER_EN_PIN);
                    uart_printf("[SM] Charging complete (VBAT %dmV >= %dmV) / Charging terminated: \n", pwr.vbat_mv, SM_VBAT_FULL_MV);
                    SM_Transition(SM_STATE_IDLE);
                    break;
                } else {
                    uart_printf("[SM] CHARGING | VBUS:%4dmV VBAT:%4dmV IBAT:%4dmA SOC:%3d%% TBAT:%3.1fC TDIE:%3dC CHG_STAT:%s\n",
                        BQ25628E_Get_VBUS_mV(), pwr.vbat_mv, BQ27Z746_Get_Current_mA(),
                        BQ27Z746_Get_SOC_pct(), BQ25628E_Get_TBAT_C(), BQ25628E_Get_TDIE_C(),
                        SM_GetChargeString(pwr.chg_stat));
                }            
                if (SM_NeedsPeriodicSTMWake(pwr)) {
                    SM_EnablePrescaler();
                    sm_context.last_stm_periodic_minute = sm_context.minute_counter;
                    SM_Transition(SM_STATE_POWER_STM);
                    break;
                }
            }
            __WFI();
             break;
        }
        case SM_STATE_POWER_STM: {
            if (!sm_context.entry_done) {
                if (sm_context.wake_reason == SM_WAKE_SETUP) {
                    DL_GPIO_setPins(DIGITAL_OUTPUT_PORTA_PORT, DIGITAL_OUTPUT_PORTA_STM_MCU_IO1_PIN);
                } else {
                    DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTA_PORT, DIGITAL_OUTPUT_PORTA_STM_MCU_IO1_PIN);
                }

                SM_EnablePrescaler();

                SM_SetSTMPower(true);
                sm_context.stm_power_on_s = sm_context.second_counter;
                sm_context.last_io2_activity_s = sm_context.second_counter;

                uart_printf("[SM] STM32 powered : reason: %s\n",
                    (sm_context.wake_reason == SM_WAKE_SETUP) ? "SETUP" : "NORMAL");

                sm_context.entry_done = true;
                sm_context.stm_data_sent = false;

                DL_GPIO_disableInterrupt(EXTERNAL_INTERRUPT_CHARGER_INT_PORT, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
                DL_GPIO_enableInterrupt(EXTERNAL_INTERRUPT_STM_MCU_IO2_PORT, EXTERNAL_INTERRUPT_STM_MCU_IO2_PIN);
                SM_PrepareTelemetryResponse();
            }
            if (stm_io2_flag) {
                sm_context.stm_data_sent = true;
                stm_io2_flag = false;
                SM_SendOffer();
            }
      
            /* Process STM32 reply and send next packet immediately */
            if (sm_context.stm_data_sent && stm32Spi.rxDone) {
                stm32Spi.rxDone = false;
                sm_context.stm_data_sent = false;
                SM_DispatchIncomingPacket();
            }

            /* Inactivity timeout (works for multi-packet sessions) */
            if (sm_context.wake_reason == SM_WAKE_NORMAL) {
                if ((sm_context.second_counter - sm_context.stm_power_on_s) >= SM_NORMAL_INACTIVITY_TIMEOUT_S) {
                    uart_printf("[SM] STM32 timeout (NORMAL mode)\n");
                    SM_SetSTMPower(false);
                    SM_ResumeSystemContext();
                }
            } else {
                if ((sm_context.second_counter - sm_context.last_io2_activity_s) >= SM_SETUP_INACTIVITY_TIMEOUT_S) {
                    uart_printf("[SM] STM32 inactivity timeout (SETUP mode)\n");
                    SM_SetSTMPower(false);
                    SM_ResumeSystemContext();
                }
            }
            break;
        }
        case SM_STATE_IDLE: {
            if (!sm_context.entry_done) {
                sm_context.wake_reason = SM_WAKE_NORMAL;
                SM_SetSTMPower(false);
                sm_context.sleep_entry_minute = sm_context.minute_counter;
                DL_GPIO_disableInterrupt(EXTERNAL_INTERRUPT_STM_MCU_IO2_PORT, EXTERNAL_INTERRUPT_STM_MCU_IO2_PIN);
                DL_GPIO_clearInterruptStatus(GPIOB, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
                DL_GPIO_enableInterrupt(GPIOB, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
                hall_wakeup_flag = false;
                SM_DisablePrescaler();
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
            }
            if (sm_context.sleep_entry_minute != sm_context.minute_counter) {
                sm_context.sleep_entry_minute = sm_context.minute_counter;
                if (SM_SafetyCheck()) return;
                SM_PowerContext_t pwr = SM_FetchPowerContext();
                if (pwr.is_charging) {
                    SM_Transition(SM_STATE_CHARGING);
                    break;
                }
                if (SM_NeedsPeriodicSTMWake(pwr)) {
                    SM_EnablePrescaler();
                    sm_context.last_stm_periodic_minute = sm_context.minute_counter;
                    SM_Transition(SM_STATE_POWER_STM);
                    break;
                }
            }
            if (hall_wakeup_flag) {
                hall_wakeup_flag = false;
                sm_context.wake_reason = SM_WAKE_SETUP;
                SM_EnablePrescaler();
                SM_Transition(SM_STATE_POWER_STM);
                break;
            }
            __WFI();
            break;
        }
        case SM_STATE_CRITICAL_FAULT: {
            if (!sm_context.entry_done) {
                SM_SetSTMPower(false);
                DL_GPIO_disableInterrupt(GPIOB, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
                DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_CHARGER_EN_PIN);
                SM_EnablePrescaler();
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
            if (sm_context.fault_source == SM_FAULT_INIT_FAILED) break;

            uint32_t elapsed = sm_context.second_counter - sm_context.fault_retry_s;
            uint32_t interval = (sm_context.fault_source == SM_FAULT_I2C_BUS) ? SM_I2C_RETRY_S : 5UL;

            if (elapsed >= interval) {
                sm_context.fault_retry_s = sm_context.second_counter;

                /* I2C bus fault: just probe the addresses, go back to INIT if found */
                if (sm_context.fault_source == SM_FAULT_I2C_BUS) {
                    uart_printf("[SM] Retrying I2C bus...\n");
                    if (I2C_TryAddress(I2C_0_INST, GAUGE_I2C_ADDR) &&
                        I2C_TryAddress(I2C_0_INST, BQ25628E_I2C_ADDR)) {
                        uart_printf("[SM] I2C devices found, returning to INIT\n");
                        SM_Transition(SM_STATE_INIT);
                    } else {
                        uart_printf("[SM] I2C still unavailable\n");
                    }
                    break;
                }

                /* Gauge / charger faults: read live status and clear if clean */
                BQ27Z746_GetSafetyStatus(I2C_0_INST, &last_safety_status);
                last_charger_status = BQ25628E_GetFaultFlags();

                if (last_safety_status == 0 && last_charger_status == 0) {
                    uart_printf("[SM] All safety flags clear\n");
                    SM_Transition(SM_STATE_IDLE);
                } else {
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
            break;
        }
    }
}

bool SM_SafetyCheck(void) {
    if (sm_context.current == SM_STATE_CRITICAL_FAULT || 
        sm_context.current == SM_STATE_INIT) return false;

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
           sm_context.sm_rtc_config.wake_interval_minutes;
}
static void SM_ResumeSystemContext(void) {
    SM_PowerContext_t pwr = SM_FetchPowerContext();
    if (pwr.is_charging) {
        uart_printf("[SM] Continuing to charge\n");
        SM_Transition(SM_STATE_CHARGING);
        sm_context.entry_done = true;
        SM_DisablePrescaler();
    } else if (pwr.is_critical_low) {
        uart_printf("[SM] Critical low battery detected, entering IDLE to conserve power\n");
        SM_Transition(SM_STATE_IDLE);
    } else {
        SM_Transition(SM_STATE_IDLE);
    }
}
/* ── Protocol helpers ───────────────────────────────────────────────────── */
static void SM_SendSimpleMsg(SM_MsgType_t type)
{
    SM_SpiPacket_t *pkt = (SM_SpiPacket_t *)stm32Spi.txBuf;
    memset(stm32Spi.txBuf, 0, stm32Spi.size);

    pkt->pkt.header.msg_type   = type;
    pkt->pkt.header.payload_id = 0;
    pkt->pkt.header.length     = 0;

    SPI_Controller_Arm(&stm32Spi);
}

static void SM_SendOffer(void) { SM_SendSimpleMsg(MSG_OFFER);}

static void SM_SendAck(void)   { SM_SendSimpleMsg(MSG_ACK); }

static void SM_SendNack(void)  { SM_SendSimpleMsg(MSG_NACK);  }
static void SM_PrepareTelemetryResponse(void)
{
    SM_PowerContext_t pwr = SM_FetchPowerContext();
    BQ27Z746_GetSafetyStatus(I2C_0_INST, &last_safety_status);

    uint8_t  chg_flags   = BQ25628E_ReadReg8(BQ25628E_REG_CHG_FLAG0);
    uint8_t  fault_flags = BQ25628E_ReadReg8(BQ25628E_REG_FAULT_FLAG0);

    uint16_t batt_status = BQ27Z746_Get_BatteryStatus();

    int btmp_dC = (int)(BQ25628E_Get_TBAT_C() * 10.0f);

    snprintf(json_buf, sizeof(json_buf),
        "{\"soc\":%d,\"soh\":%d,"
        "\"vbat\":%d,\"ibat\":%d,\"vchg\":%d,\"vsys\":%d,\"ichg\":%d,\"avgi\":%d,\"avgpwr\":%d,"
        "\"gtmp\":%d,\"ctmp\":%d,\"btmp\":%d,"
        "\"cycles\":%d," 
        "\"adapter\":%d,"
        "\"state\":\"%s\",\"wake\":\"%s\","
        "\"safety\":\"0x%08X\",\"battstat\":\"0x%04X\","
        "\"chgflags\":\"0x%02X\",\"faultflags\":\"0x%02X\","
        "\"chgstat\":\"%s\","
        "\"lowbattery\":%d}",
        BQ27Z746_Get_SOC_pct(), BQ27Z746_Get_StateOfHealth_pct(),
        pwr.vbat_mv, BQ27Z746_Get_Current_mA(), BQ25628E_Get_VBUS_mV(),
        BQ25628E_Get_VSYS_mV(), BQ25628E_Get_IBUS_mA(), BQ27Z746_Get_AvgCurrent_mA(),
        BQ27Z746_Get_AvgPower_mW(), (int)BQ27Z746_Get_InternalTemp_C(),
        BQ25628E_Get_TDIE_C(), btmp_dC,
        BQ27Z746_Get_CycleCount(),
        pwr.adapter_present ? 1 : 0, SM_GetStateString(),
        (sm_context.wake_reason == SM_WAKE_SETUP) ? "SETUP" : "NORMAL",
        (unsigned int)last_safety_status, batt_status, chg_flags, fault_flags,
        SM_GetChargeString(pwr.chg_stat), pwr.is_critical_low ? 1 : 0);

    SM_SpiPacket_t *pkt = (SM_SpiPacket_t *)stm32Spi.txBuf;
    memset(stm32Spi.txBuf, 0, stm32Spi.size);

    pkt->pkt.header.msg_type   = MSG_DATA;
    pkt->pkt.header.payload_id = PID_TELEMETRY;
    size_t len = strlen(json_buf);
    pkt->pkt.header.length     = (uint16_t)len;
    uart_printf("[SM] Telemetry response: %s\n", json_buf);
    memcpy(pkt->pkt.payload.telemetry.json, json_buf, len);

    if(pwr.is_critical_low) {
        sm_context.critical_msg_sent = true;
        uart_printf("[SM] Critical low battery detected, sending last packet till charge\n");
    }
    SPI_Controller_Arm(&stm32Spi);
}

static void SM_PrepareRTCResponse(void)
{
    SM_SpiPacket_t *pkt = (SM_SpiPacket_t *)stm32Spi.txBuf;
    memset(stm32Spi.txBuf, 0, stm32Spi.size);
    RTC_GetTime(&sm_context.sm_rtc_config);
    pkt->pkt.header.msg_type   = MSG_DATA;
    pkt->pkt.header.payload_id = PID_RTC_GET;
    pkt->pkt.header.length     = sizeof(SM_RTCConfig_t);
    pkt->pkt.payload.rtc_data = sm_context.sm_rtc_config;

    SPI_Controller_Arm(&stm32Spi);
}

static void SM_HandleRequest(uint8_t pid)
{
    if (pid == PID_TELEMETRY) {
        SM_PrepareTelemetryResponse();
    } else if (pid == PID_RTC_GET) {
        SM_PrepareRTCResponse();
    } else {
        SM_SendNack();
    }
}

static void SM_HandleConfig(uint8_t pid, const void *payload)
{
    if (pid == PID_CHARGER_CFG) {
        const SM_ChargerConfig_t *cfg = (const SM_ChargerConfig_t *)payload;
        BQ25628E_ApplyProfile(cfg);
        sm_context.sm_charger_config = *cfg;
        sm_context.charger_configured = true;
        SM_SendAck();
    } else if (pid == PID_RTC_SET) {
        const SM_RTCConfig_t *cfg = (const SM_RTCConfig_t *)payload;
        RTC_SetTime(cfg);
        sm_context.sm_rtc_config = *cfg;
        SM_SendAck();
    } else {
        SM_SendNack();
    }
}

static void SM_DispatchIncomingPacket(void)
{
    SM_MsgHeader_t *hdr = (SM_MsgHeader_t *)stm32Spi.rxBuf;

    sm_context.last_io2_activity_s = sm_context.second_counter;

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
            SM_SendNack();
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

    // while (!DL_RTC_isSafeToRead(RTC));

    calendar = DL_RTC_getCalendarTime(RTC);

    out->second = calendar.seconds;
    out->minute = calendar.minutes;
    out->hour   = calendar.hours;
    out->day    = calendar.dayOfMonth;
    out->month  = calendar.month;
    out->year = calendar.year; 

}

void RTC_SetTime(const SM_RTCConfig_t *in)
{
    DL_RTC_Calendar calendar;
    while (!DL_RTC_isSafeToRead(RTC));

    if (in->second > 59 || in->minute > 59 || in->hour > 23 ||
        in->month < 1 || in->month > 12 ||
        in->day < 1 || in->day > 31)
    {
        return;
    }

    calendar.seconds    = in->second;
    calendar.minutes    = in->minute;
    calendar.hours      = in->hour;
    calendar.dayOfMonth = in->day;
    calendar.month      = in->month;
    calendar.year = in->year;
    calendar.dayOfWeek  = 1;

    DL_RTC_initCalendar(RTC, calendar, DL_RTC_FORMAT_BINARY);
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

    return is_critical;   // true = go to CRITICAL_FAULT, false = stay in current state
}