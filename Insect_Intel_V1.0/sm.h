#ifndef SM_H
#define SM_H

#include <stdint.h>
#include <stdbool.h>

/* ── States ─────────────────────────────────────────────── */
typedef enum {
    SM_STATE_INIT,
    SM_STATE_CHARGING,
    SM_STATE_POWER_STM,
    SM_STATE_SLEEP,
    SM_STATE_CRITICAL_FAULT
} SM_State_t;

/* ── Wake reason (drives STM_MCU_IO1) ───────────────────── */
typedef enum {
    SM_WAKE_NORMAL,
    SM_WAKE_SETUP
} SM_WakeReason_t;

/* ── Fault source (drives CRITICAL_FAULT behaviour) ─────── */
typedef enum {
    SM_FAULT_NONE,
    SM_FAULT_I2C_BUS,         // I2C failed at INIT — skip I2C in fault state
    SM_FAULT_GAUGE,           // Safety status flagged by gauge
    SM_FAULT_LOW_BATTERY,     // Voltage below threshold, no adapter
    SM_FAULT_INIT_FAILED      // Generic init failure
} SM_FaultSource_t;

/* ── Context ─────────────────────────────────────────────── */
typedef struct {
    SM_State_t       current;
    SM_State_t       previous;
    SM_WakeReason_t  wake_reason;
    SM_FaultSource_t fault_source;
    bool             sm_paused;
    bool             entry_done;
    uint32_t         minute_counter;
    uint32_t         stm_power_on_ms;
    uint32_t         sleep_entry_tick;   
    bool             hall_powered;
    uint32_t         last_safety_check_ms; 
    uint32_t         adapter_missing_ms;  
    bool             adapter_missing;      
    uint32_t         fault_retry_ms; 
    bool             stm_data_sent;
    uint32_t         last_io2_activity_ms;  
} SM_Context_t;

extern SM_Context_t sm_context;

/* ── SPI Command Protocol ──────────────────────────────── */
#define STM32_CMD_CONTINUE  0xAA  // Continue operation, may request data again
#define STM32_CMD_SHUTDOWN  0x55  // Done, request shutdown
#define STM32_CMD_NONE      0x00  // Default/no command

/* ── Public API ──────────────────────────────────────────── */
void        SM_Init(void);
void        SM_Run(void);
SM_State_t  SM_GetState(void);
const char* SM_GetStateString(void);
void        SM_Transition(SM_State_t new_state);
void SM_SafetyCheck(void);
void SM_AdapterCheck(void);
void SM_SendTelemetryToSTM32(void);

#endif
