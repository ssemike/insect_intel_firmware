#include "BQ25628_functions.h"
#include "HAL/i2c.h"
#include <stdbool.h>
#include <math.h>

/* Internal Telemetry Cache */
static uint16_t g_vbat_mV = 0;
static uint16_t g_vsys_mV = 0;
static uint16_t g_vbus_mV = 0;
static int16_t  g_ibus_mA = 0;
static int16_t  g_ibat_mA = 0;
static int16_t g_tdie_C = 0;


/* -------------------------------------------------------------------------- */
/* Low-Level I2C Helpers (unchanged)                                          */
/* -------------------------------------------------------------------------- */
uint8_t BQ25628E_ReadReg8(uint8_t reg) {
    uint8_t val = 0;
    I2C_ReadDevice(I2C_0_INST, BQ25628E_I2C_ADDR, reg, &val, 1);
    return val;
}

void BQ25628E_WriteReg8(uint8_t reg, uint8_t val) {
    I2C_WriteDevice(I2C_0_INST, BQ25628E_I2C_ADDR, reg, &val, 1);
}

uint16_t BQ25628E_ReadReg16(uint8_t reg) {
    uint8_t data[2] = {0, 0};
    I2C_ReadDevice(I2C_0_INST, BQ25628E_I2C_ADDR, reg, data, 2);
    return (uint16_t)((data[1] << 8) | data[0]);
}

void BQ25628E_WriteReg16(uint8_t reg, uint16_t val) {
    uint8_t data[2];
    data[0] = (uint8_t)(val & 0xFF);
    data[1] = (uint8_t)((val >> 8) & 0xFF);
    I2C_WriteDevice(I2C_0_INST, BQ25628E_I2C_ADDR, reg, data, 2);
}

void BQ25628E_UpdateBits8(uint8_t reg, uint8_t mask, uint8_t value) {
    uint8_t old = BQ25628E_ReadReg8(reg);
    uint8_t v = (old & ~mask) | (value & mask);
    if (v != old) BQ25628E_WriteReg8(reg, v);
}

/* -------------------------------------------------------------------------- */
/* Public API Implementation                                       */
/* -------------------------------------------------------------------------- */
bool BQ25628E_Init_Default(void) {
    BQ25628E_Set_VREG_mV(3600);
    BQ25628E_Set_ICHG_mA(1000);
    BQ25628E_Set_IINDPM_mA(2000);
    BQ25628E_Set_VINDPM_mV(4500);     
    BQ25628E_Set_VSYSMIN_mV(3600);  
    // BQ25628E_Set_Precharge_mA(100);   
    // BQ25628E_Set_Termination_mA(50); 
    BQ25628E_Set_TS_Ignore(true);  
    BQ25628E_Set_PeakDischarge_6A(); 

    BQ25628E_WriteReg8(BQ25628E_REG_ADC_CTRL, BQ25628E_ADC_EN);
    BQ25628E_Disable_Watchdog();
    // BQ25628E_Set_ChargerEnable(true);
    return true;
}


#include <math.h>  // For roundf if needed; otherwise use casts.

void BQ25628E_UpdateTelemetry(void) {
    uint16_t raw;

    raw = BQ25628E_ReadReg16(BQ25628E_REG_ADC_VBAT);
    uint16_t vbat_code = (raw >> 1) & 0x0FFF;
    g_vbat_mV = (uint16_t) roundf(vbat_code * 1.99f); 

    raw = BQ25628E_ReadReg16(BQ25628E_REG_ADC_VSYS);
    uint16_t vsys_code = (raw >> 1) & 0x0FFF;
    g_vsys_mV = (uint16_t) roundf(vsys_code * 1.99f);

    raw = BQ25628E_ReadReg16(BQ25628E_REG_ADC_VBUS);
    uint16_t vbus_code = (raw >> 2) & 0x1FFF;
    g_vbus_mV = (uint16_t) roundf(vbus_code * 3.97f);

    raw = BQ25628E_ReadReg16(BQ25628E_REG_ADC_IBUS);
    int16_t ibus_code = (int16_t) (raw >> 1);
    g_ibus_mA = ibus_code * 2;

    raw = BQ25628E_ReadReg16(BQ25628E_REG_ADC_IBAT);
    if (raw == 0x8000) {
        g_ibat_mA = 0;
    } else {
        int16_t ibat_code = (int16_t) (raw >> 2);
        g_ibat_mA = ibat_code * 4;
    }
    raw = BQ25628E_ReadReg16(BQ25628E_REG_TDIE_ADC);
    int16_t tdie_code = (int16_t)(raw & 0x0FFFu);
    if (tdie_code > 0x7FF) tdie_code -= 0x1000;
    g_tdie_C = (int16_t)roundf(tdie_code * 0.5f);
}
void BQ25628E_PetWatchdog(void) {
    BQ25628E_UpdateBits8(BQ25628E_REG_CTRL0, BQ25628E_CTRL0_WD_RST, BQ25628E_CTRL0_WD_RST);
}

/* --- Setters --- */

/* --- 16-bit Setters (CORRECTED with proper bit alignment) --- */
void BQ25628E_Set_VREG_mV(uint16_t voltage_mV) {
    if (voltage_mV < 3500) voltage_mV = 3500;
    if (voltage_mV > 4800) voltage_mV = 4800;
    uint16_t code = voltage_mV / 10u;     
    uint16_t reg  = code << 3u;
    BQ25628E_WriteReg16(BQ25628E_REG_VREG, reg);
}

void BQ25628E_Set_ICHG_mA(uint16_t current_mA) {
    if (current_mA < 40)   current_mA = 40;
    if (current_mA > 2000) current_mA = 2000;
    uint16_t code = current_mA / 40u;    
    uint16_t reg  = code << 5u;
    BQ25628E_WriteReg16(BQ25628E_REG_ICHG, reg);
}


void BQ25628E_Set_IINDPM_mA(uint16_t current_mA) {
    if (current_mA < 100)  current_mA = 100;
    if (current_mA > 3200) current_mA = 3200;
    uint16_t code = current_mA / 20u;   
    uint16_t reg  = code << 4u;
    BQ25628E_WriteReg16(BQ25628E_REG_IINDPM, reg);
}

void BQ25628E_Set_VINDPM_mV(uint16_t voltage_mV) {
    if (voltage_mV < 3900) voltage_mV = 3900;   
    if (voltage_mV > 16800) voltage_mV = 16800;
    uint16_t code = voltage_mV / 40u;           
    uint16_t reg  = code << 5u;
    BQ25628E_WriteReg16(BQ25628E_REG_VINDPM, reg);
}

void BQ25628E_Set_VSYSMIN_mV(uint16_t voltage_mV) {
    if (voltage_mV < 2560) voltage_mV = 2560;
    if (voltage_mV > 3840) voltage_mV = 3840;
    uint16_t code = voltage_mV / 80u;          
    uint16_t reg  = code << 6u;
    BQ25628E_WriteReg16(BQ25628E_REG_VSYSMIN, reg);
}

void BQ25628E_Set_ChargerEnable(bool enable) {
    uint8_t val = enable ? BQ25628E_CTRL0_EN_CHG : 0;
    BQ25628E_UpdateBits8(BQ25628E_REG_CTRL0, BQ25628E_CTRL0_EN_CHG, val);
}

void BQ25628E_Set_HIZ(bool enable) {
    uint8_t val = enable ? BQ25628E_CTRL0_EN_HIZ : 0;
    BQ25628E_UpdateBits8(BQ25628E_REG_CTRL0, BQ25628E_CTRL0_EN_HIZ, val);
}

void BQ25628E_Set_Precharge_mA(uint16_t current_mA) {
    if (current_mA < 10)   current_mA = 10;
    if (current_mA > 310) current_mA = 310;
    uint16_t code = current_mA / 10u;    
    uint16_t reg  = code << 3u;
    BQ25628E_WriteReg8(BQ25628E_REG_IPRECHG, reg);
}

void BQ25628E_Set_Termination_mA(uint16_t current_mA) {
    if (current_mA < 5)   current_mA = 5;
    if (current_mA > 310) current_mA = 310;
    uint16_t code = current_mA / 5u;    
    uint16_t reg  = code << 2u;
    BQ25628E_WriteReg8(BQ25628E_REG_TERM_CTRL, reg);
}

void BQ25628E_Set_TS_Ignore(bool ignore) {
    uint8_t val = ignore ? BQ25628E_NTC0_TS_IGNORE : 0u;
    BQ25628E_UpdateBits8(BQ25628E_REG_NTC0, BQ25628E_NTC0_TS_IGNORE, val);
}

void BQ25628E_Set_PeakDischarge_6A(void) {
    BQ25628E_UpdateBits8(BQ25628E_REG_CTRL3,
                         BQ25628E_CTRL3_IBAT_PK_MASK,
                         BQ25628E_CTRL3_IBAT_PK_6A);
}


void BQ25628E_Disable_Watchdog(void) {
    // We update bits 1:0 to 00b to disable the timer
    BQ25628E_UpdateBits8(BQ25628E_REG_CTRL0, 
                         BQ25628E_CTRL0_WATCHDOG_MASK, 
                         BQ25628E_CTRL0_WATCHDOG_DIS);
}

/* Getters unchanged */
/* --- Getters --- */
uint16_t BQ25628E_Get_VBAT_mV(void) { return g_vbat_mV; }
uint16_t BQ25628E_Get_VSYS_mV(void) { return g_vsys_mV; }
uint16_t BQ25628E_Get_VBUS_mV(void) { return g_vbus_mV; }
int16_t  BQ25628E_Get_IBUS_mA(void) { return g_ibus_mA; }
int16_t  BQ25628E_Get_IBAT_mA(void) { return g_ibat_mA; }
int16_t  BQ25628E_Get_TDIE_C(void) { return g_tdie_C; }