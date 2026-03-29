
#include "ti_msp_dl_config.h"
#include "HAL/i2c.h"
#include "functions.h"
#include "HAL/uart.h"
#include "ics/BQ25628/BQ25628_functions.h"
#include "ics/BQ27Z7/BQ27Z7_functions.h"
#include "HAL/spi_master.h"

volatile bool bq_monitor_active    = false;
volatile bool hall_monitor_active  = false;
volatile bool gauge_monitor_active = false;
volatile uint32_t monitor_rate = 2000; 
volatile uint32_t hall_monitor_counter = 0;

void setupCLI(void) {
    CLI_RegisterCommand("help", cmd_help, "Show available commands");
    CLI_RegisterCommand("pwr",  cmd_pwr,  "Control power rails: 3v8, lora, lte, wifi, stm");
    CLI_RegisterCommand("i2cscan", cmd_i2cscan, "Scan I2C bus: i2cscan <0|1>");
    CLI_RegisterCommand("hall", cmd_hall, "Hall sensor: hall <pwr|status>");
    CLI_RegisterCommand("bq", cmd_bq, "BQ25628E charger control - type bq for full help");
    CLI_RegisterCommand("spi", cmd_spi, "SPI Master tx_view, tx_write, test");
    CLI_RegisterCommand("gauge",   cmd_gauge,   "BQ27Z746 gauge — type gauge for help");
}


int main(void)
{
    SYSCFG_DL_init(); 
    uart_init();  
    setupCLI();  
    i2c_init();
    NVIC_EnableIRQ(SPI_1_INST_INT_IRQN);
    SPI_Controller_Init(&stm32Spi, SPI_1_INST,  DMA_CH0_CHAN_ID, DMA_CH1_CHAN_ID, gSPI_TxPacket, gSPI_RxPacket, SPI_PACKET_SIZE);
    char processingBuffer[MAX_INPUT_LEN];

    while (1) {
        if (data_received) {
            get_UART_buffer(processingBuffer);
            CLI_ProcessInput(processingBuffer);
        }
        if (hall_monitor_active) {
            delay_cycles(monitor_rate * 32000); // Fixed 200ms delay
            
            uint32_t pin_state = DL_GPIO_readPins(EXTERNAL_INTERRUPT_PORT, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
            
            if (pin_state) {
                uart_printf("SETUP_INT: HIGH (1)\n");
            } else {
                uart_printf("SETUP_INT: LOW (0)\n");
            }
            
            // Check if user wants to stop
            if (data_received) {
                hall_monitor_active = false;
                get_UART_buffer(processingBuffer);
                uart_printf("Hall monitoring stopped\n");
            }
        }
        if (bq_monitor_active) {
            delay_cycles(monitor_rate * 32000); // 200 ms refresh

            uint32_t charger_int = DL_GPIO_readPins(EXTERNAL_INTERRUPT_PORT, EXTERNAL_INTERRUPT_CHARGER_INT_PIN); 

            BQ25628E_UpdateTelemetry();
            uint8_t stat1 = BQ25628E_ReadReg8(BQ25628E_REG_STAT1);
            uint8_t chg_stat = (stat1 >> 3) & 0x03;

            const char* desc;
            switch (chg_stat) {
                case 0: desc = "Not Charging / Terminated"; break;
                case 1: desc = "Pre/Trickle/Fast (CC)"; break;
                case 2: desc = "Taper (CV)"; break;
                case 3: desc = "Top-Off"; break;
                default: desc = "Unknown";
            }

            uart_printf("=== BQ25628E MONITOR (200ms) ===\n");
            uart_printf("CHARGER_INT : %s\n", charger_int ? "HIGH" : "LOW");
            uart_printf("Charging Status : %s  (CHG_STAT[4:3] = 0b%02b)\n", desc, chg_stat);
            uart_printf("VBUS:%4dmV VBAT:%4dmV  VSYS:%4dmV  IBUS:%4dmA  IBAT:%4dmA\n",BQ25628E_Get_VBUS_mV(),
                        BQ25628E_Get_VBAT_mV(), BQ25628E_Get_VSYS_mV(),
                        BQ25628E_Get_IBUS_mA(), BQ25628E_Get_IBAT_mA());
            uart_printf("ChgFlag0:0x%02X  FaultFlag0:0x%02X\n",
                        BQ25628E_ReadReg8(BQ25628E_REG_CHG_FLAG0),
                        BQ25628E_ReadReg8(BQ25628E_REG_FAULT_FLAG0));

            if (data_received) { 
                bq_monitor_active = false;
                get_UART_buffer(processingBuffer);
                uart_printf("Monitor stopped\n");
            }
        }
        if (gauge_monitor_active) {
            delay_cycles(monitor_rate * 32000);

            BQ27Z746_UpdateTelemetry(I2C_0_INST);

            uint16_t tte = BQ27Z746_Get_TimeToEmpty_min();
            uint16_t ttf = BQ27Z746_Get_TimeToFull_min();

            /* Determine a one-word state string from cached BatteryStatus */
            const char *state;
            if      (BQ27Z746_IsDischarging())     state = "DISCHARGING";
            else if (BQ27Z746_IsFullyCharged())    state = "FULLY CHARGED";
            else if (BQ27Z746_IsFullyDischarged()) state = "FULLY DISCHARGED";
            else                                   state = "CHARGING";

            uart_printf("=== BQ27Z746 MONITOR (200ms) ===\n");
            uart_printf("State  : %s\n", state);
            uart_printf("SOC    : %3d %%   SoH: %3d %%   Cycles: %d\n",
                        BQ27Z746_Get_SOC_pct(),
                        BQ27Z746_Get_StateOfHealth_pct(),
                        BQ27Z746_Get_CycleCount());
            uart_printf("VBAT   : %4d mV\n", BQ27Z746_Get_Voltage_mV());
            uart_printf("IBAT   : %5d mA   AvgI: %5d mA\n",
                        BQ27Z746_Get_Current_mA(),
                        BQ27Z746_Get_AvgCurrent_mA());
            uart_printf("AvgPwr : %5d mW\n", BQ27Z746_Get_AvgPower_mW());
            uart_printf("RemCap : %4d mAh  FullCap: %4d mAh\n",
                        BQ27Z746_Get_RemainingCap_mAh(),
                        BQ27Z746_Get_FullChargeCap_mAh());
            uart_printf("Temp   : %3d C   InternalTemp: %3d C\n",
                        BQ27Z746_Get_Temperature_C(),
                        BQ27Z746_Get_InternalTemp_C());

            /* TTE / TTF with 0xFFFF guard */
            uart_printf("TTE    : ");
            if (tte == 0xFFFFu) uart_printf("  ---");
            else                uart_printf("%4d min", tte);

            uart_printf("   TTF: ");
            if (ttf == 0xFFFFu) uart_printf("  ---\n");
            else                uart_printf("%4d min\n", ttf);

            uart_printf("Status : 0x%04X\n", BQ27Z746_Get_BatteryStatus());
            uart_printf("--------------------------------\n");

            if (data_received) {
                gauge_monitor_active = false;
                get_UART_buffer(processingBuffer);
                uart_printf("Gauge monitor stopped\n");
            }
        }
    }
}

void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            UARTReceive();
            break;
        default:
            break;
    }
}