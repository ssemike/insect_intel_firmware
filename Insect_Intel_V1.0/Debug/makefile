################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccs2031/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS

GEN_OPTS__FLAG := @"./device.opt" 
GEN_CMDS__FLAG := -Wl,-l"./device_linker.cmd" 

ORDERED_OBJS += \
"./ti_msp_dl_config.o" \
"./functions.o" \
"./helper_functions.o" \
"./main.o" \
"./sm.o" \
"./startup_mspm0g350x_ticlang.o" \
"./HAL/i2c.o" \
"./HAL/spi_master.o" \
"./HAL/uart.o" \
"./emulation_type_b/eeprom_emulation_type_b.o" \
"./ics/BQ25628/BQ25628_functions.o" \
"./ics/BQ27Z7/BQ27Z7_functions.o" \
"./ics/BQ27Z7/gauge.o" \
$(GEN_CMDS__FLAG) \
-Wl,-ldevice.cmd.genlibs \
-Wl,-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include HAL/subdir_vars.mk
-include emulation_type_b/subdir_vars.mk
-include ics/BQ25628/subdir_vars.mk
-include ics/BQ27Z7/subdir_vars.mk
-include subdir_rules.mk
-include HAL/subdir_rules.mk
-include emulation_type_b/subdir_rules.mk
-include ics/BQ25628/subdir_rules.mk
-include ics/BQ27Z7/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
HW_SW_interface_CCS.out 

EXE_OUTPUTS__QUOTED += \
"HW_SW_interface_CCS.out" 


# All Target
all: $(OBJS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "HW_SW_interface_CCS.out"

# Tool invocations
HW_SW_interface_CCS.out: $(OBJS) $(GEN_CMDS)
	@echo 'Arm Linker - building target: "$@"'
	"C:/ti/ccs2031/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -gdwarf-3 -Wl,-m"HW_SW_interface_CCS.map" -Wl,-i"C:/ti/mspm0_sdk_2_09_00_01/source" -Wl,-i"C:/Users/Admin/Desktop/Software code/Final Software/Insect Intel/Insect_Intel_V1.0" -Wl,-i"C:/Users/Admin/Desktop/Software code/Final Software/Insect Intel/Insect_Intel_V1.0/Debug/syscfg" -Wl,-i"C:/ti/ccs2031/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/lib" -Wl,--diag_wrap=off -Wl,--display_error_number -Wl,--warn_sections -Wl,--xml_link_info="HW_SW_interface_CCS_linkInfo.xml" -Wl,--rom_model -o "HW_SW_interface_CCS.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(GEN_MISC_FILES__QUOTED)$(GEN_FILES__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "ti_msp_dl_config.o" "functions.o" "helper_functions.o" "main.o" "sm.o" "startup_mspm0g350x_ticlang.o" "HAL\i2c.o" "HAL\spi_master.o" "HAL\uart.o" "emulation_type_b\eeprom_emulation_type_b.o" "ics\BQ25628\BQ25628_functions.o" "ics\BQ27Z7\BQ27Z7_functions.o" "ics\BQ27Z7\gauge.o" 
	-$(RM) "ti_msp_dl_config.d" "functions.d" "helper_functions.d" "main.d" "sm.d" "startup_mspm0g350x_ticlang.d" "HAL\i2c.d" "HAL\spi_master.d" "HAL\uart.d" "emulation_type_b\eeprom_emulation_type_b.d" "ics\BQ25628\BQ25628_functions.d" "ics\BQ27Z7\BQ27Z7_functions.d" "ics\BQ27Z7\gauge.d" 
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

