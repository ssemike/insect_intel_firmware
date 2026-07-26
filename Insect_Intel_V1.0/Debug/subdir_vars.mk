################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
SYSCFG_SRCS += \
../MSPM0_interface.syscfg 

C_SRCS += \
./ti_msp_dl_config.c \
../functions.c \
../helper_functions.c \
../main.c \
../sm.c \
../startup_mspm0g350x_ticlang.c 

GEN_CMDS += \
./device_linker.cmd 

GEN_FILES += \
./device_linker.cmd \
./device.opt \
./ti_msp_dl_config.c 

C_DEPS += \
./ti_msp_dl_config.d \
./functions.d \
./helper_functions.d \
./main.d \
./sm.d \
./startup_mspm0g350x_ticlang.d 

GEN_OPTS += \
./device.opt 

OBJS += \
./ti_msp_dl_config.o \
./functions.o \
./helper_functions.o \
./main.o \
./sm.o \
./startup_mspm0g350x_ticlang.o 

GEN_MISC_FILES += \
./device.cmd.genlibs \
./ti_msp_dl_config.h \
./Event.dot 

OBJS__QUOTED += \
"ti_msp_dl_config.o" \
"functions.o" \
"helper_functions.o" \
"main.o" \
"sm.o" \
"startup_mspm0g350x_ticlang.o" 

GEN_MISC_FILES__QUOTED += \
"device.cmd.genlibs" \
"ti_msp_dl_config.h" \
"Event.dot" 

C_DEPS__QUOTED += \
"ti_msp_dl_config.d" \
"functions.d" \
"helper_functions.d" \
"main.d" \
"sm.d" \
"startup_mspm0g350x_ticlang.d" 

GEN_FILES__QUOTED += \
"device_linker.cmd" \
"device.opt" \
"ti_msp_dl_config.c" 

SYSCFG_SRCS__QUOTED += \
"../MSPM0_interface.syscfg" 

C_SRCS__QUOTED += \
"./ti_msp_dl_config.c" \
"../functions.c" \
"../helper_functions.c" \
"../main.c" \
"../sm.c" \
"../startup_mspm0g350x_ticlang.c" 


