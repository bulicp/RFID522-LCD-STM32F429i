################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/patricio/Library/Mobile\ Documents/com~apple~CloudDocs/Delo/STM32CubeIDE/RFID-STM32F429I/BSP/SW4STM32/syscalls.c 

OBJS += \
./syscalls.o 

C_DEPS += \
./syscalls.d 


# Each subdirectory must supply rules for building sources it contributes
syscalls.o: /Users/patricio/Library/Mobile\ Documents/com~apple~CloudDocs/Delo/STM32CubeIDE/RFID-STM32F429I/BSP/SW4STM32/syscalls.c subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F429xx -DUSE_STM32F429I_DISCO -c -I../../../Inc -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/BSP/STM32F429I-Discovery -I../../../Utilities/Log -I../../../Drivers/CMSIS/Include -Os -ffunction-sections -Wall -Wno-unused-variable -Wno-pointer-sign -Wno-main -Wno-format -Wno-address -Wno-unused-but-set-variable -Wno-strict-aliasing -Wno-parentheses -Wno-missing-braces -fstack-usage -MMD -MP -MF"syscalls.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean--2e-

clean--2e-:
	-$(RM) ./syscalls.d ./syscalls.o

.PHONY: clean--2e-

