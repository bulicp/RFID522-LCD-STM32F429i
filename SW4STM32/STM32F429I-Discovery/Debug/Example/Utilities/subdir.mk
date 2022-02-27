################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/patricio/Library/Mobile\ Documents/com~apple~CloudDocs/Delo/STM32CubeIDE/RFID-STM32F429I/BSP/Utilities/Log/lcd_log.c 

OBJS += \
./Example/Utilities/lcd_log.o 

C_DEPS += \
./Example/Utilities/lcd_log.d 


# Each subdirectory must supply rules for building sources it contributes
Example/Utilities/lcd_log.o: /Users/patricio/Library/Mobile\ Documents/com~apple~CloudDocs/Delo/STM32CubeIDE/RFID-STM32F429I/BSP/Utilities/Log/lcd_log.c Example/Utilities/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F429xx -DUSE_STM32F429I_DISCO -c -I../../../Inc -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/BSP/STM32F429I-Discovery -I../../../Utilities/Log -I../../../Drivers/CMSIS/Include -Os -ffunction-sections -Wall -Wno-unused-variable -Wno-pointer-sign -Wno-main -Wno-format -Wno-address -Wno-unused-but-set-variable -Wno-strict-aliasing -Wno-parentheses -Wno-missing-braces -fstack-usage -MMD -MP -MF"Example/Utilities/lcd_log.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Example-2f-Utilities

clean-Example-2f-Utilities:
	-$(RM) ./Example/Utilities/lcd_log.d ./Example/Utilities/lcd_log.o

.PHONY: clean-Example-2f-Utilities

