################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
/Users/patricio/Library/Mobile\ Documents/com~apple~CloudDocs/Delo/STM32CubeIDE/RFID-STM32F429I/BSP/SW4STM32/startup_stm32f429xx.s 

OBJS += \
./Example/SW4STM32/startup_stm32f429xx.o 

S_DEPS += \
./Example/SW4STM32/startup_stm32f429xx.d 


# Each subdirectory must supply rules for building sources it contributes
Example/SW4STM32/startup_stm32f429xx.o: /Users/patricio/Library/Mobile\ Documents/com~apple~CloudDocs/Delo/STM32CubeIDE/RFID-STM32F429I/BSP/SW4STM32/startup_stm32f429xx.s Example/SW4STM32/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -x assembler-with-cpp -MMD -MP -MF"Example/SW4STM32/startup_stm32f429xx.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Example-2f-SW4STM32

clean-Example-2f-SW4STM32:
	-$(RM) ./Example/SW4STM32/startup_stm32f429xx.d ./Example/SW4STM32/startup_stm32f429xx.o

.PHONY: clean-Example-2f-SW4STM32

