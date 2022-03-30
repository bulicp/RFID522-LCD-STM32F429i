################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Example/User/GPIO.c \
../Example/User/MFRC522.c \
../Example/User/RFID-tag.c \
../Example/User/SPI.c \
../Example/User/UART.c \
/Users/patricio/Library/Mobile\ Documents/com~apple~CloudDocs/Delo/STM32CubeIDE/RFID-STM32F429I/BSP/Src/lcd.c \
/Users/patricio/Library/Mobile\ Documents/com~apple~CloudDocs/Delo/STM32CubeIDE/RFID-STM32F429I/BSP/Src/log.c \
/Users/patricio/Library/Mobile\ Documents/com~apple~CloudDocs/Delo/STM32CubeIDE/RFID-STM32F429I/BSP/Src/main.c \
/Users/patricio/Library/Mobile\ Documents/com~apple~CloudDocs/Delo/STM32CubeIDE/RFID-STM32F429I/BSP/Src/mems.c \
../Example/User/retarget.c \
../Example/User/stm32f4xx_hal_msp_spi.c \
../Example/User/stm32f4xx_hal_msp_uart.c \
/Users/patricio/Library/Mobile\ Documents/com~apple~CloudDocs/Delo/STM32CubeIDE/RFID-STM32F429I/BSP/Src/stm32f4xx_it.c \
/Users/patricio/Library/Mobile\ Documents/com~apple~CloudDocs/Delo/STM32CubeIDE/RFID-STM32F429I/BSP/Src/touchscreen.c \
/Users/patricio/Library/Mobile\ Documents/com~apple~CloudDocs/Delo/STM32CubeIDE/RFID-STM32F429I/BSP/Src/ts_calibration.c 

OBJS += \
./Example/User/GPIO.o \
./Example/User/MFRC522.o \
./Example/User/RFID-tag.o \
./Example/User/SPI.o \
./Example/User/UART.o \
./Example/User/lcd.o \
./Example/User/log.o \
./Example/User/main.o \
./Example/User/mems.o \
./Example/User/retarget.o \
./Example/User/stm32f4xx_hal_msp_spi.o \
./Example/User/stm32f4xx_hal_msp_uart.o \
./Example/User/stm32f4xx_it.o \
./Example/User/touchscreen.o \
./Example/User/ts_calibration.o 

C_DEPS += \
./Example/User/GPIO.d \
./Example/User/MFRC522.d \
./Example/User/RFID-tag.d \
./Example/User/SPI.d \
./Example/User/UART.d \
./Example/User/lcd.d \
./Example/User/log.d \
./Example/User/main.d \
./Example/User/mems.d \
./Example/User/retarget.d \
./Example/User/stm32f4xx_hal_msp_spi.d \
./Example/User/stm32f4xx_hal_msp_uart.d \
./Example/User/stm32f4xx_it.d \
./Example/User/touchscreen.d \
./Example/User/ts_calibration.d 


# Each subdirectory must supply rules for building sources it contributes
Example/User/%.o Example/User/%.su: ../Example/User/%.c Example/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F429xx -DUSE_STM32F429I_DISCO -c -I../../../Inc -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/BSP/STM32F429I-Discovery -I../../../Utilities/Log -I../../../Drivers/CMSIS/Include -Os -ffunction-sections -Wall -Wno-unused-variable -Wno-pointer-sign -Wno-main -Wno-format -Wno-address -Wno-unused-but-set-variable -Wno-strict-aliasing -Wno-parentheses -Wno-missing-braces -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Example/User/lcd.o: /Users/patricio/Library/Mobile\ Documents/com~apple~CloudDocs/Delo/STM32CubeIDE/RFID-STM32F429I/BSP/Src/lcd.c Example/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F429xx -DUSE_STM32F429I_DISCO -c -I../../../Inc -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/BSP/STM32F429I-Discovery -I../../../Utilities/Log -I../../../Drivers/CMSIS/Include -Os -ffunction-sections -Wall -Wno-unused-variable -Wno-pointer-sign -Wno-main -Wno-format -Wno-address -Wno-unused-but-set-variable -Wno-strict-aliasing -Wno-parentheses -Wno-missing-braces -fstack-usage -MMD -MP -MF"Example/User/lcd.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Example/User/log.o: /Users/patricio/Library/Mobile\ Documents/com~apple~CloudDocs/Delo/STM32CubeIDE/RFID-STM32F429I/BSP/Src/log.c Example/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F429xx -DUSE_STM32F429I_DISCO -c -I../../../Inc -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/BSP/STM32F429I-Discovery -I../../../Utilities/Log -I../../../Drivers/CMSIS/Include -Os -ffunction-sections -Wall -Wno-unused-variable -Wno-pointer-sign -Wno-main -Wno-format -Wno-address -Wno-unused-but-set-variable -Wno-strict-aliasing -Wno-parentheses -Wno-missing-braces -fstack-usage -MMD -MP -MF"Example/User/log.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Example/User/main.o: /Users/patricio/Library/Mobile\ Documents/com~apple~CloudDocs/Delo/STM32CubeIDE/RFID-STM32F429I/BSP/Src/main.c Example/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F429xx -DUSE_STM32F429I_DISCO -c -I../../../Inc -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/BSP/STM32F429I-Discovery -I../../../Utilities/Log -I../../../Drivers/CMSIS/Include -Os -ffunction-sections -Wall -Wno-unused-variable -Wno-pointer-sign -Wno-main -Wno-format -Wno-address -Wno-unused-but-set-variable -Wno-strict-aliasing -Wno-parentheses -Wno-missing-braces -fstack-usage -MMD -MP -MF"Example/User/main.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Example/User/mems.o: /Users/patricio/Library/Mobile\ Documents/com~apple~CloudDocs/Delo/STM32CubeIDE/RFID-STM32F429I/BSP/Src/mems.c Example/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F429xx -DUSE_STM32F429I_DISCO -c -I../../../Inc -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/BSP/STM32F429I-Discovery -I../../../Utilities/Log -I../../../Drivers/CMSIS/Include -Os -ffunction-sections -Wall -Wno-unused-variable -Wno-pointer-sign -Wno-main -Wno-format -Wno-address -Wno-unused-but-set-variable -Wno-strict-aliasing -Wno-parentheses -Wno-missing-braces -fstack-usage -MMD -MP -MF"Example/User/mems.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Example/User/stm32f4xx_it.o: /Users/patricio/Library/Mobile\ Documents/com~apple~CloudDocs/Delo/STM32CubeIDE/RFID-STM32F429I/BSP/Src/stm32f4xx_it.c Example/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F429xx -DUSE_STM32F429I_DISCO -c -I../../../Inc -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/BSP/STM32F429I-Discovery -I../../../Utilities/Log -I../../../Drivers/CMSIS/Include -Os -ffunction-sections -Wall -Wno-unused-variable -Wno-pointer-sign -Wno-main -Wno-format -Wno-address -Wno-unused-but-set-variable -Wno-strict-aliasing -Wno-parentheses -Wno-missing-braces -fstack-usage -MMD -MP -MF"Example/User/stm32f4xx_it.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Example/User/touchscreen.o: /Users/patricio/Library/Mobile\ Documents/com~apple~CloudDocs/Delo/STM32CubeIDE/RFID-STM32F429I/BSP/Src/touchscreen.c Example/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F429xx -DUSE_STM32F429I_DISCO -c -I../../../Inc -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/BSP/STM32F429I-Discovery -I../../../Utilities/Log -I../../../Drivers/CMSIS/Include -Os -ffunction-sections -Wall -Wno-unused-variable -Wno-pointer-sign -Wno-main -Wno-format -Wno-address -Wno-unused-but-set-variable -Wno-strict-aliasing -Wno-parentheses -Wno-missing-braces -fstack-usage -MMD -MP -MF"Example/User/touchscreen.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Example/User/ts_calibration.o: /Users/patricio/Library/Mobile\ Documents/com~apple~CloudDocs/Delo/STM32CubeIDE/RFID-STM32F429I/BSP/Src/ts_calibration.c Example/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F429xx -DUSE_STM32F429I_DISCO -c -I../../../Inc -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/BSP/STM32F429I-Discovery -I../../../Utilities/Log -I../../../Drivers/CMSIS/Include -Os -ffunction-sections -Wall -Wno-unused-variable -Wno-pointer-sign -Wno-main -Wno-format -Wno-address -Wno-unused-but-set-variable -Wno-strict-aliasing -Wno-parentheses -Wno-missing-braces -fstack-usage -MMD -MP -MF"Example/User/ts_calibration.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Example-2f-User

clean-Example-2f-User:
	-$(RM) ./Example/User/GPIO.d ./Example/User/GPIO.o ./Example/User/GPIO.su ./Example/User/MFRC522.d ./Example/User/MFRC522.o ./Example/User/MFRC522.su ./Example/User/RFID-tag.d ./Example/User/RFID-tag.o ./Example/User/RFID-tag.su ./Example/User/SPI.d ./Example/User/SPI.o ./Example/User/SPI.su ./Example/User/UART.d ./Example/User/UART.o ./Example/User/UART.su ./Example/User/lcd.d ./Example/User/lcd.o ./Example/User/lcd.su ./Example/User/log.d ./Example/User/log.o ./Example/User/log.su ./Example/User/main.d ./Example/User/main.o ./Example/User/main.su ./Example/User/mems.d ./Example/User/mems.o ./Example/User/mems.su ./Example/User/retarget.d ./Example/User/retarget.o ./Example/User/retarget.su ./Example/User/stm32f4xx_hal_msp_spi.d ./Example/User/stm32f4xx_hal_msp_spi.o ./Example/User/stm32f4xx_hal_msp_spi.su ./Example/User/stm32f4xx_hal_msp_uart.d ./Example/User/stm32f4xx_hal_msp_uart.o ./Example/User/stm32f4xx_hal_msp_uart.su ./Example/User/stm32f4xx_it.d ./Example/User/stm32f4xx_it.o ./Example/User/stm32f4xx_it.su ./Example/User/touchscreen.d ./Example/User/touchscreen.o ./Example/User/touchscreen.su ./Example/User/ts_calibration.d ./Example/User/ts_calibration.o ./Example/User/ts_calibration.su

.PHONY: clean-Example-2f-User

