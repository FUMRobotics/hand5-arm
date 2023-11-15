################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/EEprom/eeprom.c 

OBJS += \
./Core/Src/EEprom/eeprom.o 

C_DEPS += \
./Core/Src/EEprom/eeprom.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/EEprom/%.o Core/Src/EEprom/%.su Core/Src/EEprom/%.cyclo: ../Core/Src/EEprom/%.c Core/Src/EEprom/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-EEprom

clean-Core-2f-Src-2f-EEprom:
	-$(RM) ./Core/Src/EEprom/eeprom.cyclo ./Core/Src/EEprom/eeprom.d ./Core/Src/EEprom/eeprom.o ./Core/Src/EEprom/eeprom.su

.PHONY: clean-Core-2f-Src-2f-EEprom

