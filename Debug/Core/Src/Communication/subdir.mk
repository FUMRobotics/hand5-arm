################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Communication/ESP8266_UART.c 

OBJS += \
./Core/Src/Communication/ESP8266_UART.o 

C_DEPS += \
./Core/Src/Communication/ESP8266_UART.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Communication/%.o Core/Src/Communication/%.su Core/Src/Communication/%.cyclo: ../Core/Src/Communication/%.c Core/Src/Communication/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Src/EEprom -I../Core/Src/Communication -I../Core/Src/motor_Control -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Communication

clean-Core-2f-Src-2f-Communication:
	-$(RM) ./Core/Src/Communication/ESP8266_UART.cyclo ./Core/Src/Communication/ESP8266_UART.d ./Core/Src/Communication/ESP8266_UART.o ./Core/Src/Communication/ESP8266_UART.su

.PHONY: clean-Core-2f-Src-2f-Communication

