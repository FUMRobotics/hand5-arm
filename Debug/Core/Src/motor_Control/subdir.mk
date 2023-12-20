################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/motor_Control/PID.c \
../Core/Src/motor_Control/motor_Control.c 

C_DEPS += \
./Core/Src/motor_Control/PID.d \
./Core/Src/motor_Control/motor_Control.d 

OBJS += \
./Core/Src/motor_Control/PID.o \
./Core/Src/motor_Control/motor_Control.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/motor_Control/%.o Core/Src/motor_Control/%.su Core/Src/motor_Control/%.cyclo: ../Core/Src/motor_Control/%.c Core/Src/motor_Control/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Core/Src/Communication -I../Core/Src/motor_Control -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-motor_Control

clean-Core-2f-Src-2f-motor_Control:
	-$(RM) ./Core/Src/motor_Control/PID.cyclo ./Core/Src/motor_Control/PID.d ./Core/Src/motor_Control/PID.o ./Core/Src/motor_Control/PID.su ./Core/Src/motor_Control/motor_Control.cyclo ./Core/Src/motor_Control/motor_Control.d ./Core/Src/motor_Control/motor_Control.o ./Core/Src/motor_Control/motor_Control.su

.PHONY: clean-Core-2f-Src-2f-motor_Control

