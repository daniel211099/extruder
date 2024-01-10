################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/HMI/hmi_display.c \
../Core/Src/HMI/signallight_control.c 

OBJS += \
./Core/Src/HMI/hmi_display.o \
./Core/Src/HMI/signallight_control.o 

C_DEPS += \
./Core/Src/HMI/hmi_display.d \
./Core/Src/HMI/signallight_control.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/HMI/%.o Core/Src/HMI/%.su Core/Src/HMI/%.cyclo: ../Core/Src/HMI/%.c Core/Src/HMI/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-HMI

clean-Core-2f-Src-2f-HMI:
	-$(RM) ./Core/Src/HMI/hmi_display.cyclo ./Core/Src/HMI/hmi_display.d ./Core/Src/HMI/hmi_display.o ./Core/Src/HMI/hmi_display.su ./Core/Src/HMI/signallight_control.cyclo ./Core/Src/HMI/signallight_control.d ./Core/Src/HMI/signallight_control.o ./Core/Src/HMI/signallight_control.su

.PHONY: clean-Core-2f-Src-2f-HMI

