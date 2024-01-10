################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Management/global_state_machine.c 

OBJS += \
./Core/Src/Management/global_state_machine.o 

C_DEPS += \
./Core/Src/Management/global_state_machine.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Management/%.o Core/Src/Management/%.su Core/Src/Management/%.cyclo: ../Core/Src/Management/%.c Core/Src/Management/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Management

clean-Core-2f-Src-2f-Management:
	-$(RM) ./Core/Src/Management/global_state_machine.cyclo ./Core/Src/Management/global_state_machine.d ./Core/Src/Management/global_state_machine.o ./Core/Src/Management/global_state_machine.su

.PHONY: clean-Core-2f-Src-2f-Management

