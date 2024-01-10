################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Kommunikation/pc_recieve_handler.c \
../Core/Src/Kommunikation/pc_send_handler.c \
../Core/Src/Kommunikation/sensor_recieve_handler.c \
../Core/Src/Kommunikation/uart_data.c \
../Core/Src/Kommunikation/uart_processor.c 

OBJS += \
./Core/Src/Kommunikation/pc_recieve_handler.o \
./Core/Src/Kommunikation/pc_send_handler.o \
./Core/Src/Kommunikation/sensor_recieve_handler.o \
./Core/Src/Kommunikation/uart_data.o \
./Core/Src/Kommunikation/uart_processor.o 

C_DEPS += \
./Core/Src/Kommunikation/pc_recieve_handler.d \
./Core/Src/Kommunikation/pc_send_handler.d \
./Core/Src/Kommunikation/sensor_recieve_handler.d \
./Core/Src/Kommunikation/uart_data.d \
./Core/Src/Kommunikation/uart_processor.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Kommunikation/%.o Core/Src/Kommunikation/%.su Core/Src/Kommunikation/%.cyclo: ../Core/Src/Kommunikation/%.c Core/Src/Kommunikation/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Kommunikation

clean-Core-2f-Src-2f-Kommunikation:
	-$(RM) ./Core/Src/Kommunikation/pc_recieve_handler.cyclo ./Core/Src/Kommunikation/pc_recieve_handler.d ./Core/Src/Kommunikation/pc_recieve_handler.o ./Core/Src/Kommunikation/pc_recieve_handler.su ./Core/Src/Kommunikation/pc_send_handler.cyclo ./Core/Src/Kommunikation/pc_send_handler.d ./Core/Src/Kommunikation/pc_send_handler.o ./Core/Src/Kommunikation/pc_send_handler.su ./Core/Src/Kommunikation/sensor_recieve_handler.cyclo ./Core/Src/Kommunikation/sensor_recieve_handler.d ./Core/Src/Kommunikation/sensor_recieve_handler.o ./Core/Src/Kommunikation/sensor_recieve_handler.su ./Core/Src/Kommunikation/uart_data.cyclo ./Core/Src/Kommunikation/uart_data.d ./Core/Src/Kommunikation/uart_data.o ./Core/Src/Kommunikation/uart_data.su ./Core/Src/Kommunikation/uart_processor.cyclo ./Core/Src/Kommunikation/uart_processor.d ./Core/Src/Kommunikation/uart_processor.o ./Core/Src/Kommunikation/uart_processor.su

.PHONY: clean-Core-2f-Src-2f-Kommunikation

