################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/test_function.c 

OBJS += \
./Core/Inc/test_function.o 

C_DEPS += \
./Core/Inc/test_function.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/%.o Core/Inc/%.su Core/Inc/%.cyclo: ../Core/Inc/%.c Core/Inc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/oakyol/STM32CubeIDE/workspace_1.15.0/Uart_Driver/Core/Src" -I"C:/Users/oakyol/STM32CubeIDE/workspace_1.15.0/Uart_Driver/Core/Inc" -I"C:/Users/oakyol/STM32CubeIDE/workspace_1.15.0/Uart_Driver/Core/Src" -I"C:/Users/oakyol/STM32CubeIDE/workspace_1.15.0/Uart_Driver/Core/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc

clean-Core-2f-Inc:
	-$(RM) ./Core/Inc/test_function.cyclo ./Core/Inc/test_function.d ./Core/Inc/test_function.o ./Core/Inc/test_function.su

.PHONY: clean-Core-2f-Inc

