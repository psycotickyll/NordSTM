################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Sensors/lis2mdl-pid/lis2mdl_reg.c 

OBJS += \
./Drivers/Sensors/lis2mdl-pid/lis2mdl_reg.o 

C_DEPS += \
./Drivers/Sensors/lis2mdl-pid/lis2mdl_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Sensors/lis2mdl-pid/%.o Drivers/Sensors/lis2mdl-pid/%.su Drivers/Sensors/lis2mdl-pid/%.cyclo: ../Drivers/Sensors/lis2mdl-pid/%.c Drivers/Sensors/lis2mdl-pid/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L152xE -c -I../Core/Inc -I"C:/Users/kyli8/Downloads/NordSTM/Drivers/Sensors/lis2mdl-pid" -I../Drivers/STM32L1xx_HAL_Driver/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/kyli8/Downloads/NordSTM/Drivers/display" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-Sensors-2f-lis2mdl-2d-pid

clean-Drivers-2f-Sensors-2f-lis2mdl-2d-pid:
	-$(RM) ./Drivers/Sensors/lis2mdl-pid/lis2mdl_reg.cyclo ./Drivers/Sensors/lis2mdl-pid/lis2mdl_reg.d ./Drivers/Sensors/lis2mdl-pid/lis2mdl_reg.o ./Drivers/Sensors/lis2mdl-pid/lis2mdl_reg.su

.PHONY: clean-Drivers-2f-Sensors-2f-lis2mdl-2d-pid

