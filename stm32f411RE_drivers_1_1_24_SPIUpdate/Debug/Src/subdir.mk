################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/STM32M_ESP32S_DuplexBus.c 

OBJS += \
./Src/STM32M_ESP32S_DuplexBus.o 

C_DEPS += \
./Src/STM32M_ESP32S_DuplexBus.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -c -I../Inc -I"D:/Courses/Embedded/STM32/Programs/STM1/stm32f411RE_drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/STM32M_ESP32S_DuplexBus.cyclo ./Src/STM32M_ESP32S_DuplexBus.d ./Src/STM32M_ESP32S_DuplexBus.o ./Src/STM32M_ESP32S_DuplexBus.su

.PHONY: clean-Src

