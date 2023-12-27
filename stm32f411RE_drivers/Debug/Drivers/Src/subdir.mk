################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/STM32F411RE_Driver_GPIO.c 

OBJS += \
./Drivers/Src/STM32F411RE_Driver_GPIO.o 

C_DEPS += \
./Drivers/Src/STM32F411RE_Driver_GPIO.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/%.o Drivers/Src/%.su Drivers/Src/%.cyclo: ../Drivers/Src/%.c Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -c -I../Inc -I"D:/Courses/Embedded/STM32/Programs/STM1/stm32f411RE_drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Src

clean-Drivers-2f-Src:
	-$(RM) ./Drivers/Src/STM32F411RE_Driver_GPIO.cyclo ./Drivers/Src/STM32F411RE_Driver_GPIO.d ./Drivers/Src/STM32F411RE_Driver_GPIO.o ./Drivers/Src/STM32F411RE_Driver_GPIO.su

.PHONY: clean-Drivers-2f-Src

