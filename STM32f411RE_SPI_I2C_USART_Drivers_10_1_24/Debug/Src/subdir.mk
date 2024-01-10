################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/006STM32_ESP32_RxTx_Interrupt_UART.c \
../Src/sysmem.c 

OBJS += \
./Src/006STM32_ESP32_RxTx_Interrupt_UART.o \
./Src/sysmem.o 

C_DEPS += \
./Src/006STM32_ESP32_RxTx_Interrupt_UART.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -c -I../Inc -I"D:/Courses/Embedded/STM32/Programs/STM1/STM32f411RE_I2C_Drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/006STM32_ESP32_RxTx_Interrupt_UART.cyclo ./Src/006STM32_ESP32_RxTx_Interrupt_UART.d ./Src/006STM32_ESP32_RxTx_Interrupt_UART.o ./Src/006STM32_ESP32_RxTx_Interrupt_UART.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

