################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/STM32F411RE_Driver_GPIO.c \
../Drivers/Src/STM32F411RE_Driver_I2C.c \
../Drivers/Src/STM32F411RE_Driver_SPI.c 

OBJS += \
./Drivers/Src/STM32F411RE_Driver_GPIO.o \
./Drivers/Src/STM32F411RE_Driver_I2C.o \
./Drivers/Src/STM32F411RE_Driver_SPI.o 

C_DEPS += \
./Drivers/Src/STM32F411RE_Driver_GPIO.d \
./Drivers/Src/STM32F411RE_Driver_I2C.d \
./Drivers/Src/STM32F411RE_Driver_SPI.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/%.o Drivers/Src/%.su Drivers/Src/%.cyclo: ../Drivers/Src/%.c Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -c -I../Inc -I"D:/Courses/Embedded/STM32/Programs/STM1/STM32f411RE_I2C_Drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Src

clean-Drivers-2f-Src:
	-$(RM) ./Drivers/Src/STM32F411RE_Driver_GPIO.cyclo ./Drivers/Src/STM32F411RE_Driver_GPIO.d ./Drivers/Src/STM32F411RE_Driver_GPIO.o ./Drivers/Src/STM32F411RE_Driver_GPIO.su ./Drivers/Src/STM32F411RE_Driver_I2C.cyclo ./Drivers/Src/STM32F411RE_Driver_I2C.d ./Drivers/Src/STM32F411RE_Driver_I2C.o ./Drivers/Src/STM32F411RE_Driver_I2C.su ./Drivers/Src/STM32F411RE_Driver_SPI.cyclo ./Drivers/Src/STM32F411RE_Driver_SPI.d ./Drivers/Src/STM32F411RE_Driver_SPI.o ./Drivers/Src/STM32F411RE_Driver_SPI.su

.PHONY: clean-Drivers-2f-Src

