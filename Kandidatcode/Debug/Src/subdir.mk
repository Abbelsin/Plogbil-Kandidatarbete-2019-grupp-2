################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/DistSensor.c \
../Src/gpio.c \
../Src/i2c.c \
../Src/main.c \
../Src/motorstyrning.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_it.c \
../Src/syscalls.c \
../Src/system_stm32f4xx.c \
../Src/tcs3472.c \
../Src/varvtal.c 

OBJS += \
./Src/DistSensor.o \
./Src/gpio.o \
./Src/i2c.o \
./Src/main.o \
./Src/motorstyrning.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o \
./Src/syscalls.o \
./Src/system_stm32f4xx.o \
./Src/tcs3472.o \
./Src/varvtal.o 

C_DEPS += \
./Src/DistSensor.d \
./Src/gpio.d \
./Src/i2c.d \
./Src/main.d \
./Src/motorstyrning.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d \
./Src/syscalls.d \
./Src/system_stm32f4xx.d \
./Src/tcs3472.d \
./Src/varvtal.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F401xE -I"C:/Users/albin/Documents/stmprojects/Kandidatcode/Inc" -I"C:/Users/albin/Documents/stmprojects/Kandidatcode/VL53L0X_1.0.2/Api/core/inc" -I"C:/Users/albin/Documents/stmprojects/Kandidatcode/VL53L0X_1.0.2/Api/platform/inc" -I"C:/Users/albin/Documents/stmprojects/Kandidatcode/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/albin/Documents/stmprojects/Kandidatcode/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/albin/Documents/stmprojects/Kandidatcode/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/albin/Documents/stmprojects/Kandidatcode/Drivers/CMSIS/Include" -I"C:/Users/albin/Documents/stmprojects/Kandidatcode/Inc"  -O2 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


