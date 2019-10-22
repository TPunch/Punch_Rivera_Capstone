################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/main.c \
../Src/stm32l4xx_hal_msp.c \
../Src/stm32l4xx_it.c \
../Src/syscalls.c \
../Src/system_stm32l4xx.c 

OBJS += \
./Src/main.o \
./Src/stm32l4xx_hal_msp.o \
./Src/stm32l4xx_it.o \
./Src/syscalls.o \
./Src/system_stm32l4xx.o 

C_DEPS += \
./Src/main.d \
./Src/stm32l4xx_hal_msp.d \
./Src/stm32l4xx_it.d \
./Src/syscalls.d \
./Src/system_stm32l4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32L433xx -I"D:/ECE486_Labs/Cap/Punch_Rivera_Capstone/ADXL362 Testing/L433RC Nucleo/Inc" -I"D:/ECE486_Labs/Cap/Punch_Rivera_Capstone/ADXL362 Testing/L433RC Nucleo/Drivers/STM32L4xx_HAL_Driver/Inc" -I"D:/ECE486_Labs/Cap/Punch_Rivera_Capstone/ADXL362 Testing/L433RC Nucleo/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"D:/ECE486_Labs/Cap/Punch_Rivera_Capstone/ADXL362 Testing/L433RC Nucleo/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"D:/ECE486_Labs/Cap/Punch_Rivera_Capstone/ADXL362 Testing/L433RC Nucleo/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


