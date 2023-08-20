################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/gpio.c \
../Src/lcd.c \
../Src/main.c \
../Src/nvic.c \
../Src/syscalls.c \
../Src/syscfg.c \
../Src/sysmem.c \
../Src/systick.c 

OBJS += \
./Src/gpio.o \
./Src/lcd.o \
./Src/main.o \
./Src/nvic.o \
./Src/syscalls.o \
./Src/syscfg.o \
./Src/sysmem.o \
./Src/systick.o 

C_DEPS += \
./Src/gpio.d \
./Src/lcd.d \
./Src/main.d \
./Src/nvic.d \
./Src/syscalls.d \
./Src/syscfg.d \
./Src/sysmem.d \
./Src/systick.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F0 -DSTM32F051C6Tx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/gpio.cyclo ./Src/gpio.d ./Src/gpio.o ./Src/gpio.su ./Src/lcd.cyclo ./Src/lcd.d ./Src/lcd.o ./Src/lcd.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/nvic.cyclo ./Src/nvic.d ./Src/nvic.o ./Src/nvic.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/syscfg.cyclo ./Src/syscfg.d ./Src/syscfg.o ./Src/syscfg.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/systick.cyclo ./Src/systick.d ./Src/systick.o ./Src/systick.su

.PHONY: clean-Src

