################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/fx3_config.c \
../src/fx3_device.c \
../src/fx3_error.c \
../src/fx3_gpio.c \
../src/fx3_log.c \
../src/fx3_spi.c \
../src/fx3_temperature.c \
../src/fx3_usb.c \
../src/fx3_usb_vendor.c 

OBJS += \
./src/fx3_config.o \
./src/fx3_device.o \
./src/fx3_error.o \
./src/fx3_gpio.o \
./src/fx3_log.o \
./src/fx3_spi.o \
./src/fx3_temperature.o \
./src/fx3_usb.o \
./src/fx3_usb_vendor.o 

C_DEPS += \
./src/fx3_config.d \
./src/fx3_device.d \
./src/fx3_error.d \
./src/fx3_gpio.d \
./src/fx3_log.d \
./src/fx3_spi.d \
./src/fx3_temperature.d \
./src/fx3_usb.d \
./src/fx3_usb_vendor.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=arm926ej-s -marm -mthumb-interwork -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -Wall -I"$(FX3_SDK)/fw_lib/1_3_3/inc" -I"$(FX3_SDK)/../armgcc/arm-none-eabi/include" -I"../inc" -g3 -gdwarf-2 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

