################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ar0234.c \
../cyfxslfifosync.c \
../cyfxslfifousbdscr.c \
../cyfxtx.c \
../JY901.c \
../standalone_spi/cyfx_gpio_spi_standalone.c 

S_UPPER_SRCS += \
../cyfx_gcc_startup.S 

OBJS += \
./ar0234.o \
./cyfx_gcc_startup.o \
./cyfxslfifosync.o \
./cyfxslfifousbdscr.o \
./cyfxtx.o \
./JY901.o \
./cyfx_gpio_spi_standalone.o 

C_DEPS += \
./ar0234.d \
./cyfxslfifosync.d \
./cyfxslfifousbdscr.d \
./cyfxtx.d \
./JY901.d \
./cyfx_gpio_spi_standalone.d 

S_UPPER_DEPS += \
./cyfx_gcc_startup.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.S
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Windows GCC Assembler'
	arm-none-eabi-gcc -x assembler-with-cpp -I"$(FX3_SDK)/firmware/u3p_firmware/inc" -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=arm926ej-s -mthumb-interwork -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Windows GCC C Compiler'
	arm-none-eabi-gcc -D__CYU3P_TX__=1 -I"$(FX3_SDK)/firmware/u3p_firmware/inc" -I"..\." -Os -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=arm926ej-s -mthumb-interwork -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

cyfx_gpio_spi_standalone.o: ../standalone_spi/cyfx_gpio_spi_standalone.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Windows GCC C Compiler'
	arm-none-eabi-gcc -D__CYU3P_TX__=1 -I"$(FX3_SDK)/firmware/u3p_firmware/inc" -I"..\." -Os -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=arm926ej-s -mthumb-interwork -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


