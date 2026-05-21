################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := cs-rm -rf
.DEFAULT_GOAL := all

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_LIST += \
ios.lst \

SECONDARY_SIZE += \
ios.siz \


# All Target
all: ios.elf secondary-outputs

# Tool invocations

ios.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Sourcery Windows GCC C Linker'
	arm-none-eabi-gcc $(OBJS) $(USER_OBJS) $(LIBS) -T"$(FX3_SDK)/firmware/common/fx3.ld" -nostartfiles -Wl,-Map,ios.map -Wl,-d -Wl,--no-wchar-size-warning -Wl,--entry,CyU3PFirmwareEntry "$(FX3_SDK)/firmware/u3p_firmware/lib/fx3_release/cyfxapi.a" "$(FX3_SDK)/firmware/u3p_firmware/lib/fx3_release/cyu3lpp.a" "$(FX3_SDK)/firmware/u3p_firmware/lib/fx3_release/cyu3threadx.a"  "$(FX3_SDK)/ARM GCC/arm-none-eabi/lib/libc.a" "$(FX3_SDK)/ARM GCC/lib/gcc/arm-none-eabi/4.8.1/libgcc.a" -lm -mcpu=arm926ej-s -mthumb-interwork -o"ios.elf"
	@echo 'Finished building target: $@'
	@echo ' '
	@echo 'Converting ELF to image (.img) format'
	-"$(FX3_SDK)/util/elf2img/elf2img.exe" -i ios.elf -o ios.img
	@echo ' '

ios.lst: ios.elf
	@echo 'Invoking: ARM Sourcery Windows GNU Create Listing'
	arm-none-eabi-objdump -h -S ios.elf >"ios.lst"
	@echo 'Finished building: $@'
	@echo ' '

ios.siz: ios.elf
	@echo 'Invoking: ARM Sourcery Windows GNU Print Size'
	arm-none-eabi-size  --format=berkeley ios.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(SECONDARY_SIZE)$(OBJS)$(C_DEPS)$(ASM_DEPS)$(EXECUTABLES)$(SECONDARY_LIST)$(S_UPPER_DEPS) ios.elf
	-@echo ' '

post-build:
	-@echo 'Converting ELF to image (.img) format'
	-"$(FX3_SDK)/util/elf2img/elf2img.exe" -i ios.elf -o ios.img
	-@echo ' '

secondary-outputs: $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
