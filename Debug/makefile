################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F0xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
NucleoTestPrj.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
NucleoTestPrj.list \

OBJCOPY_BIN += \
NucleoTestPrj.bin \


# All Target
all: NucleoTestPrj.elf secondary-outputs

# Tool invocations
NucleoTestPrj.elf: $(OBJS) $(USER_OBJS) D:\stm_workspace\NucleoTestPrj\STM32F030R8TX_FLASH.ld
	arm-none-eabi-gcc -o "NucleoTestPrj.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"D:\stm_workspace\NucleoTestPrj\STM32F030R8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="NucleoTestPrj.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

NucleoTestPrj.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "NucleoTestPrj.list"
	@echo 'Finished building: $@'
	@echo ' '

NucleoTestPrj.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "NucleoTestPrj.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
