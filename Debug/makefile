################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := Telemetry_1.4.1_LTE-M
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Telemetry_1.4.1_LTE-M.elf \

MAP_FILES += \
Telemetry_1.4.1_LTE-M.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Telemetry_1.4.1_LTE-M.list \


# All Target
all: main-build

# Main-build Target
main-build: Telemetry_1.4.1_LTE-M.elf secondary-outputs

# Tool invocations
Telemetry_1.4.1_LTE-M.elf Telemetry_1.4.1_LTE-M.map: $(OBJS) $(USER_OBJS) /home/a731/STM32CubeIDE/workspace_1.2.0/Telemetry_1.4.1_LTE-M/STM32F446RETX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Telemetry_1.4.1_LTE-M.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/a731/STM32CubeIDE/workspace_1.2.0/Telemetry_1.4.1_LTE-M/STM32F446RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Telemetry_1.4.1_LTE-M.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Telemetry_1.4.1_LTE-M.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Telemetry_1.4.1_LTE-M.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Telemetry_1.4.1_LTE-M.elf Telemetry_1.4.1_LTE-M.list Telemetry_1.4.1_LTE-M.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
