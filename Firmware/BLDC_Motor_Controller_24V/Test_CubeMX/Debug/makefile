################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
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
Test_CubeMX.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Test_CubeMX.list \

OBJCOPY_BIN += \
Test_CubeMX.bin \


# All Target
all: Test_CubeMX.elf secondary-outputs

# Tool invocations
Test_CubeMX.elf: $(OBJS) $(USER_OBJS) C:\Users\rkdgu\Documents\Projects\24V\ BLDC\ Motor\ Controller\Firmware\BLDC_Motor_Controller_24V\Test_CubeMX\STM32F103RCTX_FLASH.ld
	arm-none-eabi-gcc -o "Test_CubeMX.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\rkdgu\Documents\Projects\24V BLDC Motor Controller\Firmware\BLDC_Motor_Controller_24V\Test_CubeMX\STM32F103RCTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Test_CubeMX.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Test_CubeMX.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Test_CubeMX.list"
	@echo 'Finished building: $@'
	@echo ' '

Test_CubeMX.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Test_CubeMX.bin"
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
