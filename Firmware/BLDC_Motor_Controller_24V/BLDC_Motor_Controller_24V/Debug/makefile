################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/Src/subdir.mk
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
BLDC_Motor_Controller_24V.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
BLDC_Motor_Controller_24V.list \

OBJCOPY_BIN += \
BLDC_Motor_Controller_24V.bin \


# All Target
all: BLDC_Motor_Controller_24V.elf secondary-outputs

# Tool invocations
BLDC_Motor_Controller_24V.elf: $(OBJS) $(USER_OBJS) C:\Users\rkdgu\Documents\Projects\24V\ BLDC\ Motor\ Controller\Firmware\BLDC_Motor_Controller_24V\BLDC_Motor_Controller_24V\STM32F103RCTX_FLASH.ld
	arm-none-eabi-gcc -o "BLDC_Motor_Controller_24V.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\rkdgu\Documents\Projects\24V BLDC Motor Controller\Firmware\BLDC_Motor_Controller_24V\BLDC_Motor_Controller_24V\STM32F103RCTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="BLDC_Motor_Controller_24V.map" -Wl,--gc-sections -static -u _printf_float --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

BLDC_Motor_Controller_24V.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "BLDC_Motor_Controller_24V.list"
	@echo 'Finished building: $@'
	@echo ' '

BLDC_Motor_Controller_24V.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "BLDC_Motor_Controller_24V.bin"
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
