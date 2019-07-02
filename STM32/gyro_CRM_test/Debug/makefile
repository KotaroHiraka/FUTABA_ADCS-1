################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
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
gyro_CRM_test.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
gyro_CRM_test.list \


# All Target
all: gyro_CRM_test.elf secondary-outputs

# Tool invocations
gyro_CRM_test.elf: $(OBJS) $(USER_OBJS) D:\Programming\FUTABA_ADCS\STM32\gyro_CRM_test\STM32F446RETX_FLASH.ld
	arm-none-eabi-gcc -o "gyro_CRM_test.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\Programming\FUTABA_ADCS\STM32\gyro_CRM_test\STM32F446RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="gyro_CRM_test.map" -Wl,--gc-sections -static -u _printf_float --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

gyro_CRM_test.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "gyro_CRM_test.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
