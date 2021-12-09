################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include user_platform/subdir.mk
-include user_app/subdir.mk
-include sdk_profiles/subdir.mk
-include sdk_driver/subdir.mk
-include sdk_boot/subdir.mk
-include sdk_ble/subdir.mk
-include sdk_arch/subdir.mk
-include sdk_app/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
barebone_585.hex \

SECONDARY_SIZE += \
barebone_585.siz \


# All Target
all:
	$(MAKE) --no-print-directory pre-build
	$(MAKE) --no-print-directory main-build

# Main-build Target
main-build: barebone_585.elf secondary-outputs

# Tool invocations
barebone_585.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross ARM C Linker'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g3 -T "ldscript_DA14585_586.lds" -Xlinker --gc-sections -L"../../../../../../sdk/platform/system_library/output/Keil_5" -L"E:\github_prjs\wecare_prjs\SDK_6.0.16.1144\DA145xx_SDK\6.0.16.1144\projects\target_apps\ble_examples\ble_app_barebone\Eclipse/DA14585" -L../../../../../.././/sdk/common_project_files/misc -Wl,-Map,"barebone_585.map" --specs=nano.specs --specs=nosys.specs -v -Wl,--no-wchar-size-warning -o "barebone_585.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

barebone_585.hex: barebone_585.elf
	@echo 'Invoking: Cross ARM GNU Create Flash Image'
	arm-none-eabi-objcopy -O ihex "barebone_585.elf"  "barebone_585.hex"
	@echo 'Finished building: $@'
	@echo ' '

barebone_585.siz: barebone_585.elf
	@echo 'Invoking: Cross ARM GNU Print Size'
	arm-none-eabi-size --format=berkeley "barebone_585.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(SECONDARY_FLASH)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_UPPER_DEPS)$(C_DEPS) barebone_585.elf
	-@echo ' '

pre-build:
	-make generate_ldscripts PRE_BUILD_EXTRA_DEFS="" CC="arm-none-eabi-gcc" LDSCRIPT_INCLUDE_DIR="../../src/config ../../../../../.././/sdk/common_project_files" LDSCRIPT_PATH="../../../../../.././/sdk/common_project_files/ldscripts" LDSCRIPT_FILES="ldscript_DA14585_586.lds"
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_SIZE)

.PHONY: all clean dependents main-build pre-build

-include ../makefile.targets
