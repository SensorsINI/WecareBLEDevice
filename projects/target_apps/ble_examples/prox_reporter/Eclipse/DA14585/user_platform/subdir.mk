################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
E:/github_prjs/wecare_prjs/SDK_6.0.16.1144/DA145xx_SDK/6.0.16.1144/projects/target_apps/ble_examples/prox_reporter/src/platform/user_periph_setup.c 

OBJS += \
./user_platform/user_periph_setup.o 

C_DEPS += \
./user_platform/user_periph_setup.d 


# Each subdirectory must supply rules for building sources it contributes
user_platform/user_periph_setup.o: E:/github_prjs/wecare_prjs/SDK_6.0.16.1144/DA145xx_SDK/6.0.16.1144/projects/target_apps/ble_examples/prox_reporter/src/platform/user_periph_setup.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g3 -I"../../../../../../sdk/app_modules/api" -I"../../../../../../sdk/platform/include/CMSIS/5.6.0/Include" -I"../../../../../../sdk/ble_stack/controller/em" -I"../../../../../../sdk/ble_stack/controller/llc" -I"../../../../../../sdk/ble_stack/controller/lld" -I"../../../../../../sdk/ble_stack/controller/llm" -I"../../../../../../sdk/ble_stack/ea/api" -I"../../../../../../sdk/ble_stack/em/api" -I"../../../../../../sdk/ble_stack/hci/api" -I"../../../../../../sdk/ble_stack/hci/src" -I"../../../../../../sdk/ble_stack/host/att" -I"../../../../../../sdk/ble_stack/host/att/attc" -I"../../../../../../sdk/ble_stack/host/att/attm" -I"../../../../../../sdk/ble_stack/host/att/atts" -I"../../../../../../sdk/ble_stack/host/gap" -I"../../../../../../sdk/ble_stack/host/gap/gapc" -I"../../../../../../sdk/ble_stack/host/gap/gapm" -I"../../../../../../sdk/ble_stack/host/gatt" -I"../../../../../../sdk/ble_stack/host/gatt/gattc" -I"../../../../../../sdk/ble_stack/host/gatt/gattm" -I"../../../../../../sdk/ble_stack/host/l2c/l2cc" -I"../../../../../../sdk/ble_stack/host/l2c/l2cm" -I"../../../../../../sdk/ble_stack/host/smp" -I"../../../../../../sdk/ble_stack/host/smp/smpc" -I"../../../../../../sdk/ble_stack/host/smp/smpm" -I"../../../../../../sdk/ble_stack/profiles" -I"../../../../../../sdk/ble_stack/profiles/anc" -I"../../../../../../sdk/ble_stack/profiles/anc/ancc/api" -I"../../../../../../sdk/ble_stack/profiles/anp" -I"../../../../../../sdk/ble_stack/profiles/anp/anpc/api" -I"../../../../../../sdk/ble_stack/profiles/anp/anps/api" -I"../../../../../../sdk/ble_stack/profiles/bas/basc/api" -I"../../../../../../sdk/ble_stack/profiles/bas/bass/api" -I"../../../../../../sdk/ble_stack/profiles/bcs" -I"../../../../../../sdk/ble_stack/profiles/bcs/bcsc/api" -I"../../../../../../sdk/ble_stack/profiles/bcs/bcss/api" -I"../../../../../../sdk/ble_stack/profiles/blp" -I"../../../../../../sdk/ble_stack/profiles/blp/blpc/api" -I"../../../../../../sdk/ble_stack/profiles/blp/blps/api" -I"../../../../../../sdk/ble_stack/profiles/bms" -I"../../../../../../sdk/ble_stack/profiles/bms/bmsc/api" -I"../../../../../../sdk/ble_stack/profiles/bms/bmss/api" -I"../../../../../../sdk/ble_stack/profiles/cpp" -I"../../../../../../sdk/ble_stack/profiles/cpp/cppc/api" -I"../../../../../../sdk/ble_stack/profiles/cpp/cpps/api" -I"../../../../../../sdk/ble_stack/profiles/cscp" -I"../../../../../../sdk/ble_stack/profiles/cscp/cscpc/api" -I"../../../../../../sdk/ble_stack/profiles/cscp/cscps/api" -I"../../../../../../sdk/ble_stack/profiles/cts" -I"../../../../../../sdk/ble_stack/profiles/cts/ctsc/api" -I"../../../../../../sdk/ble_stack/profiles/cts/ctss/api" -I"../../../../../../sdk/ble_stack/profiles/custom" -I"../../../../../../sdk/ble_stack/profiles/custom/custs/api" -I"../../../../../../sdk/ble_stack/profiles/dis/disc/api" -I"../../../../../../sdk/ble_stack/profiles/dis/diss/api" -I"../../../../../../sdk/ble_stack/profiles/find" -I"../../../../../../sdk/ble_stack/profiles/find/findl/api" -I"../../../../../../sdk/ble_stack/profiles/find/findt/api" -I"../../../../../../sdk/ble_stack/profiles/gatt/gatt_client/api" -I"../../../../../../sdk/ble_stack/profiles/glp" -I"../../../../../../sdk/ble_stack/profiles/glp/glpc/api" -I"../../../../../../sdk/ble_stack/profiles/glp/glps/api" -I"../../../../../../sdk/ble_stack/profiles/hogp" -I"../../../../../../sdk/ble_stack/profiles/hogp/hogpbh/api" -I"../../../../../../sdk/ble_stack/profiles/hogp/hogpd/api" -I"../../../../../../sdk/ble_stack/profiles/hogp/hogprh/api" -I"../../../../../../sdk/ble_stack/profiles/hrp" -I"../../../../../../sdk/ble_stack/profiles/hrp/hrpc/api" -I"../../../../../../sdk/ble_stack/profiles/hrp/hrps/api" -I"../../../../../../sdk/ble_stack/profiles/htp" -I"../../../../../../sdk/ble_stack/profiles/htp/htpc/api" -I"../../../../../../sdk/ble_stack/profiles/htp/htpt/api" -I"../../../../../../sdk/ble_stack/profiles/lan" -I"../../../../../../sdk/ble_stack/profiles/lan/lanc/api" -I"../../../../../../sdk/ble_stack/profiles/lan/lans/api" -I"../../../../../../sdk/ble_stack/profiles/pasp" -I"../../../../../../sdk/ble_stack/profiles/pasp/paspc/api" -I"../../../../../../sdk/ble_stack/profiles/pasp/pasps/api" -I"../../../../../../sdk/ble_stack/profiles/prox/proxm/api" -I"../../../../../../sdk/ble_stack/profiles/prox/proxr/api" -I"../../../../../../sdk/ble_stack/profiles/rscp" -I"../../../../../../sdk/ble_stack/profiles/rscp/rscpc/api" -I"../../../../../../sdk/ble_stack/profiles/rscp/rscps/api" -I"../../../../../../sdk/ble_stack/profiles/scpp" -I"../../../../../../sdk/ble_stack/profiles/scpp/scppc/api" -I"../../../../../../sdk/ble_stack/profiles/scpp/scpps/api" -I"../../../../../../sdk/ble_stack/profiles/suota/suotar/api" -I"../../../../../../sdk/ble_stack/profiles/tip" -I"../../../../../../sdk/ble_stack/profiles/tip/tipc/api" -I"../../../../../../sdk/ble_stack/profiles/tip/tips/api" -I"../../../../../../sdk/ble_stack/profiles/uds" -I"../../../../../../sdk/ble_stack/profiles/uds/udsc/api" -I"../../../../../../sdk/ble_stack/profiles/uds/udss/api" -I"../../../../../../sdk/ble_stack/profiles/wss" -I"../../../../../../sdk/ble_stack/profiles/wss/wssc/api" -I"../../../../../../sdk/ble_stack/profiles/wss/wsss/api" -I"../../../../../../sdk/ble_stack/rwble" -I"../../../../../../sdk/ble_stack/rwble_hl" -I"../../../../../../sdk/common_project_files" -I"../../../../../../sdk/platform/arch" -I"../../../../../../sdk/platform/arch/boot" -I"../../../../../../sdk/platform/arch/boot/ARM" -I"../../../../../../sdk/platform/arch/boot/GCC" -I"../../../../../../sdk/platform/arch/compiler" -I"../../../../../../sdk/platform/arch/compiler/ARM" -I"../../../../../../sdk/platform/arch/compiler/GCC" -I"../../../../../../sdk/platform/arch/ll" -I"../../../../../../sdk/platform/arch/main" -I"../../../../../../sdk/platform/core_modules/arch_console" -I"../../../../../../sdk/platform/core_modules/common/api" -I"../../../../../../sdk/platform/core_modules/crypto" -I"../../../../../../sdk/platform/core_modules/dbg/api" -I"../../../../../../sdk/platform/core_modules/gtl/api" -I"../../../../../../sdk/platform/core_modules/gtl/src" -I"../../../../../../sdk/platform/core_modules/h4tl/api" -I"../../../../../../sdk/platform/core_modules/ke/api" -I"../../../../../../sdk/platform/core_modules/ke/src" -I"../../../../../../sdk/platform/core_modules/nvds/api" -I"../../../../../../sdk/platform/core_modules/rf/api" -I"../../../../../../sdk/platform/core_modules/rwip/api" -I"../../../../../../sdk/platform/driver/adc" -I"../../../../../../sdk/platform/driver/battery" -I"../../../../../../sdk/platform/driver/ble" -I"../../../../../../sdk/platform/driver/dma" -I"../../../../../../sdk/platform/driver/gpio" -I"../../../../../../sdk/platform/driver/hw_otpc" -I"../../../../../../sdk/platform/driver/i2c" -I"../../../../../../sdk/platform/driver/i2c_eeprom" -I"../../../../../../sdk/platform/driver/pdm" -I"../../../../../../sdk/platform/driver/reg" -I"../../../../../../sdk/platform/driver/rtc" -I"../../../../../../sdk/platform/driver/spi" -I"../../../../../../sdk/platform/driver/spi_flash" -I"../../../../../../sdk/platform/driver/spi_hci" -I"../../../../../../sdk/platform/driver/syscntl" -I"../../../../../../sdk/platform/driver/systick" -I"../../../../../../sdk/platform/driver/timer" -I"../../../../../../sdk/platform/driver/trng" -I"../../../../../../sdk/platform/driver/uart" -I"../../../../../../sdk/platform/driver/wkupct_quadec" -I"../../../../../../sdk/platform/include" -I"../../../../../../sdk/platform/system_library/include" -I"../../../../../../third_party/hash" -I"../../../../../../third_party/rand" -I"../../../../../../sdk/platform/utilities/otp_hdr" -I"E:/github_prjs/wecare_prjs/SDK_6.0.16.1144/DA145xx_SDK/6.0.16.1144/projects/target_apps/ble_examples/prox_reporter/Eclipse/../src" -I"E:/github_prjs/wecare_prjs/SDK_6.0.16.1144/DA145xx_SDK/6.0.16.1144/projects/target_apps/ble_examples/prox_reporter/Eclipse/../src/config" -include"E:\github_prjs\wecare_prjs\SDK_6.0.16.1144\DA145xx_SDK\6.0.16.1144\projects\target_apps\ble_examples\prox_reporter\src\config\da1458x_config_basic.h" -include"E:\github_prjs\wecare_prjs\SDK_6.0.16.1144\DA145xx_SDK\6.0.16.1144\projects\target_apps\ble_examples\prox_reporter\src\config\da1458x_config_advanced.h" -include"E:\github_prjs\wecare_prjs\SDK_6.0.16.1144\DA145xx_SDK\6.0.16.1144\projects\target_apps\ble_examples\prox_reporter\src\config\user_config.h" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


