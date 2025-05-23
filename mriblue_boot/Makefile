PROJECT_NAME     := mriblue_boot
TARGETS          := mriblue_boot
OUTPUT_DIRECTORY := _build

SDK_ROOT := ../nrfSDK
NRF_TOOL_PATH := ../nrfTools
PROJ_DIR := .
MRI_PATH := ../mri

$(OUTPUT_DIRECTORY)/mriblue_boot.out: \
  LINKER_SCRIPT  := nrf52.ld

# Source files common to all targets
SRC_FILES += \
  $(SDK_ROOT)/components/libraries/util/app_error.c \
  $(SDK_ROOT)/components/libraries/timer/app_timer.c \
  $(SDK_ROOT)/components/libraries/util/app_util_platform.c \
  $(SDK_ROOT)/components/libraries/fds/fds.c \
  $(SDK_ROOT)/components/libraries/fstorage/fstorage.c \
  $(SDK_ROOT)/components/libraries/util/nrf_assert.c \
  $(SDK_ROOT)/components/libraries/util/sdk_mapped_flags.c \
  $(SDK_ROOT)/components/drivers_nrf/clock/nrf_drv_clock.c \
  $(SDK_ROOT)/components/drivers_nrf/common/nrf_drv_common.c \
  $(SDK_ROOT)/components/ble/common/ble_advdata.c \
  $(SDK_ROOT)/components/ble/ble_advertising/ble_advertising.c \
  $(SDK_ROOT)/components/ble/common/ble_conn_params.c \
  $(SDK_ROOT)/components/ble/common/ble_conn_state.c \
  $(SDK_ROOT)/components/ble/common/ble_srv_common.c \
  $(SDK_ROOT)/components/ble/peer_manager/gatt_cache_manager.c \
  $(SDK_ROOT)/components/ble/peer_manager/gatts_cache_manager.c \
  $(SDK_ROOT)/components/ble/peer_manager/id_manager.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_data.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_data_storage.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_database.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_id.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_manager.c \
  $(SDK_ROOT)/components/ble/peer_manager/pm_buffer.c \
  $(SDK_ROOT)/components/ble/peer_manager/pm_mutex.c \
  $(SDK_ROOT)/components/ble/peer_manager/security_dispatcher.c \
  $(SDK_ROOT)/components/ble/peer_manager/security_manager.c \
  $(SDK_ROOT)/components/toolchain/system_nrf52.c \
  $(SDK_ROOT)/components/softdevice/common/softdevice_handler/softdevice_handler.c \
  $(SDK_ROOT)/components/ble/ble_services/ble_nus/ble_nus.c \
  $(SDK_ROOT)/components/drivers_nrf/timer/nrf_drv_timer.c \
  $(SDK_ROOT)/components/libraries/simple_timer/app_simple_timer.c \
  $(SDK_ROOT)/components/drivers_nrf/hal/nrf_nvmc.c \
  $(PROJ_DIR)/main.c \
  $(PROJ_DIR)/CircularQueue.c \
  $(PROJ_DIR)/gcc_startup_nrf52.S \

# Include folders common to all targets
INC_FOLDERS += \
  $(PROJ_DIR) \
  $(MRI_PATH) \
  $(MRI_PATH)/cmsis \
  $(MRI_PATH)/cmsis/NRF52 \
  $(SDK_ROOT)/components/drivers_nrf/comp \
  $(SDK_ROOT)/components/drivers_nrf/twi_master \
  $(SDK_ROOT)/components/ble/ble_services/ble_ancs_c \
  $(SDK_ROOT)/components/ble/ble_services/ble_ias_c \
  $(SDK_ROOT)/components/softdevice/s132/headers \
  $(SDK_ROOT)/components/libraries/pwm \
  $(SDK_ROOT)/components/libraries/usbd/class/cdc/acm \
  $(SDK_ROOT)/components/libraries/usbd/class/hid/generic \
  $(SDK_ROOT)/components/libraries/usbd/class/msc \
  $(SDK_ROOT)/components/libraries/usbd/class/hid \
  $(SDK_ROOT)/components/libraries/log \
  $(SDK_ROOT)/components/libraries/atomic \
  $(SDK_ROOT)/components/ble/ble_services/ble_gls \
  $(SDK_ROOT)/components/libraries/fstorage \
  $(SDK_ROOT)/components/drivers_nrf/i2s \
  $(SDK_ROOT)/components/libraries/gpiote \
  $(SDK_ROOT)/components/drivers_nrf/gpiote \
  $(SDK_ROOT)/components/boards \
  $(SDK_ROOT)/components/drivers_nrf/common \
  $(SDK_ROOT)/components/ble/ble_advertising \
  $(SDK_ROOT)/components/drivers_nrf/adc \
  $(SDK_ROOT)/components/softdevice/s132/headers/nrf52 \
  $(SDK_ROOT)/components/ble/ble_services/ble_bas_c \
  $(SDK_ROOT)/components/ble/ble_services/ble_hrs_c \
  $(SDK_ROOT)/components/libraries/queue \
  $(SDK_ROOT)/components/ble/ble_dtm \
  $(SDK_ROOT)/components/toolchain/cmsis/include \
  $(SDK_ROOT)/components/ble/ble_services/ble_rscs_c \
  $(SDK_ROOT)/components/drivers_nrf/uart \
  $(SDK_ROOT)/components/ble/common \
  $(SDK_ROOT)/components/ble/ble_services/ble_lls \
  $(SDK_ROOT)/components/drivers_nrf/wdt \
  $(SDK_ROOT)/components/libraries/bsp \
  $(SDK_ROOT)/components/ble/ble_services/ble_bas \
  $(SDK_ROOT)/components/libraries/experimental_section_vars \
  $(SDK_ROOT)/components/ble/ble_services/ble_ans_c \
  $(SDK_ROOT)/components/libraries/slip \
  $(SDK_ROOT)/components/libraries/mem_manager \
  $(SDK_ROOT)/external/segger_rtt \
  $(SDK_ROOT)/components/libraries/csense_drv \
  $(SDK_ROOT)/components/drivers_nrf/hal \
  $(SDK_ROOT)/components/ble/ble_services/ble_nus_c \
  $(SDK_ROOT)/components/drivers_nrf/rtc \
  $(SDK_ROOT)/components/ble/ble_services/ble_ias \
  $(SDK_ROOT)/components/libraries/usbd/class/hid/mouse \
  $(SDK_ROOT)/components/drivers_nrf/ppi \
  $(SDK_ROOT)/components/ble/ble_services/ble_dfu \
  $(SDK_ROOT)/components/drivers_nrf/twis_slave \
  $(SDK_ROOT)/components \
  $(SDK_ROOT)/components/libraries/scheduler \
  $(SDK_ROOT)/components/ble/ble_services/ble_lbs \
  $(SDK_ROOT)/components/ble/ble_services/ble_hts \
  $(SDK_ROOT)/components/drivers_nrf/delay \
  $(SDK_ROOT)/components/libraries/crc16 \
  $(SDK_ROOT)/components/drivers_nrf/timer \
  $(SDK_ROOT)/components/libraries/util \
  $(SDK_ROOT)/components/drivers_nrf/pwm \
  $(SDK_ROOT)/components/libraries/usbd/class/cdc \
  $(SDK_ROOT)/components/libraries/csense \
  $(SDK_ROOT)/components/drivers_nrf/rng \
  $(SDK_ROOT)/components/libraries/low_power_pwm \
  $(SDK_ROOT)/components/libraries/hardfault \
  $(SDK_ROOT)/components/ble/ble_services/ble_cscs \
  $(SDK_ROOT)/components/libraries/uart \
  $(SDK_ROOT)/components/libraries/hci \
  $(SDK_ROOT)/components/libraries/usbd/class/hid/kbd \
  $(SDK_ROOT)/components/drivers_nrf/spi_slave \
  $(SDK_ROOT)/components/drivers_nrf/lpcomp \
  $(SDK_ROOT)/components/libraries/timer \
  $(SDK_ROOT)/components/drivers_nrf/power \
  $(SDK_ROOT)/components/libraries/usbd/config \
  $(SDK_ROOT)/components/toolchain \
  $(SDK_ROOT)/components/libraries/led_softblink \
  $(SDK_ROOT)/components/drivers_nrf/qdec \
  $(SDK_ROOT)/components/ble/ble_services/ble_cts_c \
  $(SDK_ROOT)/components/drivers_nrf/spi_master \
  $(SDK_ROOT)/components/ble/ble_services/ble_nus \
  $(SDK_ROOT)/components/ble/ble_services/ble_hids \
  $(SDK_ROOT)/components/drivers_nrf/pdm \
  $(SDK_ROOT)/components/libraries/crc32 \
  $(SDK_ROOT)/components/libraries/usbd/class/audio \
  $(SDK_ROOT)/components/libraries/sensorsim \
  $(SDK_ROOT)/components/ble/peer_manager \
  $(SDK_ROOT)/components/drivers_nrf/swi \
  $(SDK_ROOT)/components/ble/ble_services/ble_tps \
  $(SDK_ROOT)/components/ble/ble_services/ble_dis \
  $(SDK_ROOT)/components/device \
  $(SDK_ROOT)/components/ble/nrf_ble_qwr \
  $(SDK_ROOT)/components/libraries/button \
  $(SDK_ROOT)/components/libraries/usbd \
  $(SDK_ROOT)/components/drivers_nrf/saadc \
  $(SDK_ROOT)/components/ble/ble_services/ble_lbs_c \
  $(SDK_ROOT)/components/ble/ble_racp \
  $(SDK_ROOT)/components/toolchain/gcc \
  $(SDK_ROOT)/components/libraries/fds \
  $(SDK_ROOT)/components/libraries/twi \
  $(SDK_ROOT)/components/drivers_nrf/clock \
  $(SDK_ROOT)/components/ble/ble_services/ble_rscs \
  $(SDK_ROOT)/components/drivers_nrf/usbd \
  $(SDK_ROOT)/components/softdevice/common/softdevice_handler \
  $(SDK_ROOT)/components/ble/ble_services/ble_hrs \
  $(SDK_ROOT)/components/libraries/log/src \
  $(SDK_ROOT)/components/libraries/simple_timer \

# Libraries common to all targets
# Add in MRI debug monitor.
LIB_FILES += $(MRI_PATH)/lib/armv7-m/libmri_nrf52dk_fpu_hard.a

# C flags common to all targets
CFLAGS += -D__HEAP_SIZE=0
CFLAGS += -DDEBUG_NRF_USER
CFLAGS += -DNRF52
CFLAGS += -DNRF52_PAN_64
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DBOARD_PCA10040
CFLAGS += -DNRF52832
CFLAGS += -DNRF52_PAN_12
CFLAGS += -DNRF52_PAN_58
CFLAGS += -DNRF52_PAN_54
CFLAGS += -DNRF52_PAN_31
CFLAGS += -DNRF52_PAN_51
CFLAGS += -DNRF52_PAN_36
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DNRF52_PAN_15
CFLAGS += -DNRF_SD_BLE_API_VERSION=3
CFLAGS += -DSWI_DISABLE0
CFLAGS += -DNRF52_PAN_20
CFLAGS += -DNRF52_PAN_55
CFLAGS += -DS132
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS += -Wall -Werror -Os -g3
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# keep every function in separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums

# Disabling inlining tends to make debugging easier.
#CFLAGS += -fno-inline

# Pull in priv_config.h if it exists.
PRIV_CONFIG := $(wildcard priv_config.h)
ifneq "$(PRIV_CONFIG)" ""
    CFLAGS += -include $(PRIV_CONFIG)
endif

# C++ flags common to all targets
CXXFLAGS += -fno-rtti

# Assembler flags common to all targets
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -D__HEAP_SIZE=0
ASMFLAGS += -DNRF52
ASMFLAGS += -DNRF52_PAN_64
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DBOARD_PCA10040
ASMFLAGS += -DNRF52832
ASMFLAGS += -DNRF52_PAN_12
ASMFLAGS += -DNRF52_PAN_58
ASMFLAGS += -DNRF52_PAN_54
ASMFLAGS += -DNRF52_PAN_31
ASMFLAGS += -DNRF52_PAN_51
ASMFLAGS += -DNRF52_PAN_36
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DNRF52_PAN_15
ASMFLAGS += -DNRF_SD_BLE_API_VERSION=3
ASMFLAGS += -DSWI_DISABLE0
ASMFLAGS += -DNRF52_PAN_20
ASMFLAGS += -DNRF52_PAN_55
ASMFLAGS += -DS132
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb -mabi=aapcs
ASMFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16

# Linker flags
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys
LDFLAGS += -Wl,--allow-multiple-definition
LDFLAGS += -Wl,--wrap=mriPlatform_EnteringDebugger
LDFLAGS += -Wl,--wrap=mriPlatform_MemRead32
LDFLAGS += -Wl,--wrap=mriPlatform_MemRead16
LDFLAGS += -Wl,--wrap=mriPlatform_MemRead8
LDFLAGS += -Wl,--wrap=mriPlatform_MemWrite32
LDFLAGS += -Wl,--wrap=mriPlatform_MemWrite16
LDFLAGS += -Wl,--wrap=mriPlatform_MemWrite8
LDFLAGS += -Wl,--wrap=mriPlatform_EnableSingleStep
LDFLAGS += -Wl,--wrap=mriPlatform_SetHardwareBreakpointOfGdbKind
LDFLAGS += -Wl,--wrap=mriPlatform_SetHardwareBreakpoint
LDFLAGS += -Wl,--wrap=mriPlatform_ClearHardwareBreakpointOfGdbKind
LDFLAGS += -Wl,--wrap=mriPlatform_ClearHardwareBreakpoint
LDFLAGS += -Wl,--wrap=mriPlatform_SetHardwareWatchpoint
LDFLAGS += -Wl,--wrap=mriPlatform_ClearHardwareWatchpoint
LDFLAGS += -Wl,--wrap=mriSemihost_WriteToFileOrConsole
LDFLAGS += -Wl,--wrap=mriPlatform_ResetDevice

.PHONY: $(TARGETS) default all clean help flash flash_softdevice sdk mri

# Default target - first one defined
default: mriblue_boot

# Print all targets that can be built
help:
	@echo The following rules are available:
	@echo   sdk - Needs to be run first to install Nordic SDK, required for build.
	@echo   mri - Make sure that the mri submodule prereq is initialized and built.
	@echo   clean - Deletes all output binaries so that next build is performed from scratch.
	@echo   all - Builds all supported targets - nrf52832_xxaa.
	@echo   erase - Erases all of the FLASH on the nRF5 device.
	@echo   flash_softdevice - Flashes the Nordic BLE Soft Device. Will erase device first.
	@echo   flash - Just flashes your application firmware. flash_softdevice must be run at least once before.
	@echo 	mriblue_boot

TEMPLATE_PATH := $(SDK_ROOT)/components/toolchain/gcc

-include $(TEMPLATE_PATH)/Makefile.common

$(foreach target, $(TARGETS), $(call define_target, $(target)))

# Flash the program
flash: $(OUTPUT_DIRECTORY)/mriblue_boot.hex
	@echo Flashing: $<
	$(NRF_TOOL_PATH)/nrfjprog/nrfjprog --program $< -f nrf52 --sectorerase
	$(NRF_TOOL_PATH)/nrfjprog/nrfjprog --reset -f nrf52

# Flash softdevice
flash_softdevice: erase
	@echo Flashing: s132_nrf52_3.0.0_softdevice.hex
	$(NRF_TOOL_PATH)/nrfjprog/nrfjprog --program $(SDK_ROOT)/components/softdevice/s132/hex/s132_nrf52_3.0.0_softdevice.hex -f nrf52 --sectorerase
	$(NRF_TOOL_PATH)/nrfjprog/nrfjprog --reset -f nrf52

erase:
	$(NRF_TOOL_PATH)/nrfjprog/nrfjprog --eraseall -f nrf52


SDK_VER := nRF5_SDK_12.3.0_d7731ad
SDK_ZIP := $(SDK_VER).zip
SDK_URL := https://developer.nordicsemi.com/nRF5_SDK/pieces/nRF5_SDK_v12.x.x/$(SDK_ZIP)
SDK_ZIP_PATH = $(dir $(SDK_ROOT))$(SDK_ZIP)

# sdk target is used to download SDK from the Nordic website.
sdk: $(SDK_ROOT)

# The SDK source code only exists after it has been unzipped from the SDK archive.
$(SDK_ROOT): $(SDK_ZIP_PATH)
	@echo Unzipping $<
	$(NO_ECHO)unzip -d $@ $<
	$(NO_ECHO)mv $(SDK_ROOT)/$(SDK_VER)/* $(SDK_ROOT)
	$(NO_ECHO)rm -d $(SDK_ROOT)/$(SDK_VER)
	$(NO_ECHO)patch -d $(SDK_ROOT) -p1 <$(dir $(SDK_ROOT))sdk.patch

# Download SDK archive from the Nordic website.
$(SDK_ZIP_PATH):
	@echo Downloading $@
	$(NO_ECHO)curl -L0 $(SDK_URL) >$@

mri :
	@echo Initializing mri submodules...
	@cd ..; git submodule update --init mri
	@echo Building mri...
	@$(MAKE) --no-print-directory -C ../mri
