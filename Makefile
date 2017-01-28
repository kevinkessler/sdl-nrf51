#  Copyright (c) 2016 Kevin Kessler - All Rights Reserved
#------------------------------------------------------------------------------
# Firmware build
#
# Selectable build options
#------------------------------------------------------------------------------

USE_RBC_MESH_SERIAL  ?= "no"
USE_BUTTONS          ?= "no"
USE_DFU              ?= "no"

#------------------------------------------------------------------------------
# Define relative paths to SDK components
#------------------------------------------------------------------------------

ifeq ($(OS),Windows_NT)
	SDK_BASE      := D:/NRF5/SDK9
	COMPONENTS    := $(SDK_BASE)/components
	TEMPLATE_PATH := $(COMPONENTS)/toolchain/gcc
	GNU_INSTALL_ROOT=D:/ARM/GNUTOOLS/5.2-2015q4
	MESH_BASE     := D:/NRF5/nRF51-ble-bcast-mesh/nRF51
	OPENOCD_BASE  := D:/NRF5/openocd-0.9.0
	NRFJPROG_BASE := C:/Program Files (x86)/Nordic Semiconductor/nrf5x/bin
else
	SDK_BASE      := ${HOME}/nrf5/SDK9
	COMPONENTS    := $(SDK_BASE)/components
	TEMPLATE_PATH := $(COMPONENTS)/toolchain/gcc
	GNU_INSTALL_ROOT := /usr/local/gcc-arm-none-eabi-5_2-2015q4
	MESH_BASE     := ${HOME}/nrf5/nRF51-ble-bcast-mesh/nRF51
	OPENOCD_BASE  := ${HOME}/nrf5/openocd-0.9.0
	NRFJPROG_BASE := ${HOME}/nrf5/nrfjprog
endif

ifeq ($(USE_DFU), "yes")
	LINKER_SCRIPT := ./gcc_nrf51_s110_xxac.ld
else
	LINKER_SCRIPT := ./gcc_nrf51_s110_xxac_dfu.ld
endif

LINKER_SCRIPT := ./gcc_nrf51_s110_xxac.ld

ifeq ($(USE_RBC_MESH_SERIAL), "yes")
	SERIAL_STRING := "_serial"
else
	SERIAL_STRING := ""
endif

ifeq ($(USE_DFU), "yes")
	DFU_STRING="_dfu"
else
	DFU_STRING=""
endif

OUTPUT_NAME := sdl-all


#------------------------------------------------------------------------------
# Proceed cautiously beyond this point.  Little should change.
#------------------------------------------------------------------------------

export OUTPUT_NAME
export GNU_INSTALL_ROOT

MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) )


# echo suspend
ifeq ("$(VERBOSE)","1")
  NO_ECHO :=
else
  NO_ECHO := @
endif

ifeq ($(MAKECMDGOALS),debug)
  BUILD_TYPE := debug
else
  BUILD_TYPE := release
endif

# Toolchain commands
GNU_PREFIX = arm-none-eabi
CC       := $(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc
AS       := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as"
AR       := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar" -r
LD       := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld"
NM       := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm"
OBJDUMP  := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump"
OBJCOPY  := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy"
SIZE     := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size"
MK       := mkdir
RM       := rm -rf
CP       := cp
GENDAT   := ./gen_dat
GENZIP   := zip
OPENOCD  := "$(OPENOCD_BASE)/bin-x64/openocd"
NRFJPROG := "$(NRFJPROG_BASE)/nrfjprog"

BUILDMETRICS  := ./buildmetrics.py

# function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

# source common to all targets

C_SOURCE_FILES += main.c
C_SOURCE_FILES += sdl_service.c
C_SOURCE_FILES += sdl_config.c
C_SOURCE_FILES += sdl_power_with_ip.c
C_SOURCE_FILES += sdl_switch.c
C_SOURCE_FILES += sdl_power.c
C_SOURCE_FILES += sdl_button.c
C_SOURCE_FILES += RTT/SEGGER_RTT.c
C_SOURCE_FILES += RTT/SEGGER_RTT_printf.c

ifeq ($(USE_RBC_MESH_SERIAL), "yes")
	CFLAGS += -D RBC_MESH_SERIAL=1

	C_SOURCE_FILES += $(MESH_BASE)/rbc_mesh/src/serial_handler_spi.c
	C_SOURCE_FILES += $(MESH_BASE)/rbc_mesh/src/mesh_aci.c
	C_SOURCE_FILES += $(COMPONENTS)/drivers_nrf/spi_slave/spi_slave.c
endif

ifeq ($(USE_DFU), "yes")
	CFLAGS += -D MESH_DFU=1
	C_SOURCE_FILES += $(MESH_BASE)/rbc_mesh/src/dfu_app.c
	C_SOURCE_FILES += $(MESH_BASE)/rbc_mesh/src/mesh_flash.c
	C_SOURCE_FILES += $(MESH_BASE)/rbc_mesh/src/nrf_flash.c
endif


C_SOURCE_FILES += $(MESH_BASE)/rbc_mesh/src/radio_control.c
C_SOURCE_FILES += $(MESH_BASE)/rbc_mesh/src/rbc_mesh.c
C_SOURCE_FILES += $(MESH_BASE)/rbc_mesh/src/timer.c
C_SOURCE_FILES += $(MESH_BASE)/rbc_mesh/src/timer_scheduler.c
C_SOURCE_FILES += $(MESH_BASE)/rbc_mesh/src/timeslot.c
C_SOURCE_FILES += $(MESH_BASE)/rbc_mesh/src/trickle.c
C_SOURCE_FILES += $(MESH_BASE)/rbc_mesh/src/mesh_gatt.c
C_SOURCE_FILES += $(MESH_BASE)/rbc_mesh/src/transport_control.c
C_SOURCE_FILES += $(MESH_BASE)/rbc_mesh/src/fifo.c
C_SOURCE_FILES += $(MESH_BASE)/rbc_mesh/src/event_handler.c
C_SOURCE_FILES += $(MESH_BASE)/rbc_mesh/src/version_handler.c
C_SOURCE_FILES += $(MESH_BASE)/rbc_mesh/src/handle_storage.c
C_SOURCE_FILES += $(MESH_BASE)/rbc_mesh/src/mesh_packet.c
C_SOURCE_FILES += $(MESH_BASE)/rbc_mesh/src/rand.c

C_SOURCE_FILES += $(COMPONENTS)/ble/common/ble_advdata.c
C_SOURCE_FILES += $(COMPONENTS)/ble/ble_services/ble_dfu/ble_dfu.c
C_SOURCE_FILES += $(COMPONENTS)/ble/device_manager/device_manager_peripheral.c
C_SOURCE_FILES += $(COMPONENTS)/toolchain/system_nrf51.c
C_SOURCE_FILES += $(COMPONENTS)/softdevice/common/softdevice_handler/softdevice_handler.c
C_SOURCE_FILES += $(COMPONENTS)/softdevice/common/softdevice_handler/softdevice_handler_appsh.c
C_SOURCE_FILES += $(COMPONENTS)/drivers_nrf/uart/app_uart.c
C_SOURCE_FILES += $(COMPONENTS)/drivers_nrf/gpiote/nrf_drv_gpiote.c
C_SOURCE_FILES += $(COMPONENTS)/drivers_nrf/pstorage/pstorage.c
C_SOURCE_FILES += $(COMPONENTS)/libraries/timer/app_timer.c
C_SOURCE_FILES += $(COMPONENTS)/libraries/timer/app_timer_appsh.c
C_SOURCE_FILES += $(COMPONENTS)/libraries/bootloader_dfu/dfu_app_handler.c
C_SOURCE_FILES += $(COMPONENTS)/libraries/bootloader_dfu/bootloader_util.c
C_SOURCE_FILES += $(COMPONENTS)/libraries/scheduler/app_scheduler.c
C_SOURCE_FILES += $(COMPONENTS)/drivers_nrf/common/nrf_drv_common.c
C_SOURCE_FILES += $(COMPONENTS)/drivers_nrf/hal/nrf_delay.c

# assembly files common to all targets
ASM_SOURCE_FILES  += $(COMPONENTS)/toolchain/gcc/gcc_startup_nrf51.s

# includes common to all targets

INC_PATHS += -Iinclude
INC_PATHS += -I$(MESH_BASE)/rbc_mesh
INC_PATHS += -I$(MESH_BASE)/rbc_mesh/include
INC_PATHS += -I$(MESH_BASE)/SDK/bsp
INC_PATHS += -IRTT

INC_PATHS += -I$(COMPONENTS)/softdevice/s110/headers
INC_PATHS += -I$(COMPONENTS)/softdevice/common/softdevice_handler
INC_PATHS += -I$(COMPONENTS)/toolchain/gcc
INC_PATHS += -I$(COMPONENTS)/libraries/util
INC_PATHS += -I$(COMPONENTS)/ble/common
INC_PATHS += -I$(COMPONENTS)/ble/ble_services/ble_dfu
INC_PATHS += -I$(COMPONENTS)/ble/device_manager
INC_PATHS += -I$(COMPONENTS)/drivers_nrf/hal
INC_PATHS += -I$(COMPONENTS)/drivers_nrf/spi_slave
INC_PATHS += -I$(COMPONENTS)/drivers_nrf/uart
INC_PATHS += -I$(COMPONENTS)/drivers_nrf/gpiote
INC_PATHS += -I$(COMPONENTS)/drivers_nrf/common
INC_PATHS += -I$(COMPONENTS)/drivers_nrf/pstorage
INC_PATHS += -I$(COMPONENTS)/drivers_nrf/ble_flash
INC_PATHS += -I$(COMPONENTS)/drivers_nrf/hal
INC_PATHS += -I$(COMPONENTS)/libraries/timer
INC_PATHS += -I$(COMPONENTS)/libraries/scheduler
INC_PATHS += -I$(COMPONENTS)/libraries/bootloader_dfu
INC_PATHS += -I$(COMPONENTS)/libraries/trace
INC_PATHS += -I$(COMPONENTS)/toolchain
INC_PATHS += -I$(COMPONENTS)/device

OBJECT_DIRECTORY = build
LISTING_DIRECTORY = $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY = $(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

ifeq ($(BUILD_TYPE),debug)
  DEBUG_FLAGS += -D DEBUG -g -O0 -fomit-frame-pointer
else
  DEBUG_FLAGS += -D NDEBUG -O3
endif

# flags common to all targets
#CFLAGS += -save-temps
CFLAGS += $(DEBUG_FLAGS)
CFLAGS += -D NRF51
CFLAGS += -D BLE_STACK_SUPPORT_REQD
CFLAGS += -D S110
CFLAGS += -D SOFTDEVICE_PRESENT
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
CFLAGS += -Wall -Werror
CFLAGS += -Wa,-adhln
CFLAGS += -mfloat-abi=soft
CFLAGS += -ffunction-sections
CFLAGS += -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin

LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_NAME).map
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0
LDFLAGS += $(DEBUG_FLAGS)
LDFLAGS += -Wl,--gc-sections
LDFLAGS += --specs=nano.specs -lc -lnosys

# Assembler flags
ASMFLAGS += $(DEBUG_FLAGS)
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -D NRF51
ASMFLAGS += -D BLE_STACK_SUPPORT_REQD
ASMFLAGS += -D S110
ASMFLAGS += -D SOFTDEVICE_PRESENT


C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )

TOOLCHAIN_BASE = $(basename $(notdir $(GNU_INSTALL_ROOT)))

TIMESTAMP := $(shell date +'%s')

vpath %.c $(C_PATHS)
vpath %.s $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

all: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_NAME).elf
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).elf
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

	@echo "*****************************************************"
	@echo "build project: $(OUTPUT_NAME)"
	@echo "build type:    $(BUILD_TYPE)"
	@echo "build with:    $(TOOLCHAIN_BASE)"
	@echo "build options  --"
	@echo "               USE_RBC_MESH_SERIAL $(USE_RBC_MESH_SERIAL)"
	@echo "               USE_DFU             $(USE_DFU)"
	@echo "build products --"
	@echo "               $(OUTPUT_NAME).elf"
	@echo "               $(OUTPUT_NAME).hex"
	@echo "*****************************************************"

debug : all

release : all

# Create build directories
$(BUILD_DIRECTORIES):
	echo $(MAKEFILE_NAME)
	$(MK) $@

# Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) \
	-c $< -o $@ > $(OUTPUT_BINARY_DIRECTORY)/$*.lst

# Assemble files
$(OBJECT_DIRECTORY)/%.o: %.s
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<

# Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).elf: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_NAME).elf
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).elf

# Create binary .bin file from the .elf file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).elf
	@echo Preparing: $(OUTPUT_NAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).elf $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).bin

# Create binary .hex file from the .elf file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).elf
	@echo Preparing: $(OUTPUT_NAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).elf $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).hex

finalize: genbin genhex echosize

genbin:
	@echo Preparing: $(OUTPUT_NAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).elf $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).bin

# Create binary .hex file from the .elf file
genhex:
	@echo Preparing: $(OUTPUT_NAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).elf $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).hex

echosize:
	-@echo ""
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).elf
	-@echo ""

clean:
	$(RM) $(BUILD_DIRECTORIES)

cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o
	
flash_softdevice:
#	$(NO_ECHO)$(OPENOCD) -s $(OPENOCD_BASE)/scripts -f interface/stlink-v2.cfg -f target/nrf51.cfg -c "program /NRF5/SDK9/components/softdevice/s110/hex/s110_softdevice.hex verify reset exit"
	$(NO_ECHO) $(NRFJPROG) --reset --program D:/NRF5/SDK9/components/softdevice/s110/hex/s110_softdevice.hex

flash:
#	$(NO_ECHO)$(OPENOCD) -s $(OPENOCD_BASE)/scripts -f interface/stlink-v2.cfg -f target/nrf51.cfg -c "program $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).hex verify reset exit"
	$(NO_ECHO) $(NRFJPROG) --erasepage 0x00000-0x3f800
#	$(NO_ECHO) $(NRFJPROG) --eraseall
	$(NO_ECHO) $(NRFJPROG) --program $(COMPONENTS)/softdevice/s110/hex/s110_softdevice.hex
	$(NO_ECHO) $(NRFJPROG) --program $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).hex
	$(NO_ECHO) $(NRFJPROG) --reset