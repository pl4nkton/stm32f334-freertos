# Optimization level, can be [0, 1, 2, 3, s]. 
#     0 = turn off optimization. s = optimize for size.
# 
OPT = -Os -flto
# OPT = -O1         # for debugging

BOARD ?= BOARD_NUCLEO

FLASH_ADDR = 0x08000000

# Object files directory
# Warning: this will be removed by make clean!
#
OBJDIR = obj_app

# Target file name (without extension)
TARGET = $(OBJDIR)/mppt

# Define all C source files (dependencies are generated automatically)
#
INCDIRS += .
INCDIRS += Source

SOURCES += Source/main.c

SOURCES += Source/board.c
SOURCES += Source/syscalls.c
SOURCES += Source/ustime.c
SOURCES += Source/term_rs232.c

SOURCES += Source/command.c
SOURCES += Source/fault_handler.c
SOURCES += Source/led_task.c
SOURCES += Source/param_table.c
SOURCES += Source/parameter.c
SOURCES += Source/readline.c
SOURCES += Source/shell_task.c
SOURCES += Source/util.c
#SOURCES += Source/filter.c
SOURCES += Source/watchdog.c
#SOURCES += Source/version.c

SOURCES += Shared/cobsr.c
SOURCES += Shared/errors.c
#SOURCES += Shared/crc16.c
#SOURCES += Shared/crc32.c

# FreeRTOS
#
FREERTOS_BASE = Libraries/FreeRTOSV8.2.1
FREERTOS_DIR = $(FREERTOS_BASE)/FreeRTOS/Source

INCDIRS += $(FREERTOS_DIR)/include
INCDIRS += $(FREERTOS_DIR)/portable/GCC/ARM_CM4F

SOURCES += $(FREERTOS_DIR)/tasks.c
SOURCES += $(FREERTOS_DIR)/queue.c
SOURCES += $(FREERTOS_DIR)/list.c
SOURCES += $(FREERTOS_DIR)/croutine.c
SOURCES += $(FREERTOS_DIR)/portable/GCC/ARM_CM4F/port.c

# Tracealyzer
#
TRACE_DIR = $(FREERTOS_BASE)/FreeRTOS-Plus/Source/FreeRTOS-Plus-Trace
INCDIRS += $(TRACE_DIR)/Include

SOURCES += $(TRACE_DIR)/trcBase.c
SOURCES += $(TRACE_DIR)/trcKernel.c
SOURCES += $(TRACE_DIR)/trcUser.c
SOURCES += $(TRACE_DIR)/trcHardwarePort.c
SOURCES += $(TRACE_DIR)/trcKernelPort.c


# Standard peripheral library
#
CPPFLAGS += -DUSE_STDPERIPH_DRIVER
CPPFLAGS += -DUSE_FULL_ASSERT

STDPERIPH_DIR = Libraries/STM32F30x_StdPeriph_Driver-V1.2.1

INCDIRS += $(STDPERIPH_DIR)/inc

SOURCES += $(STDPERIPH_DIR)/src/stm32f30x_misc.c
SOURCES += $(STDPERIPH_DIR)/src/stm32f30x_adc.c
SOURCES += $(STDPERIPH_DIR)/src/stm32f30x_dac.c
SOURCES += $(STDPERIPH_DIR)/src/stm32f30x_dma.c
SOURCES += $(STDPERIPH_DIR)/src/stm32f30x_exti.c
SOURCES += $(STDPERIPH_DIR)/src/stm32f30x_flash.c
SOURCES += $(STDPERIPH_DIR)/src/stm32f30x_gpio.c
SOURCES += $(STDPERIPH_DIR)/src/stm32f30x_i2c.c
SOURCES += $(STDPERIPH_DIR)/src/stm32f30x_iwdg.c
SOURCES += $(STDPERIPH_DIR)/src/stm32f30x_rcc.c
SOURCES += $(STDPERIPH_DIR)/src/stm32f30x_spi.c
SOURCES += $(STDPERIPH_DIR)/src/stm32f30x_syscfg.c
SOURCES += $(STDPERIPH_DIR)/src/stm32f30x_tim.c
SOURCES += $(STDPERIPH_DIR)/src/stm32f30x_usart.c


# CMSIS-Library
#
CMSIS_DIR = Libraries/CMSIS-3.2.0

INCDIRS += $(CMSIS_DIR)/Include
INCDIRS += $(CMSIS_DIR)/Device/ST/STM32F30x/Include

SOURCES += $(CMSIS_DIR)/Device/ST/STM32F30x/Source/startup_stm32f334x8.s

# Board support
#
ifeq ($(BOARD), BOARD_NUCLEO)
    CPPFLAGS += -DBOARD_NUCLEO
    CPPFLAGS += -DHSE_VALUE=8000000
    SOURCES  += $(CMSIS_DIR)/Device/ST/STM32F30x/Source/system_stm32f30x.c
else
ifeq ($(BOARD), REV_A)
    CPPFLAGS += -DBOARD_REV_A
    CPPFLAGS += -DHSE_VALUE=8000000
    SOURCES  += $(CMSIS_DIR)/Device/ST/STM32F30x/Source/system_stm32f30x.c
else
    $(error Unknown BOARD type: must be BOARD_NUCLEO or REV_A)
endif
endif

CPPFLAGS += -DSTM32F334x8
LDSCRIPT = Source/stm32f334_app.ld


#============================================================================
#
OBJECTS  += $(addprefix $(OBJDIR)/,$(addsuffix .o,$(basename $(SOURCES))))
CPPFLAGS += $(addprefix -I,$(INCDIRS))

#---------------- Preprocessor Options ----------------
#  -fsingle...    make better use of the single-precision FPU
#  -g             generate debugging information
#  -save-temps    preserve .s and .i-files
#
CPPFLAGS += -fsingle-precision-constant
CPPFLAGS += -g
# CPPFLAGS += -save-temps=obj

#---------------- C Compiler Options ----------------
#  -O*            optimization level
#  -f...          tuning, see GCC documentation
#  -Wall...       warning level
#
CFLAGS += $(OPT)
CFLAGS += -std=gnu11
CFLAGS += -ffunction-sections
CFLAGS += -fdata-sections
CFLAGS += -Wall
CFLAGS += -Wstrict-prototypes
#CFLAGS += -Wextra
#CFLAGS += -Wpointer-arith
#CFLAGS += -Winline
#CFLAGS += -Wunreachable-code
#CFLAGS += -Wundef
#CFLAGS += -fexceptions -fno-use-linker-plugin

# Use a friendly C dialect
CPPFLAGS += -fno-strict-aliasing
CPPFLAGS += -fwrapv

#---------------- C++ Compiler Options ----------------
#
CXXFLAGS += $(OPT)
CXXFLAGS += -ffunction-sections
CXXFLAGS += -fdata-sections
CXXFLAGS += -Wall

#---------------- Assembler Options ----------------
#  -Wa,...    tell GCC to pass this to the assembler
#

#---------------- Linker Options ----------------
#  -Wl,...      tell GCC to pass this to linker
#    -Map       create map file
#    --cref     add cross reference to  map file
#
LDFLAGS += $(OPT)
LDFLAGS += -lm -lc_nano -lg_nano -specs=nano.specs
LDFLAGS += -Wl,-Map=$(TARGET).map,--cref
LDFLAGS += -Wl,--gc-sections,--relax

# LDFLAGS += -specs=nano.specs -u _printf_float -u _scanf_float
LDFLAGS += -T$(LDSCRIPT)

#============================================================================

# Define programs and commands
#
TOOLCHAIN = arm-none-eabi-
CC       = $(TOOLCHAIN)gcc
OBJCOPY  = $(TOOLCHAIN)objcopy
OBJDUMP  = $(TOOLCHAIN)objdump
SIZE     = $(TOOLCHAIN)size
NM       = $(TOOLCHAIN)nm
MKDIR    = mkdir
DOXYGEN  = doxygen
STLINK_WIN = Tools/st-link/ST-LINK_CLI.exe
STLINK_LNX = st-flash
POSTLD   = Tools/add_version_info.py # -q

# Compiler flags to generate dependency files
#
GENDEPFLAGS = -MMD -MP

# Combine all necessary flags and optional flags
# Add target processor to flags.
#
CPU = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16

CFLAGS   += $(CPU)
CXXFLAGS += $(CPU)
ASFLAGS  += $(CPU)
LDFLAGS  += $(CPU)

ifeq ($(OS),Windows_NT)
	STLINK = $(STLINK_WIN) -c SWD -P $(TARGET).hex -Run
else
	UNAME_S := $(shell uname -s)
	ifeq ($(UNAME_S),Linux)
		STLINK = $(STLINK_LNX) --reset write $(TARGET).bin $(FLASH_ADDR)
	endif
	ifeq ($(UNAME_S),Darwin)
		$(error Don't know how to flash under OSX)
	endif
endif

# Default target
#
all:  gccversion build showsize

build: elf hex bin lss sym

elf: $(TARGET).elf
hex: $(TARGET).hex
bin: $(TARGET).bin
lss: $(TARGET).lss
sym: $(TARGET).sym


doxygen:
	@echo
	@echo Creating Doxygen documentation
	@$(DOXYGEN)


boot:
	$(MAKE) -f Bootloader/Makefile

boot_clean:
	$(MAKE) -f Bootloader/Makefile clean

boot_flash:
	$(MAKE) -f Bootloader/Makefile flash


# Display compiler version information
#
gccversion: 
	@$(CC) --version


# Show the final program size
#
showsize: build
	@echo
	@$(SIZE) $(TARGET).elf 2>/dev/null


# Flash the device  
#
flash: build showsize
	$(STLINK)

# Target: clean project
#
clean:
	@echo Cleaning project:
	rm -rf $(OBJDIR)

# Include the base rules
#
include base.mak

# Include the dependency files
#
-include $(OBJECTS:.o=.d)

# Listing of phony targets
#
.PHONY: all build flash clean \
        boot boot_clean boot_flash \
        doxygen elf lss sym \
        showsize gccversion
