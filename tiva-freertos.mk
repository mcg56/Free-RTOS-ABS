BUILD_DIR ?= build

ifndef TARGET
TARGET := $(BUILD_DIR)/$(notdir $(shell pwd)).elf
endif

ifndef PROJECT_DIR
$(error PROJECT_DIR must be defined)
endif

TOOLCHAIN ?= arm-none-eabi

CC = $(TOOLCHAIN)-gcc
LD = $(TOOLCHAIN)-gcc
AR = $(TOOLCHAIN)-ar

ifneq (,$(shell which $(TOOLCHAIN)-gdb))
GDB ?= $(TOOLCHAIN)-gdb
else
# This supersedes arm-none-eabi-gdb on Debian (Ubuntu?) installs
GDB ?= gdb-multiarch
endif

OPENOCD ?= openocd

FREERTOS_ARCH ?= GCC/ARM_CM4F
FREERTOS_HEAP_IMPL ?= heap_1

SCRIPTS_DIR = $(PROJECT_DIR)/scripts

TIVAWARE_DIR = $(PROJECT_DIR)/SW-TM4C-2.2.0.295
TIVAWARE_INCLUDES = -I"$(TIVAWARE_DIR)"
TIVAWARE_LIBS = $(TIVAWARE_DIR)/driverlib/gcc/libdriver.a
TIVAWARE_SRC = $(TIVAWARE_DIR)/utils/ustdlib.c

FREERTOS_DIR = $(PROJECT_DIR)/FreeRTOS-Kernel-10.4.6
FREERTOS_ARCH_DIR = $(FREERTOS_DIR)/portable/$(FREERTOS_ARCH)
FREERTOS_INCLUDES = \
	-I"$(FREERTOS_DIR)/include" \
	-I"$(FREERTOS_ARCH_DIR)"
FREERTOS_SRC = \
	$(FREERTOS_DIR)/croutine.c \
	$(FREERTOS_DIR)/event_groups.c \
	$(FREERTOS_DIR)/list.c \
	$(FREERTOS_DIR)/queue.c \
	$(FREERTOS_DIR)/stream_buffer.c \
	$(FREERTOS_DIR)/tasks.c \
	$(FREERTOS_DIR)/timers.c \
	$(FREERTOS_DIR)/portable/MemMang/$(FREERTOS_HEAP_IMPL).c \
	$(FREERTOS_ARCH_DIR)/port.c

OPT ?= -O0

CFLAGS += \
	-mcpu=cortex-m4 \
	-mthumb \
	-mfloat-abi=hard \
	-mfpu=fpv4-sp-d16 \
	-DPART_TM4C123GH6PM \
	-g3 -Wall -W $(OPT) \
	$(CFLAGS_EXTRA)

SRC += startup.c $(notdir $(TIVAWARE_SRC) $(FREERTOS_SRC))
VPATH += $(PROJECT_DIR) $(dir $(TIVAWARE_SRC) $(FREERTOS_SRC))
LIBS += $(TIVAWARE_LIBS)
INCLUDES += -I"$(PROJECT_DIR)" $(FREERTOS_INCLUDES) $(TIVAWARE_INCLUDES)

ifndef VERBOSE
Q=@
endif

all: $(TARGET)

info:
	@echo TARGET=$(TARGET)
	@echo TOOLCHAIN=$(TOOLCHAIN)
	@echo CC=$(CC)
	@echo LD=$(LD)
	@echo GDB=$(GDB)
	@echo SRC=$(SRC)
	@echo VPATH=$(VPATH)

$(BUILD_DIR)/%.o: %.c
	@mkdir -p "$(@D)"
	$(info CC $^)
	$(Q)$(CC) $(CFLAGS) $(INCLUDES) -o $@ -c $<

$(TARGET): $(addprefix $(BUILD_DIR)/, $(patsubst %.c,%.o,$(SRC)))
	$(info LD $@)
	$(Q)$(LD) $(CFLAGS) -T$(SCRIPTS_DIR)/link.ld -o $@ $^ $(LIBS)

clean:
	rm -rf $(BUILD_DIR) $(TARGET)

# Use GDB to load the code onto the device
program: $(TARGET)
	$(GDB) -batch -x $(SCRIPTS_DIR)/program.gdb $^

# Use GDB to debug the device
debug: $(TARGET)
	$(GDB) -x $(SCRIPTS_DIR)/debug.gdb $^

openocd:
	$(OPENOCD) -f board/ek-tm4c123gxl.cfg

.PHONY: all clean debug info openocd program
