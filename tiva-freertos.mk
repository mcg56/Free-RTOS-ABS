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
FREERTOS_HEAP_IMPL ?= heap_4

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
	-specs=nano.specs \
	-specs=nosys.specs \
	-g3 -Wall -W $(OPT) \
	$(INCLUDES) \
	$(CFLAGS_EXTRA)

LDFLAGS += \
	-T$(SCRIPTS_DIR)/link.ld \
	-mcpu=cortex-m4 \
	-mthumb \
	-mfloat-abi=hard \
	-mfpu=fpv4-sp-d16 \
	-specs=nano.specs \
	-specs=nosys.specs

SRC += startup.c $(notdir $(TIVAWARE_SRC) $(FREERTOS_SRC))
OBJS = $(addprefix $(BUILD_DIR)/, $(patsubst %.c,%.o,$(SRC)))
VPATH += $(PROJECT_DIR) $(sort $(dir $(TIVAWARE_SRC) $(FREERTOS_SRC)))
LDLIBS += $(TIVAWARE_LIBS)
INCLUDES += -I"$(PROJECT_DIR)" $(FREERTOS_INCLUDES) $(TIVAWARE_INCLUDES)
DEP_DIR = $(BUILD_DIR)/deps
DEPS = $(sort $(addprefix $(DEP_DIR)/, $(notdir $(sort $(SRC:.c=.d)))))

ifndef VERBOSE
Q=@
endif

all: $(TARGET)

info:
	@echo TARGET=$(TARGET)
	@echo TOOLCHAIN=$(TOOLCHAIN)
	@echo CC=$(CC)
	@echo CFLAGS=$(CFLAGS)
	@echo LD=$(LD)
	@echo LDFLAGS=$(LDFLAGS)
	@echo LDLIBS=$(LDLIBS)
	@echo GDB=$(GDB)
	@echo SRC=$(SRC)
	@echo VPATH=$(VPATH)
	@echo DEPS=$(DEPS)

$(DEP_DIR):
	mkdir -p $(DEP_DIR)

$(DEPS): | $(DEPDIR)

-include $(DEPS)

$(BUILD_DIR)/%.o: %.c
	@mkdir -p "$(@D)"
	$(info CC $<)
	$(Q)$(CC) $(CFLAGS) -o $@ -c $<
	$(Q)printf "$(BUILD_DIR)/" > $(DEP_DIR)/$*.d
	$(Q)$(CC) -MM $(CFLAGS) $< >> $(DEP_DIR)/$*.d

$(TARGET): $(DEP_DIR) $(OBJS)
	$(info LD $@)
	$(Q)$(LD) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS)

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
