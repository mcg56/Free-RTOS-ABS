LIBS_SYSTEM_DIR = $(LIBS_DIR)/lib_system

VPATH += $(LIBS_SYSTEM_DIR)
INCLUDES += -I"$(LIBS_SYSTEM_DIR)"

SRC += ap_system.c

