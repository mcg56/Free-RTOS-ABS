LIBS_DIR = $(PROJECT_DIR)/libs

VPATH += $(LIBS_DIR)
INCLUDES += -I$(LIBS_DIR)/include

SRC += ap_pwm.c 
SRC += OrbitOLEDInterface.c

include $(LIBS_DIR)/lib_OrbitOled/Oled.mk