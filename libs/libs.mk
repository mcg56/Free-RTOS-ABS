LIBS_DIR = $(PROJECT_DIR)/libs

VPATH += $(LIBS_DIR)
INCLUDES += -I$(LIBS_DIR)/include

include $(LIBS_DIR)/lib_OrbitOled/ap_Oled.mk
include $(LIBS_DIR)/lib_pwm/ap_pwm.mk
include $(LIBS_DIR)/lib_buttons/ap_buttons.mk
include $(LIBS_DIR)/lib_adc/ap_adc.mk
include $(LIBS_DIR)/lib_system/ap_system.mk

