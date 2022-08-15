LIBS_DIR = $(PROJECT_DIR)/libs

VPATH += $(LIBS_DIR)

include $(LIBS_DIR)/lib_OrbitOled/Oled.mk
include $(LIBS_DIR)/lib_pwm/pwm.mk
include $(LIBS_DIR)/lib_buttons/buttons.mk
include $(LIBS_DIR)/lib_adc/adc.mk
include $(LIBS_DIR)/lib_system/system.mk
include $(LIBS_DIR)/lib_uart/uart.mk

