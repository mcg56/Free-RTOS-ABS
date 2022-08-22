LIBS_DIR = $(PROJECT_DIR)/Project1/libs

VPATH += $(LIBS_DIR)

include $(LIBS_DIR)/lib_OrbitOled/Oled.mk
include $(LIBS_DIR)/lib_pwm/pwm.mk
include $(LIBS_DIR)/lib_buttons/buttons.mk
include $(LIBS_DIR)/lib_uart/uart.mk


