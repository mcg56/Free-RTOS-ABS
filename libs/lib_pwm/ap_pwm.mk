LIBS_PWM_DIR = $(LIBS_DIR)/lib_pwm

VPATH += $(LIBS_PWM_DIR)
INCLUDES += -I"$(LIBS_PWM_DIR)"

SRC += ap_pwm.c
