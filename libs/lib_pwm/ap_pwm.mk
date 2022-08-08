LIBS_PWM_DIR = $(LIBS_DIR)/lib_pwm

VPATH += $(LIBS_PWM_DIR)
INCLUDES += -I"$(LIBS_PWM_DIR)"

LDLIBS = -lm

SRC += ap_pwm_input.c ap_pwm_output.c
