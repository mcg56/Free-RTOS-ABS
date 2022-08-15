LIBS_PWM_DIR = $(LIBS_DIR)/lib_pwm

VPATH += $(LIBS_PWM_DIR)
INCLUDES += -I"$(LIBS_PWM_DIR)"

LDLIBS = -lm

SRC += pwm_input.c pwm_output.c
