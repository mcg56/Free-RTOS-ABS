LIBS_ADC_DIR = $(LIBS_DIR)/lib_adc

VPATH += $(LIBS_ADC_DIR)
INCLUDES += -I"$(LIBS_ADC_DIR)"

SRC += ap_adc.c 
SRC += ap_circBufT.c 