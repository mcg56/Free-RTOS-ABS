LIBS_UART_DIR = $(LIBS_DIR)/lib_uart

VPATH += $(LIBS_UART_DIR)
INCLUDES += -I"$(LIBS_UART_DIR)"

SRC += ap_uart.c

