LIBS_ORBITOLED_DIR = $(LIBS_DIR)/lib_OrbitOled

VPATH += $(LIBS_ORBITOLED_DIR)
INCLUDES += -I"$(LIBS_ORBITOLED_DIR)"

SRC += ChrFont0.c
SRC += delay.c
SRC += FillPat.c
SRC += OrbitOled.c
SRC += OrbitOledChar.c
SRC += OrbitOledGrph.c
