
SRC += libs/ap_pwm.c 
SRC += libs/OrbitOLEDInterface.c
INCLUDES += -I"$(PROJECT_DIR)/libs/include"



SRC += libs/lib_OrbitOled/ChrFont0.c
SRC += libs/lib_OrbitOled/delay.c
SRC += libs/lib_OrbitOled/FillPat.c
SRC += libs/lib_OrbitOled/OrbitOled.c
SRC += libs/lib_OrbitOled/OrbitOledChar.c
SRC += libs/lib_OrbitOled/OrbitOledGrph.c


#INCLUDES += -I"$(PROJECT_DIR)/libs/OrbitOLED"
INCLUDES += -I"$(PROJECT_DIR)/libs/lib_OrbitOled"
