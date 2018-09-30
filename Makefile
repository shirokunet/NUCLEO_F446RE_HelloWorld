PROJECT         := HelloWorld
DEVICES         := NUCLEO_F446RE

GCC4MBED_DIR    := ../..
NO_FLOAT_SCANF  := 1
NO_FLOAT_PRINTF := 0

MBED_OS_ENABLE	:= 0

include $(GCC4MBED_DIR)/build/gcc4mbed_NUCLEO_F446RE.mk

