#
# Component Makefile
#

COMPONENT_ADD_INCLUDEDIRS := include

COMPONENT_SRCDIRS := src

ifndef IS_BOOTLOADER_BUILD
CFLAGS += -DUSING_IBUS_FASTER_GET
endif