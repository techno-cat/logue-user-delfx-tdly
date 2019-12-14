# #############################################################################
# Project Customization
# #############################################################################

PROJECT = tempo_delay

UCSRC = $(wildcard ../user/lib/*.c)

UCXXSRC = ../user/delay.cpp

# NOTE: Relative path from `Makefile` that refer this file.
UINCDIR = ../user/lib

UDEFS =

ULIB = 

ULIBDIR =
