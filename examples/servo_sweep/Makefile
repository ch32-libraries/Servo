all : flash

TARGET:=servo_sweep
TARGET_EXT:=cpp

TARGET_MCU?=CH32V003
ADDITIONAL_C_FILES+=../../src/Servo.cpp

include ../../ch32fun/ch32fun/ch32fun.mk

# Must come after the include: ch32fun.mk sets CFLAGS via `?=`, so setting
# CFLAGS before the include (even via +=) makes that `?=` a no-op and
# silently drops -Os/-flto/-ffunction-sections/-fdata-sections.
CFLAGS+=-I../../src

flash : cv_flash
clean : cv_clean
