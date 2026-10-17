# SPDX-FileCopyrightText: Copyright (c) 2026 Mikey Sklar for Adafruit Industries
# SPDX-License-Identifier: MIT
# ogg_native, the C build of the kernel, as a native module:
#   make MPY_DIR=path/to/circuitpython ARCH=armv7emsp
#   make MPY_DIR=path/to/circuitpython all-arches
# Writes ../../adafruit_ogg/ogg_native.<arch>.mpy. Needs arm-none-eabi-gcc. Use the
# CircuitPython tree whose firmware will load it (the .mpy version must match).
MPY_DIR ?= ../../../circuitpython
MOD = ogg_native
SRC = ogg_native.c
ARCH ?= armv7emsp
# libgcc for 64-bit multiplies and division on armv6m
LINK_RUNTIME = 1
CFLAGS_EXTRA = -O2
# mpy_ld's symbol order depends on Python's hash seed; pin it so builds are repeatable
export PYTHONHASHSEED = 0

include $(MPY_DIR)/py/dynruntime.mk

ARCHES = armv6m armv7emsp

all-arches:
	@set -e; for a in $(ARCHES); do \
		rm -rf build; \
		$(MAKE) --no-print-directory MPY_DIR=$(MPY_DIR) ARCH=$$a; \
		mv $(MOD).mpy ../../adafruit_ogg/$(MOD).$$a.mpy; \
	done; rm -rf build

.PHONY: all-arches
