# SPDX-FileCopyrightText: Copyright (c) 2026 Mikey Sklar for Adafruit Industries
# SPDX-License-Identifier: MIT
# ogg_kernel, one turbo .mpy per architecture, built by mpy-cross -march:
#   make MPY_CROSS=path/to/circuitpython/mpy-cross/build/mpy-cross
# Writes ../adafruit_ogg/ogg_kernel.<arch>.mpy for each arch in ARCHES. Use an mpy-cross
# from the CircuitPython tree whose firmware will load it (the .mpy version must match).
# Boards find the file by the name in microcontroller.cpu.architecture (#11453).
MPY_CROSS ?= mpy-cross
SRC ?= ogg_turbo.py
OUT = ../adafruit_ogg
ARCHES = armv6m armv7emsp xtensawin rv32imc

all: $(foreach a,$(ARCHES),$(OUT)/ogg_kernel.$(a).mpy)

$(OUT)/ogg_kernel.%.mpy: $(SRC)
	$(MPY_CROSS) -march=$* -o $@ $<

.PHONY: all
