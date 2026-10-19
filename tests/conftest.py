# SPDX-FileCopyrightText: Copyright (c) 2026 Mikey Sklar for Adafruit Industries
# SPDX-License-Identifier: MIT
"""pytest setup: CI builds the bundles from this checkout after the tests, so don't leave a
__pycache__ in the package for them to pick up."""

import sys

sys.dont_write_bytecode = True
