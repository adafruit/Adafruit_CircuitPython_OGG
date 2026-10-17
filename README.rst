Introduction
============


.. image:: https://readthedocs.org/projects/adafruit-circuitpython-ogg/badge/?version=latest
    :target: https://docs.circuitpython.org/projects/ogg/en/latest/
    :alt: Documentation Status


.. image:: https://raw.githubusercontent.com/adafruit/Adafruit_CircuitPython_Bundle/main/badges/adafruit_discord.svg
    :target: https://adafru.it/discord
    :alt: Discord


.. image:: https://github.com/adafruit/Adafruit_CircuitPython_OGG/workflows/Build%20CI/badge.svg
    :target: https://github.com/adafruit/Adafruit_CircuitPython_OGG/actions
    :alt: Build Status


.. image:: https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json
    :target: https://github.com/astral-sh/ruff
    :alt: Code Style: Ruff

Play Ogg Vorbis files on CircuitPython. ``OggDecoder`` decodes Ogg Vorbis audio to 16-bit PCM,
and ``OggPlayer`` streams it to an ``I2SOut``, ``PWMAudioOut`` or ``AudioOut`` while your code
keeps running. On an RP2350 it plays 44.1 kHz stereo files at the stock 150 MHz with about a
quarter of the CPU.

The decoder is written from the Xiph.Org Vorbis I specification. No third-party decoder code is
included. The audio decode runs in a prebuilt kernel for each chip architecture, in fixed
point, and stays within a few LSB of ffmpeg's floating point decode.


Dependencies
=============
This library depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_ 11 or later, on a board
  that can load native ``.mpy`` files (``microcontroller.cpu.architecture`` is not ``None``)

The decoding loops are a precompiled kernel, one file per architecture. Arm chips use a C build
of the kernel, ``ogg_native``; the others use a turbo (``@micropython.viper``) build,
``ogg_kernel``. Both have the same functions and give the same output to within an LSB.

=================  =========================  ======================================
Architecture       Kernel                     Boards
=================  =========================  ======================================
``armv6m``         ``ogg_native`` (C)         RP2040
``armv7emsp``      ``ogg_native`` (C)         SAMD51 (M4), nRF52840, RP2350, STM32F4
``xtensawin``      ``ogg_kernel`` (turbo)     ESP32-S2, ESP32-S3
``rv32imc``        ``ogg_kernel`` (turbo)     ESP32-C3, C5, C6
=================  =========================  ======================================

``ogg_kernel`` is also built for the two Arm architectures, as a fallback. Without a kernel the
decoder falls back to plain Python, which is correct but far slower than real time for music.
Only the RP2350 has been tested for playback.

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://circuitpython.org/libraries>`_
or individual libraries can be installed using
`circup <https://github.com/adafruit/circup>`_.

Installing to a Connected CircuitPython Device with Circup
==========================================================

Make sure that you have ``circup`` installed in your Python environment.
Install it with the following command if necessary:

.. code-block:: shell

    pip3 install circup

With ``circup`` installed and your CircuitPython device connected use the
following command to install:

.. code-block:: shell

    circup install adafruit_ogg

Or the following command to update an existing version:

.. code-block:: shell

    circup update

Usage Example
=============

.. code-block:: python

    import adafruit_ogg
    from adafruit_fruitjam.peripherals import Peripherals

    decoder = adafruit_ogg.OggDecoder("song.ogg")
    fruit_jam = Peripherals(audio_output="headphone", sample_rate=decoder.sample_rate)

    player = adafruit_ogg.OggPlayer(fruit_jam.audio)
    player.play(decoder)
    while player.playing:
        player.update()  # decode the next piece; call this often while playing

``decoder.comments`` holds the file's tags (``TITLE``, ``ARTIST`` and so on), and
``decoder.open(path)`` switches to another file, reusing the buffers.

Speed and memory
================

Measured on an Adafruit Fruit Jam (RP2350B) at 150 MHz. Load is decode time divided by audio
time; under 100% keeps up.

=========================  ======  =================
Format                     Load    Opening a file
=========================  ======  =================
44.1 kHz stereo            24%     0.5 s
22.05 kHz mono             8%      0.5 s
11.025 kHz mono            5%      0.3 s
=========================  ======  =================

- The decoder needs about 100 KB (11 kHz mono) to 175 KB (48 kHz stereo), most of it codebook
  tables, which depend on the file.
- Keep the decoder in internal SRAM. On the RP2350, with its buffers in PSRAM it decodes about
  four times slower (93% load at 44.1 kHz stereo). Create it first in ``code.py``, before other
  large allocations such as ``Peripherals``, and reuse it with ``decoder.open()``.
- ``OggPlayer`` decodes straight into the half of its buffer that the audio DMA has already
  copied, so it does not work with ``audiomixer``, which reads the buffer in place.
- Opening a file parses its codebooks, which takes a fraction of a second. Let the player
  finish or stop it before opening the next file.

Building the kernels
====================

``src/ogg_turbo.py`` is the turbo kernel (``@micropython.viper`` functions) and
``src/native/ogg_native.c`` the same functions in C. ``src/Makefile`` compiles the turbo kernel
with ``mpy-cross -march`` for each architecture into ``adafruit_ogg/ogg_kernel.<arch>.mpy``;
``src/native/Makefile`` builds the C kernel with the CircuitPython native module tools
(``py/dynruntime.mk``, needs ``arm-none-eabi-gcc`` and ``pyelftools``) into
``adafruit_ogg/ogg_native.<arch>.mpy``:

.. code-block:: shell

    make -C src MPY_CROSS=path/to/circuitpython/mpy-cross/build/mpy-cross
    make -C src/native MPY_DIR=path/to/circuitpython all-arches

Use the CircuitPython version the files will run on.

Testing
=======

``tools/make_test_media.py`` downloads public domain and CC0 recordings from Wikimedia Commons
and makes smaller versions (it needs ``ffmpeg`` and the ``soundfile`` package).
``tests/test_decoder.py`` decodes them with the kernel, run under CPython by a model that fails
on any 32-bit overflow, and with the plain Python decoder, and compares both with ffmpeg.
``tests/test_codebook.py`` and ``tests/test_ogg_reader.py`` need no media.
``tools/kernel_check.py`` compares the C and turbo kernels on a board and times them.

Documentation
=============
API documentation for this library can be found on `Read the Docs <https://docs.circuitpython.org/projects/ogg/en/latest/>`_.

For information on building library documentation, please check out
`this guide <https://learn.adafruit.com/creating-and-sharing-a-circuitpython-library/sharing-our-docs-on-readthedocs#sphinx-5-1>`_.

Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/adafruit/Adafruit_CircuitPython_OGG/blob/HEAD/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.
