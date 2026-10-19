Simple test
------------

Ensure your device works with this simple test. It plays ``song.ogg`` from CIRCUITPY through
the Fruit Jam headphone jack.

.. literalinclude:: ../examples/ogg_simpletest.py
    :caption: examples/ogg_simpletest.py
    :linenos:

Fruit Jam microSD playlist
--------------------------

Plays every Ogg Vorbis file on the microSD card, using the card mount CircuitPython sets up.

.. literalinclude:: ../examples/ogg_fruitjam_sd_playlist.py
    :caption: examples/ogg_fruitjam_sd_playlist.py
    :linenos:

Feather with a MAX98357A I2S amp
--------------------------------

Plays ``song.ogg`` through a mono I2S amp and speaker on any Feather, at a lower ``level``.

.. literalinclude:: ../examples/ogg_feather_i2s_amp_max98357a.py
    :caption: examples/ogg_feather_i2s_amp_max98357a.py
    :linenos:
