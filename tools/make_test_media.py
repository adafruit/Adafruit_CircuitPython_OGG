# SPDX-FileCopyrightText: Copyright (c) 2026 Mikey Sklar for Adafruit Industries
# SPDX-License-Identifier: MIT
"""Download the Ogg Vorbis test music to media/ and make the smaller versions.

All three recordings are from Wikimedia Commons (Musopen and the European Archive):

- Bach, BWV 147, "Jesus bleibet meine Freude" (CC0)
  https://commons.wikimedia.org/wiki/File:Bach,_BWV_147,_10._Jesus_bleibet_meine_Freude.ogg
- Scriabin, prelude op. 67 no. 1 (public domain)
  https://commons.wikimedia.org/wiki/File:Alexander_Scriabin_-_prelude_no._1,_op._67.ogg
- Mendelssohn, A Midsummer Night's Dream, Dance of Clowns (public domain)
  https://commons.wikimedia.org/wiki/File:A_Midsummer_Night%27s_Dream_Op._61_Dance_of_Clowns_(Mendelssohn)_European_Archive.ogg

The originals are 44.1 kHz stereo. The smaller versions are re-encoded with libvorbis
(through the soundfile package) at the rates a microcontroller plays easily. Each file gets
a ``.raw`` reference (16-bit little endian PCM) decoded by ffmpeg.

Needs ``curl`` and ``ffmpeg`` on the PATH and the ``numpy`` and ``soundfile`` packages.

usage: python3 tools/make_test_media.py
"""

import os
import subprocess

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MEDIA = os.environ.get("OGG_MEDIA") or os.path.join(HERE, "media")
COMMONS = "https://commons.wikimedia.org/wiki/Special:FilePath/"
SOURCES = (
    ("bach_44k_stereo.ogg", "Bach,_BWV_147,_10._Jesus_bleibet_meine_Freude.ogg"),
    ("scriabin_44k_stereo.ogg", "Alexander_Scriabin_-_prelude_no._1,_op._67.ogg"),
    (
        "mendelssohn_44k_stereo.ogg",
        "A_Midsummer_Night%27s_Dream_Op._61_Dance_of_Clowns_(Mendelssohn)_European_Archive.ogg",
    ),
)
# name, made from, rate, channels, libsndfile compression level (0 best .. 1 smallest)
DERIVED = (
    ("bach_22k_mono.ogg", "bach_44k_stereo.ogg", 22050, 1, 0.6),
    ("scriabin_22k_stereo.ogg", "scriabin_44k_stereo.ogg", 22050, 2, 0.6),
    ("mendelssohn_11k_mono.ogg", "mendelssohn_44k_stereo.ogg", 11025, 1, 0.6),
    ("bach_48k_stereo_q1.ogg", "bach_44k_stereo.ogg", 48000, 2, 0.0),
    ("bach_8k_mono_q0.ogg", "bach_44k_stereo.ogg", 8000, 1, 1.0),
)
SECONDS = 40  # length of the derived files


def encode(src, dst, rate, channels, level):
    import numpy as np  # noqa: PLC0415, only needed here
    import soundfile as sf  # noqa: PLC0415

    pcm = subprocess.run(
        ["ffmpeg", "-v", "error", "-t", str(SECONDS), "-i", src, "-ac", str(channels)]
        + ["-ar", str(rate), "-f", "f32le", "-"],
        capture_output=True,
        check=True,
    ).stdout
    data = np.frombuffer(pcm, dtype="<f4").reshape(-1, channels)
    with sf.SoundFile(
        dst, "w", rate, channels, format="OGG", subtype="VORBIS", compression_level=level
    ) as out:
        for i in range(0, len(data), 8192):  # libsndfile can crash on one huge write
            out.write(data[i : i + 8192])


def main():
    os.makedirs(MEDIA, exist_ok=True)
    for name, page in SOURCES:
        path = os.path.join(MEDIA, name)
        if not os.path.exists(path):
            print("downloading", page)
            subprocess.run(
                ["curl", "-sSfL", "-A", "adafruit_ogg test media", "-o", path, COMMONS + page],
                check=True,
            )
    for name, src, rate, channels, level in DERIVED:
        path = os.path.join(MEDIA, name)
        if not os.path.exists(path):
            encode(os.path.join(MEDIA, src), path, rate, channels, level)
    for name in sorted(os.listdir(MEDIA)):
        if name.endswith(".ogg"):
            path = os.path.join(MEDIA, name)
            subprocess.run(
                ["ffmpeg", "-v", "error", "-y", "-i", path, "-f", "s16le", path[:-4] + ".raw"],
                check=True,
            )
            print("%-28s %8d bytes" % (name, os.path.getsize(path)))


if __name__ == "__main__":
    main()
