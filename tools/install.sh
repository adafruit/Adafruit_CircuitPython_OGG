#!/bin/sh
# SPDX-FileCopyrightText: Copyright (c) 2026 Mikey Sklar for Adafruit Industries
# SPDX-License-Identifier: MIT
#
# Copy adafruit_ogg, test music (to /ogg, plus /song.ogg for the simple test) and optionally
# one script (as code.py) to a mounted CIRCUITPY drive. An existing code.py is kept as
# code.py.bak.
#
# usage: tools/install.sh /Volumes/CIRCUITPY [script]
#   script: kernel_check (from tools/), or an example name such as ogg_simpletest
#   (from examples/)
#
# Check boot_out.txt first when several boards are plugged in: CIRCUITPY volume names get
# renumbered.
set -e
dest="$1"
example="$2"
here="$(cd "$(dirname "$0")/.." && pwd)"
[ -d "$dest" ] || { echo "usage: $0 /path/to/CIRCUITPY [script]"; exit 1; }
[ -f "$here/media/bach_22k_mono.ogg" ] || python3 "$here/tools/make_test_media.py"

mkdir -p "$dest/lib" "$dest/ogg"
rm -rf "$dest/lib/adafruit_ogg"
mkdir "$dest/lib/adafruit_ogg"
cp "$here"/adafruit_ogg/*.py "$here"/adafruit_ogg/*.mpy "$dest/lib/adafruit_ogg/"
for f in bach_22k_mono mendelssohn_11k_mono scriabin_22k_stereo scriabin_44k_stereo; do
    cp "$here/media/$f.ogg" "$dest/ogg/$f.ogg"
done
cp "$here/media/scriabin_44k_stereo.ogg" "$dest/song.ogg"
if [ -n "$example" ]; then
    src="$here/tools/$example.py"
    [ -f "$src" ] || src="$here/examples/$example.py"
    [ -f "$src" ] || { echo "no script $example in tools/ or examples/"; exit 1; }
    [ -f "$dest/code.py" ] && cp "$dest/code.py" "$dest/code.py.bak"
    cp "$src" "$dest/code.py"
fi
sync
echo "installed adafruit_ogg ${example:+and $example as code.py }on $dest"
