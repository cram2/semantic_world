#!/bin/bash
source /opt/ros/jazzy/setup.bash
# Determine the directory of this script and change to the examples directory relative to it
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXAMPLES_DIR="$(cd "$SCRIPT_DIR/../examples" && pwd)"
cd "$EXAMPLES_DIR"
rm -rf tmp
mkdir tmp
jupytext --to notebook *.md
mv *.ipynb tmp
cd tmp
treon --thread 1 -v