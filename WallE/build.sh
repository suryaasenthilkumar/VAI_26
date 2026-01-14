#!/bin/bash
# Build script for WallE VEX V5 project

# Add ARM toolchain to PATH
export PATH="/Applications/VEXcode V5.app/Contents/Resources/toolchain/vexv5/osx/gcc/bin:$PATH"

# Build the project
make "$@"
