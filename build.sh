# !/bin/bash

# Run once

mkdir -p build
cd build
cp ../rebuild.sh ./
cmake ..
make clean
make -j