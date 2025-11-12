# !/bin/bash

# Run once

mkdir -p build
cd build
cp ../rebuild.sh ./
cmake .. -DCMAKE_PREFIX_PATH=~/Qt/6.9.3/gcc_64
make clean
<<<<<<< HEAD
make -j
=======
make -j${nproc}
>>>>>>> dev-connect-vis-ctrl
