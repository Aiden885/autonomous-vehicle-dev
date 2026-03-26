#! /bin/bash
mkdir build
cd build
rm -r *
cmake ..
make
./main
cd ..
rm -r build
