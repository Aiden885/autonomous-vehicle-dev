#! /bin/bash
mkdir build
cd build
cmake ..
make
./main
cd ..
#rm -r build