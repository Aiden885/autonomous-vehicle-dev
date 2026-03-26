#! /bin/bash
apt-get update
apt-get install -y xvfb
Xvfb :1 -screen 0 1024x768x16 &
export DISPLAY=:1
mkdir build
cd build
rm -r *
cmake ..
make
./main
cd ..
rm -r build
