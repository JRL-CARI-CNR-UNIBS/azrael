#!/usr/bin/env bash
set -e

cd /tmp
rm -rf iir1

git clone --depth 1 https://github.com/berndporr/iir1.git
cd iir1
mkdir build
cd build
cmake ..
make -j"$(nproc)"
sudo make install
sudo ldconfig
