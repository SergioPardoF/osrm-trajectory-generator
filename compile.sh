#!/bin/sh

echo "Clean project"
sh ./clean.sh

echo "Download external projects"
git submodule init
git submodule update --init --recursive

# Compile OSRM
echo "Compiling OSRM"
mkdir -p external/osrm-backend/build
cd external/osrm-backend/build
cmake ..
make
make install
cd ../../../

# Compile generator
echo "Compiling generator"
mkdir build
cd build
cmake ..
make
cd ..
