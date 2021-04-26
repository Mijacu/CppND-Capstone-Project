#!/bin/bash

# Armadillo
# Dependencies
sudo apt -y install cmake libopenblas-dev liblapack-dev
# Source code
wget http://sourceforge.net/projects/arma/files/armadillo-10.4.1.tar.xz
mv ./armadillo-10.4.1.tar.xz ~
cd ~
tar -xvf armadillo-10.4.1.tar.xz
cd armadillo-10.4.1
cmake .
make
sudo make install
