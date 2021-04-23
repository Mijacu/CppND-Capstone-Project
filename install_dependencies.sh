#!/bin/bash

# Armadillo
wget http://sourceforge.net/projects/arma/files/armadillo-10.4.1.tar.xz
mv ./armadillo-10.4.1.tar.xz ~
cd ~
tar -xvf armadillo-10.4.1.tar.xz
cd armadillo-10.4.1
cmake .
make
make install
