#!/bin/bash

# Simbody
# Dependencies
# Some dependencies are repeated throughout the installation script
sudo apt-get -y install cmake liblapack-dev
sudo apt-get -y install freeglut3-dev libxi-dev libxmu-dev
sudo apt-get install doxygen
sudo apt-get -y install git
# Source code
git clone https://github.com/simbody/simbody.git ~/simbody-source
git checkout Simbody-3.7
# Configure and generate Makefiles
mkdir ~/simbody-build
cd ~/simbody-build
cmake ~/simbody-source
make doxygen
make -j8
make -j8 install
# Test your insallation
# cd ~/simbody-build
# ./SimbodyInstallTest

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
