# CPPND: Capstone Hello World Repo

This is a starter repo for the Capstone project in the [Udacity C++ Nanodegree Program](https://www.udacity.com/course/c-plus-plus-nanodegree--nd213).

The Capstone Project gives you a chance to integrate what you've learned throughout this program. This project will become an important part of your portfolio to share with current and future colleagues and employers.

In this project, you can build your own C++ application starting with this repo, following the principles you have learned throughout this Nanodegree Program. This project will demonstrate that you can independently create applications using a wide range of C++ features.

## Dependencies for Running Locally
* Install Armadillo 
    * Install Armadillo dependencies
    ```sh
    sudo apt install cmake libopenblas-dev liblapack-dev
    ```
    * Download the stable version of Armadillo. [stable-version](http://arma.sourceforge.net/download.html)
    and install it: 
    ```sh
    cd Downloads/
    cd arma*
    cmake .
    make
    sudo make install
    ```

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./HelloWorld`.