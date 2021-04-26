# CPPND: Capstone Project Repository

<img src="images/robotic_arm.png"/>

Repository for the Capstone project in the [Udacity C++ Nanodegree Program](https://www.udacity.com/course/c-plus-plus-nanodegree--nd213).

This project builds the representation of a 2 deegres of freedom (DOF) robotic arm with capabilities to compute forward and inverse kinematics.

Specifications:
* Angle 0: Rotational DOF that joins the first link with the base.
* Angle 1: Rotation DOF that joins the first and second link.
* Link 1: A 15 cm rigid body.
* Link 2: A 5 cm rigid body.

## Dependencies for Running Locally
* Run the install dependencies script for Linux: 
    ```sh
    chmod +x install_dependencies.sh
    ./install_dependencies.sh
    ```
or

### Install Armadillo 
* Windows installation:
    [instructions to install Armadillo on Windows](https://solarianprogrammer.com/2017/03/24/getting-started-armadillo-cpp-linear-algebra-windows-mac-linux/)
* Mac installation:
    ```sh
    xcode-select --install
    brew tap homebrew/science
    brew info armadillo
    ```

* Linux installation:
    * Install Armadillo dependencies
    ```sh
    sudo apt install cmake libopenblas-dev liblapack-dev
    ```
    * Download the stable version of Armadillo [stable-version](http://arma.sourceforge.net/download.html)
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
4. Run it: `./CapstoneProject`.

## Source Code Structure
* main.cpp: Contains code to generate a trajectory and compute angles to traverse that trajectory using a 2 DOF robotic arm.
* CirculateTrajectory.h/cpp: Generates positions for a circular trajectory with a fixed radius of 12.5 cm.
* Trajectory.h/cpp: Interface to generate a trajectory.
* RoboticArm.h/cpp: Implements forward and inverse kinematics for a 2 DOF robotic arm.
* Forward/InverseKinematics.h/cpp: Interfaces used to implement kinematics.
* Position.h/cpp: Class that holds a mat object with position data.
* Conversions.h/cpp: Converts angles in grades to radians, and vice versa.

## Rubric Points Addressed
* The project demonstrates an understanding of C++ functions and control structures
* The project uses Object Oriented Programming techniques
* Classes use appropriate access specifiers for class members
* Class constructors utilize member initialization lists: e.g. RoboticArm.h/cpp 
* Classes use abstract implementation details from their interfaces: e.g. RoboticArm.h/cpp, CircularTrajectory.h/cpp
* Classes follow an appropriate inheritance hierarchy: e.g. RoboticArm.h/cpp
* Overloaded functions allow the same function to operate on different parameters: e.g. Position.h/cpp
* Derived class functions override virtual base class functions: e.g. RoboticArm.h/cpp

