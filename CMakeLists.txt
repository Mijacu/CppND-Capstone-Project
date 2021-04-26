cmake_minimum_required(VERSION 3.7)

project(CppCapstoneProject)

add_definitions(-std=c++17)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

# Armadillo
find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})

# Simbody
find_package(Simbody REQUIRED)
include_directories(${Simbody_INCLUDE_DIR}s)

# Find all executables
file(GLOB project_SRCS src/*.cpp) #src/*.h

# Add project executable
add_executable(CapstoneProject ${project_SRCS})
target_link_libraries(CapstoneProject ${ARMADILLO_LIBRARIES})
target_link_libraries(CapstoneProject ${Simbody_LIBRARIES})
