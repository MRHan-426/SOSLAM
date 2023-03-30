## 1.Check your dependencies, mine are as follows:
---
```shell
cmake 3.26.0
libboost 1.71.0
eigen 3.3.7
```

```shell
sudo apt install libeigen3-dev
sudo apt install libboost-all-dev
sudo apt install libtbb-dev
```

## 2.Download GTSAM 4.1.1

Note that higher version may bring unexpected errors.

```shell
git clone --branch 4.1.1 https://github.com/borglab/gtsam.git
```

## 3.Modify Eigen cmake config file: cmake/HandleEigen.cmake

add below line just after "option(GTSAM_USE_SYSTEM_EIGEN...)"

```shell
set(GTSAM_USE_SYSTEM_EIGEN ON)
```
Then:

```shell

mkdir build
cmake ..
make check
sudo make install
```


## 4.My CMakelist will be like:

```shell
cmake_minimum_required(VERSION 3.18)
project(gtsam_soslam)
set(GTSAM_SOSLAM_VERSION 0.0.1)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra")

# Find packages
find_package(Eigen3 REQUIRED)
find_package(GTSAM 4.1.1 REQUIRED)
find_package(Boost REQUIRED)

# Set include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIR}
    ${GTSAM_INCLUDE_DIR}
)
link_directories(${Boost_LIBRARY_DIR})


# Add library
add_library(${PROJECT_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/AlignedBox2.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/AlignedBox3.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/BoundingBoxFactor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ConstrainedDualQuadric.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/DualConic.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/QuadricAngleFactor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/QuadricCamera.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/SoSlam.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Constants.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Utilities.cpp)
target_link_libraries(${PROJECT_NAME}
    ${EIGEN3_LIBS}
    ${GTSAM_LIBRARIES}
    ${Boost_LIBRARIES})

# Add executable
add_executable(soslam_exe src/main.cpp)
target_link_libraries(soslam_exe ${PROJECT_NAME})
```


I have quite a lot bugs, and have trouble with the visualization part.
