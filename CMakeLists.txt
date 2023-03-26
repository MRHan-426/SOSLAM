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
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Utilities.cpp
    )
target_link_libraries(${PROJECT_NAME}
    ${EIGEN3_LIBS}
    ${GTSAM_LIBRARIES}
    ${Boost_LIBRARIES})

# Add executable
# add_executable(soslam_exe src/main.cpp)
# target_link_libraries(soslam_exe ${PROJECT_NAME})
