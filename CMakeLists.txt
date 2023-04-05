cmake_minimum_required(VERSION 3.18)
project(gtsam_soslam)
set(GTSAM_SOSLAM_VERSION 0.0.1)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter -std=c++14 -Wall -Wextra")
# set(OpenCV_DIR /path/to/opencv)

# Find packages
find_package(Eigen3 REQUIRED)
find_package(GTSAM 4.1.1 REQUIRED)
find_package(Boost REQUIRED)
find_package(TBB REQUIRED) # Add this line
find_package(OpenCV REQUIRED)
find_package(Pangolin 0.8 REQUIRED)
find_package(pugixml REQUIRED)

# Set include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIR}
    ${GTSAM_INCLUDE_DIR}
    ${TBB_INCLUDE_DIRS} 
    ${OpenCV_INCLUDE_DIRS} # Add this line
    ${Pangolin_INCLUDE_DIRS}
)
#link_directories(${Boost_LIBRARY_DIR})


# Add library
add_library(${PROJECT_NAME}
    ${PROJECT_SOURCE_DIR}/src/AlignedBox2.cpp
    ${PROJECT_SOURCE_DIR}/src/AlignedBox3.cpp
    ${PROJECT_SOURCE_DIR}/src/BoundingBoxFactor.cpp
    ${PROJECT_SOURCE_DIR}/src/ConstrainedDualQuadric.cpp
    ${PROJECT_SOURCE_DIR}/src/DualConic.cpp
    ${PROJECT_SOURCE_DIR}/src/QuadricAngleFactor.cpp
    ${PROJECT_SOURCE_DIR}/src/QuadricCamera.cpp
    ${PROJECT_SOURCE_DIR}/src/SoSlam.cpp
    ${PROJECT_SOURCE_DIR}/src/Constants.cpp
    ${PROJECT_SOURCE_DIR}/src/Utilities.cpp
    ${PROJECT_SOURCE_DIR}/src/SemanticScaleFactor.cpp
    ${PROJECT_SOURCE_DIR}/src/PlaneSupportingFactor.cpp
    ${PROJECT_SOURCE_DIR}/src/Viewer.cc
    ${PROJECT_SOURCE_DIR}/src/MapDrawer.cc
#    ${PROJECT_SOURCE_DIR}/src/FrameDrawer.cc
    ${PROJECT_SOURCE_DIR}/src/HandMadeData.cpp
    ${PROJECT_SOURCE_DIR}/src/Evaluation.cpp)

target_link_libraries(${PROJECT_NAME}
        gtsam
        pango_display
        pango_python
        pugixml
        ${EIGEN3_LIBS}
        ${Boost_LIBRARIES}
        ${TBB_LIBRARIES} # Add this line
        ${OpenCV_LIBS}
        )

# Add executable
add_executable(soslam_exe src/main.cpp)
target_link_libraries(soslam_exe
        gtsam
        pango_display
        pango_python
        pugixml
        ${EIGEN3_LIBS}
        ${Boost_LIBRARIES}
        ${TBB_LIBRARIES}
        ${OpenCV_LIBS}
        ${PROJECT_NAME}
        ${OpenCV_LIBS}
)



