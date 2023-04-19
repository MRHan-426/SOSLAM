cmake_minimum_required(VERSION 3.18)
project(gtsam_soslam)
set(GTSAM_SOSLAM_VERSION 0.0.1)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter -std=c++14 -Wall -Wextra")

# Find packages
find_package(Eigen3 REQUIRED)
find_package(GTSAM 4.1.1 REQUIRED)
find_package(Boost REQUIRED)
find_package(TBB REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Pangolin 0.8 REQUIRED)
find_package(pugixml REQUIRED)
find_package(PCL REQUIRED)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") # use this in Ubuntu 16.04

# Set include directories
include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIR}
        ${GTSAM_INCLUDE_DIR}
        ${TBB_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS})

# Add library
add_library(${PROJECT_NAME}
        ${PROJECT_SOURCE_DIR}/src/AlignedBox2.cpp
        ${PROJECT_SOURCE_DIR}/src/BoundingBoxFactor.cpp
        ${PROJECT_SOURCE_DIR}/src/builder.cpp
        ${PROJECT_SOURCE_DIR}/src/ConstrainedDualQuadric.cpp
        ${PROJECT_SOURCE_DIR}/src/Constants.cpp
        ${PROJECT_SOURCE_DIR}/src/DualConic.cpp
        ${PROJECT_SOURCE_DIR}/src/Evaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/FrameDrawer.cc
        ${PROJECT_SOURCE_DIR}/src/Geometry.cpp
        ${PROJECT_SOURCE_DIR}/src/HandMadeData.cpp
        ${PROJECT_SOURCE_DIR}/src/Map.cpp
        ${PROJECT_SOURCE_DIR}/src/MapDrawer.cc
        ${PROJECT_SOURCE_DIR}/src/PlaneSupportingFactor.cpp
        ${PROJECT_SOURCE_DIR}/src/Polygon.cpp
        ${PROJECT_SOURCE_DIR}/src/QuadricCamera.cpp
        ${PROJECT_SOURCE_DIR}/src/SemanticScaleFactor.cpp
        ${PROJECT_SOURCE_DIR}/src/SoSlam.cpp
        ${PROJECT_SOURCE_DIR}/src/SymmetryFactor.cpp
        ${PROJECT_SOURCE_DIR}/src/Utilities.cpp
        ${PROJECT_SOURCE_DIR}/src/Viewer.cc)

target_link_libraries(${PROJECT_NAME}
        gtsam
        pango_display
        pugixml
        ${EIGEN3_LIBS}
        ${Boost_LIBRARIES}
        ${TBB_LIBRARIES}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES})

# Add executable
add_executable(soslam_exe src/main.cpp)
target_link_libraries(soslam_exe
        gtsam
        pango_display
        pugixml
        ${PROJECT_NAME}
        ${EIGEN3_LIBS}
        ${Boost_LIBRARIES}
        ${TBB_LIBRARIES}
        ${OpenCV_LIBS}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES})
