cmake_minimum_required(VERSION 2.8.3)
project(vins_estimator)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(OpenCV REQUIRED)

find_package(Pangolin REQUIRED)

# message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)

find_package(Ceres REQUIRED)

find_package(Eigen3)
include_directories(
        ${PROJECT_SOURCE_DIR}/vins_estimator/include
        ${EIGEN3_INCLUDE_DIR}
        ${CERES_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_library(camera_models SHARED
        src/camera_models/chessboard/Chessboard.cc
        src/camera_models/calib/CameraCalibration.cc
        src/camera_models/camera_models/Camera.cc
        src/camera_models/camera_models/CameraFactory.cc
        src/camera_models/camera_models/CostFunctionFactory.cc
        src/camera_models/camera_models/PinholeCamera.cc
        src/camera_models/camera_models/PinholeFullCamera.cc
        src/camera_models/camera_models/CataCamera.cc
        src/camera_models/camera_models/EquidistantCamera.cc
        src/camera_models/camera_models/ScaramuzzaCamera.cc
        src/camera_models/sparse_graph/Transform.cc
        src/camera_models/gpl/gpl.cc
        src/camera_models/gpl/EigenQuaternionParameterization.cc)

target_link_libraries(camera_models ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})


add_library(vins_lib SHARED
    src/System.cpp
    src/estimator/parameters.cpp
    src/estimator/estimator.cpp
    src/estimator/feature_manager.cpp
    src/factor/pose_local_parameterization.cpp
    src/factor/projectionTwoFrameOneCamFactor.cpp
    src/factor/projectionTwoFrameTwoCamFactor.cpp
    src/factor/projectionOneFrameTwoCamFactor.cpp
    src/factor/marginalization_factor.cpp
    src/utility/utility.cpp
    src/utility/visualization.cpp
    src/utility/CameraPoseVisualization.cpp
    src/initial/solve_5pts.cpp
    src/initial/initial_aligment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp
    src/featureTracker/feature_tracker.cpp)

target_link_libraries(vins_lib ${OpenCV_LIBS} ${CERES_LIBRARIES} ${Pangolin_LIBRARIES} camera_models)

add_executable(run_in_sim src/run_in_sim.cpp)
target_link_libraries(run_in_sim 
                      vins_lib 
                      -lpthread)

