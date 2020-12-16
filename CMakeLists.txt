cmake_minimum_required(VERSION 3.10.0)

project(GlobalReconstruction)

set(CMAKE_CXX_STANDARD 17)
set(GCC_COVERAGE_COMPILE_FLAGS " -Os -DNDEBUG")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")

include(ExternalProject)

ExternalProject_Add(siftgpu
        SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/siftgpu"
        BUILD_IN_SOURCE 1
        BUILD_COMMAND $(MAKE)
        INSTALL_COMMAND ""
        )

find_package(GTSAM REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(CUDA REQUIRED)
find_package(PCL 1.8 REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIR})
include_directories(${OpenGL_INCLUDE_DIR})
include_directories(${CUDA_INCLUDE_DIR})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/siftgpu/src/SiftGPU)

set(SIFTGPU_LIBS "${CMAKE_CURRENT_SOURCE_DIR}/siftgpu/bin/libsiftgpu.so")


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(CG
        test/main.cpp
        src/files.cpp src/features.cpp include/features.h src/VertexCG.cpp include/VertexCG.h src/CorrespondenceGraph.cpp include/CorrespondenceGraph.h src/transformationRt.cpp include/transformationRt.h src/cameraRGBD.cpp include/cameraRGBD.h src/siftModule.cpp include/siftModule.h src/groundTruthTransformer.cpp include/groundTruthTransformer.h src/images.cpp include/images.h src/rotationAveraging.cpp include/rotationAveraging.h src/util.cpp include/util.h src/quaternions.cpp include/quaternions.h src/ICP.cpp include/ICP.h
        )

target_link_libraries(CG
        ${Sophus_LIBRARIES}
        ${CUDA_LIBRARIES}
        ${SIFTGPU_LIBS}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        gtsam
        )
