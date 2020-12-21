cmake_minimum_required(VERSION 3.10.0)

project(GlobalReconstructionRGBD)

set(CMAKE_CXX_STANDARD 17)
set(GCC_COVERAGE_COMPILE_FLAGS " -Os")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")

include(ExternalProject)

ExternalProject_Add(siftgpu
        SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/siftgpu"
        BUILD_IN_SOURCE 1
        BUILD_COMMAND $(MAKE)
        INSTALL_COMMAND ""
        )

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

set(GDR_HEADER_FILES

        ${PROJECT_SOURCE_DIR}/include/errors.h
        ${PROJECT_SOURCE_DIR}/include/features.h
        ${PROJECT_SOURCE_DIR}/include/VertexCG.h
        ${PROJECT_SOURCE_DIR}/include/CorrespondenceGraph.h
        ${PROJECT_SOURCE_DIR}/include/transformationRt.h
        ${PROJECT_SOURCE_DIR}/include/cameraRGBD.h
        ${PROJECT_SOURCE_DIR}/include/siftModule.h
        ${PROJECT_SOURCE_DIR}/include/groundTruthTransformer.h
        ${PROJECT_SOURCE_DIR}/include/images.h
        ${PROJECT_SOURCE_DIR}/include/rotationAveraging.h
        ${PROJECT_SOURCE_DIR}/include/util.h
        ${PROJECT_SOURCE_DIR}/include/quaternions.h
        ${PROJECT_SOURCE_DIR}/include/ICP.h
        )

set(GDR_SOURCE_FILES
        ${PROJECT_SOURCE_DIR}/src/features.cpp
        ${PROJECT_SOURCE_DIR}/src/files.cpp
        ${PROJECT_SOURCE_DIR}/src/VertexCG.cpp
        ${PROJECT_SOURCE_DIR}/src/CorrespondenceGraph.cpp
        ${PROJECT_SOURCE_DIR}/src/transformationRt.cpp
        ${PROJECT_SOURCE_DIR}/src/cameraRGBD.cpp
        ${PROJECT_SOURCE_DIR}/src/siftModule.cpp
        ${PROJECT_SOURCE_DIR}/src/groundTruthTransformer.cpp
        ${PROJECT_SOURCE_DIR}/src/images.cpp
        ${PROJECT_SOURCE_DIR}/src/rotationAveraging.cpp
        ${PROJECT_SOURCE_DIR}/src/quaternions.cpp
        ${PROJECT_SOURCE_DIR}/src/util.cpp
        ${PROJECT_SOURCE_DIR}/src/ICP.cpp
        )

add_library(GDR_LIB
        ${GDR_SOURCE_FILES}
        ${GDR_HEADER_FILES}
        )

add_dependencies(GDR_LIB siftgpu)

target_link_libraries(GDR_LIB
        ${Sophus_LIBRARIES}
        ${CUDA_LIBRARIES}
        ${SIFTGPU_LIBS}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${Boost_FILESYSTEM_LIBRARIES}
        ${Boost_LIBRARIES}
        )

enable_testing()
add_subdirectory(test)
