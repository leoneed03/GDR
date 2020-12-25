cmake_minimum_required(VERSION 3.10.0)

project(GDR)

set(CMAKE_CXX_STANDARD 17)
set(GCC_COVERAGE_COMPILE_FLAGS " -Os")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules")
set(GDR_FLAGS "")

include(ExternalProject)

ExternalProject_Add(siftgpu
        SOURCE_DIR "${PROJECT_SOURCE_DIR}/siftgpu"
        BUILD_IN_SOURCE 1
        BUILD_COMMAND $(MAKE)
        INSTALL_COMMAND ""
        )

find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(CUDA REQUIRED)
find_package(Eigen3 REQUIRED)

FIND_PACKAGE(Boost COMPONENTS program_options timer filesystem serialization REQUIRED)

if (${DEBUG_PRINT_PROGRESS})
    list(APPEND GDR_FLAGS "-DDEBUG_PRINT_PROGRESS")
endif ()

set(SIFTGPU_LIBS "${PROJECT_SOURCE_DIR}/siftgpu/bin/libsiftgpu.so")

set(GDR_HEADER_FILES
        ${PROJECT_SOURCE_DIR}/include/keyFeatures.h
        ${PROJECT_SOURCE_DIR}/include/fileProc.h
        ${PROJECT_SOURCE_DIR}/include/VertexCG.h
        ${PROJECT_SOURCE_DIR}/include/CorrespondenceGraph.h
        ${PROJECT_SOURCE_DIR}/include/transformationRt.h
        ${PROJECT_SOURCE_DIR}/include/cameraRGBD.h
        ${PROJECT_SOURCE_DIR}/include/siftModule.h
        ${PROJECT_SOURCE_DIR}/include/groundTruthTransformer.h
        ${PROJECT_SOURCE_DIR}/include/rotationAveraging.h
        ${PROJECT_SOURCE_DIR}/include/quaternions.h
        ${PROJECT_SOURCE_DIR}/include/util.h
        ${PROJECT_SOURCE_DIR}/include/ICP.h
        ${PROJECT_SOURCE_DIR}/include/umeyama.h
        ${PROJECT_SOURCE_DIR}/include/poseEstimation.h
        ${PROJECT_SOURCE_DIR}/include/pseudonames.h
        ${PROJECT_SOURCE_DIR}/include/printer.h
        ${PROJECT_SOURCE_DIR}/include/errors.h
        )

set(GDR_SOURCE_FILES
        ${PROJECT_SOURCE_DIR}/src/keyFeatures.cpp
        ${PROJECT_SOURCE_DIR}/src/fileProc.cpp
        ${PROJECT_SOURCE_DIR}/src/VertexCG.cpp
        ${PROJECT_SOURCE_DIR}/src/CorrespondenceGraph.cpp
        ${PROJECT_SOURCE_DIR}/src/transformationRt.cpp
        ${PROJECT_SOURCE_DIR}/src/cameraRGBD.cpp
        ${PROJECT_SOURCE_DIR}/src/siftModule.cpp
        ${PROJECT_SOURCE_DIR}/src/groundTruthTransformer.cpp
        ${PROJECT_SOURCE_DIR}/src/rotationAveraging.cpp
        ${PROJECT_SOURCE_DIR}/src/quaternions.cpp
        ${PROJECT_SOURCE_DIR}/src/util.cpp
        ${PROJECT_SOURCE_DIR}/src/ICP.cpp
        ${PROJECT_SOURCE_DIR}/src/umeyama.cpp
        ${PROJECT_SOURCE_DIR}/src/poseEstimation.cpp
        )

add_library(GDR_LIB
        ${GDR_SOURCE_FILES}
        ${GDR_HEADER_FILES}
        )

add_dependencies(GDR_LIB
        siftgpu
        )

target_compile_definitions(GDR_LIB
        PUBLIC
        ${GDR_FLAGS}
        )
target_link_libraries(GDR_LIB
        ${CUDA_LIBRARIES}
        ${SIFTGPU_LIBS}
        ${OpenCV_LIBS}
        ${Boost_FILESYSTEM_LIBRARIES}
        ${Boost_LIBRARIES}
        )

target_include_directories(GDR_LIB
        PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/siftgpu/src/SiftGPU
        PRIVATE
        ${OpenGL_INCLUDE_DIR}
        )

enable_testing()
add_subdirectory(test)
