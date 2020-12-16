cmake_minimum_required(VERSION 3.0.0)
project(GlobalReconstruction)

set(CMAKE_CXX_STANDARD 17)
set(GCC_COVERAGE_COMPILE_FLAGS " -Os -DNDEBUG")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")

find_package(GTSAM REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
#find_package(Pangolin REQUIRED)
#set(ICPCUDA "externals/ICPCUDA")



include(ExternalProject)

ExternalProject_Add(siftgpu
        SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/siftgpu"
        BUILD_IN_SOURCE 1
        BUILD_COMMAND $(MAKE)
        INSTALL_COMMAND ""
        )

#set(Sophus_INCLUDE_DIR "${ICPCUDA}/third-party/Sophus" CACHE PATH "Sophus Include directory")

#add_subdirectory(externals)
#add_subdirectory(externals/gtsam)
#add_library(gtsam)

#include_directories(${ICPCUDA} ${Sophus_INCLUDE_DIR})

# SiftGPU
#include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${OpenGL_INCLUDE_DIR})
include_directories(${CUDA_INCLUDE_DIR})
#include_directories("siftgpu/src/SiftGPU")

set(SIFTGPU_LIBS "${CMAKE_CURRENT_SOURCE_DIR}/siftgpu/bin/libsiftgpu.so")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PCL 1.8 REQUIRED)

find_package(Boost COMPONENTS thread system filesystem REQUIRED)
message("   Boost information:")
message("   Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
message("   Boost_LIBRARIES: ${Boost_LIBRARIES}")
message("   Boost_LIBRARY_DIRS: ${Boost_LIBRARY_DIRS}")

include_directories(${Boost_INCLUDE_DIRS})

link_directories(${Boost_LIBRARY_DIRS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})
add_executable(test1 test/main.cpp src/files.cpp src/features.cpp include/features.h src/VertexCG.cpp include/VertexCG.h src/CorrespondenceGraph.cpp include/CorrespondenceGraph.h src/transformationRt.cpp include/transformationRt.h src/cameraRGBD.cpp include/cameraRGBD.h src/siftModule.cpp include/siftModule.h src/groundTruthTransformer.cpp include/groundTruthTransformer.h src/images.cpp include/images.h src/rotationAveraging.cpp include/rotationAveraging.h src/util.cpp include/util.h src/quaternions.cpp include/quaternions.h src/ICP.cpp include/ICP.h)

target_link_libraries(test1
        ${OpenCV_LIBS}
        ${SIFTGPU_LIBS}
        ${GLEW_LIBRARIES}
        ${GLUT_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${CUDA_LIB_PATH}
        ${PCL_LIBRARIES}
#        ${Sophus_LIBRARIES}
#        ${Pangolin_LIBRARIES}
        Eigen3::Eigen
        pthread
        gtsam
        )
