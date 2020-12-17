cmake_minimum_required(VERSION 3.10.0)

project(GlobalReconstructionICP)
cmake_minimum_required(VERSION 3.10.0)

project(GlobalReconstructionICP)

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


find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
#find_package(Pangolin REQUIRED)
find_package(CUDA REQUIRED)
find_package(PCL 1.8 REQUIRED)

#include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIR})
#include_directories(${ICPCUDA})

#file(GLOB srcs ${ICPCUDA}/ICPOdometry.cpp ${ICPCUDA}/*.cu ${ICPCUDA}/*.h ${ICPCUDA}/*.cuh)
#file(GLOB cuda ${ICPCUDA}/Cuda/*.cu ${ICPCUDA}/ICPOdometry.cpp ${ICPCUDA}/Cuda/*.cuh ${ICPCUDA}/*.h)
#file(GLOB containers ${ICPCUDA}/Cuda/containers/*.cpp ${ICPCUDA}/Cuda/containers/*.h ${ICPCUDA}/containers/*.cu ${ICPCUDA}/Cuda/containers/*.cuh)

#set(CUDA_ARCH_BIN "" CACHE STRING "Specify 'real' GPU arch to build binaries for, BIN(PTX) format is supported. Example: 1.3 2.1(1.3) or 13 21(13)")
#set(CUDA_ARCH_PTX "" CACHE STRING "Specify 'virtual' PTX arch to build PTX intermediate code for. Example: 1.0 1.2 or 10 12")
#
#include("CudaDetect.cmake")
#detect_installed_gpus(CUDA_NVCC_ARCHS)
#foreach(NVCC_ARCH IN LISTS CUDA_NVCC_ARCHS)
#    list(APPEND CUDA_ARCH_BIN "${NVCC_ARCH} ")
#endforeach(NVCC_ARCH)


#include("CudaComputeTargetFlags.cmake")
#append_target_arch_flags()
#
#set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}  "-Xcompiler;-fPIC;--expt-relaxed-constexpr")
#
#CUDA_COMPILE(cuda_objs ${cuda})


# SiftGPU
#include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${OpenGL_INCLUDE_DIR})
include_directories(${CUDA_INCLUDE_DIR})


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/siftgpu/src/SiftGPU)

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
#set(CMAKE_CXX_FLAGS "-O3 -msse2 -msse3 -Wall -std=c++11")


add_dependencies(CG siftgpu)

add_executable(CG
#        ${srcs}
#        ${cuda}
#        ${cuda_objs}
#        ${containers}
        testICPCUDA.cpp
        src/files.cpp src/features.cpp include/features.h src/VertexCG.cpp include/VertexCG.h src/CorrespondenceGraph.cpp include/CorrespondenceGraph.h src/transformationRt.cpp include/transformationRt.h src/cameraRGBD.cpp include/cameraRGBD.h src/siftModule.cpp include/siftModule.h src/groundTruthTransformer.cpp include/groundTruthTransformer.h test/testEigen.cpp src/images.cpp include/images.h src/rotationAveraging.cpp include/rotationAveraging.h src/util.cpp include/util.h src/quaternions.cpp include/quaternions.h src/ICP.cpp include/ICP.h
        )
add_executable(transformGroundTruth test/transformGroundTruth.cpp src/groundTruthTransformer.cpp src/files.cpp src/quaternions.cpp src/util.cpp)

target_link_libraries(transformGroundTruth ${Boost_LIBRARIES})

target_link_libraries(CG
        ${Sophus_LIBRARIES}
        ${CUDA_LIBRARIES}
        ${SIFTGPU_LIBS}
        ${GLEW_LIBRARIES}
        ${GLUT_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${CUDA_LIB_PATH}
        ${PCL_LIBRARIES}
        gtsam
        )
#
#
#cmake_minimum_required(VERSION 2.8.3)
#project(GlobalReconstruction)
#
#set(CMAKE_CXX_STANDARD 17)
#set(GCC_COVERAGE_COMPILE_FLAGS " -Os -DNDEBUG")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
#
#find_package(GTSAM REQUIRED)
#find_package(OpenCV REQUIRED)
#find_package(OpenGL REQUIRED)
#find_package(GLUT REQUIRED)
#find_package(GLEW REQUIRED)
#find_package(Pangolin REQUIRED)
#find_package(CUDA REQUIRED)
#find_package(Eigen3 3.3 REQUIRED NO_MODULE)
#find_package(PCL 1.8 REQUIRED)
#
#set(ICPCUDA "externals/ICPCUDA")
#set(Sophus_INCLUDE_DIR "${ICPCUDA}/third-party/Sophus" CACHE PATH "Sophus Include directory")
#
#add_subdirectory(externals)
#add_subdirectory(externals/ICPCUDA)
#include_directories(${ICPCUDA} ${Sophus_INCLUDE_DIR})
#
## SiftGPU
#include_directories(${Pangolin_INCLUDE_DIRS})
#include_directories(${OpenGL_INCLUDE_DIR})
#include_directories(${CUDA_INCLUDE_DIR})
#include_directories(${CUDA_INCLUDE_DIRS})
#include_directories("siftgpu/src/SiftGPU")
#
#set(SIFTGPU_LIBS "${CMAKE_CURRENT_SOURCE_DIR}/siftgpu/bin/libsiftgpu.so")
#
#file(GLOB srcs ${ICPCUDA}/ICPOdometry.cpp ${ICPCUDA}/*.cu ${ICPCUDA}/*.h ${ICPCUDA}/*.cuh)
#file(GLOB cuda ${ICPCUDA}/Cuda/*.cu ${ICPCUDA}/ICPOdometry.cpp ${ICPCUDA}/Cuda/*.cuh ${ICPCUDA}/Cuda/*.h)
#file(GLOB containers ${ICPCUDA}/Cuda/containers/*.cpp ${ICPCUDA}/Cuda/containers/*.h ${ICPCUDA}/Cuda/containers/*.cu ${ICPCUDA}/Cuda/containers/*.cuh)
#
#
#set(CUDA_ARCH_BIN "" CACHE STRING "Specify 'real' GPU arch to build binaries for, BIN(PTX) format is supported. Example: 1.3 2.1(1.3) or 13 21(13)")
#set(CUDA_ARCH_PTX "" CACHE STRING "Specify 'virtual' PTX arch to build PTX intermediate code for. Example: 1.0 1.2 or 10 12")
#
#include("CudaDetect.cmake")
#detect_installed_gpus(CUDA_NVCC_ARCHS)
#foreach(NVCC_ARCH IN LISTS CUDA_NVCC_ARCHS)
#    list(APPEND CUDA_ARCH_BIN "${NVCC_ARCH} ")
#endforeach(NVCC_ARCH)
#
#include("CudaComputeTargetFlags.cmake")
#append_target_arch_flags()
#
#set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}  "-Xcompiler;-fPIC;--expt-relaxed-constexpr")
#
#CUDA_COMPILE(cuda_objs ${cuda})
#
#find_package(Boost COMPONENTS thread system filesystem REQUIRED)
#message("   Boost information:")
#message("   Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
#message("   Boost_LIBRARIES: ${Boost_LIBRARIES}")
#message("   Boost_LIBRARY_DIRS: ${Boost_LIBRARY_DIRS}")
#
#include_directories(${Boost_INCLUDE_DIRS})
#
#link_directories(${Boost_LIBRARY_DIRS})
#
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})
#
#add_executable(test ${srcs} ${cuda} ${cuda_objs}  ${containers} test/main.cpp src/files.cpp src/features.cpp include/features.h src/VertexCG.cpp include/VertexCG.h src/CorrespondenceGraph.cpp include/CorrespondenceGraph.h src/transformationRt.cpp include/transformationRt.h src/cameraRGBD.cpp include/cameraRGBD.h src/siftModule.cpp include/siftModule.h src/groundTruthTransformer.cpp include/groundTruthTransformer.h test/testEigen.cpp src/images.cpp include/images.h src/rotationAveraging.cpp include/rotationAveraging.h src/util.cpp include/util.h src/quaternions.cpp include/quaternions.h src/ICP.cpp include/ICP.h)
##add_executable(trySIFT test/trysift.cpp src/files.cpp src/features.cpp include/features.h src/VertexCG.cpp include/VertexCG.h src/CorrespondenceGraph.cpp include/CorrespondenceGraph.h)
##add_executable(transformGroundTruth test/transformGroundTruth.cpp src/groundTruthTransformer.cpp src/files.cpp)
##add_executable(testEigen test/testEigen.cpp)
#
##target_link_libraries(transformGroundTruth ${Boost_LIBRARIES})
#
#
#target_link_libraries(test
#        ${TBB_LIBRARIES}
#        ${OpenCV_LIBS}
#        ${SIFTGPU_LIBS}
#        ${GLEW_LIBRARIES}
#        ${GLUT_LIBRARIES}
#        ${OPENGL_LIBRARIES}
#        ${CUDA_LIBS_11}
#        ${CUDA_LIB_PATH}
#        ${CUDA_LIBRARIES}
#        ${PCL_LIBRARIES}
#        ${Sophus_LIBRARIES}
#        ${Pangolin_LIBRARIES}
#        Eigen3::Eigen
#        pthread
#        gtsam
#        )
##target_include_directories(test ${ICPCUDA})
#
##target_link_libraries(trySIFT
##        ${OpenCV_LIBS}
##        ${SIFTGPU_LIBS}
##        ${GLEW_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES}
##        ${CUDA_LIB_PATH}
##        )
