cmake_minimum_required(VERSION 3.10.0)

project(GDR)

set(CMAKE_CXX_STANDARD 17)
set(GCC_COVERAGE_COMPILE_FLAGS " -Os -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules")
set(GDR_FLAGS "")

find_package(TBB REQUIRED)
find_package(METIS REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(CUDA REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(
  Boost
  COMPONENTS program_options timer filesystem serialization
  REQUIRED)

set(local_custom_Pangolin_INSTALL_DIR _installed_Pangolin)
set(custom_Pangolin_BUILD_NAME g-build)

set(ICPCUDA_PROJECT_PATH ${PROJECT_SOURCE_DIR}/third-party/ICPCUDA)
set(ICPCUDA_LIBS ${ICPCUDA_PROJECT_PATH}/${custom_Pangolin_BUILD_NAME}/libICP.so)
set(ICPCUDA_third_party ${ICPCUDA_PROJECT_PATH}/third-party)
set(ICPCUDA_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/third-party/ICPCUDA/include")

set(Pangolin_INSTALL_DIR "${PROJECT_SOURCE_DIR}/third-party/ICPCUDA/${custom_Pangolin_BUILD_NAME}/${local_custom_Pangolin_INSTALL_DIR}" CACHE PATH "Pangolin install directory")
set(Pangolin_INCLUDE_DIRS "${Pangolin_INSTALL_DIR}/include")
set(Pangolin_LIBRARIES ${Pangolin_INSTALL_DIR}/lib/libpangolin.so)

set(Sophus_INCLUDE_DIRS ${ICPCUDA_third_party}/Sophus)

include(ExternalProject)

ExternalProject_Add(
    icpCuda
    SOURCE_DIR "${PROJECT_SOURCE_DIR}/third-party/ICPCUDA"
    BINARY_DIR "${PROJECT_SOURCE_DIR}/third-party/ICPCUDA/${custom_Pangolin_BUILD_NAME}"
    CMAKE_ARGS -DPangolin_LOCAL_INSTALL_DIR=${local_custom_Pangolin_INSTALL_DIR}
    -DPangolin_BUILD_NAME=${custom_Pangolin_BUILD_NAME}
    INSTALL_COMMAND "")

ExternalProject_Add(
  siftgpu
  SOURCE_DIR ${PROJECT_SOURCE_DIR}/siftgpu
  BUILD_IN_SOURCE 1
  BUILD_COMMAND $(MAKE) -j1
  INSTALL_COMMAND "")

 ExternalProject_Add(gtsam SOURCE_DIR "${PROJECT_SOURCE_DIR}/third-party/gtsam"
 BINARY_DIR "${PROJECT_SOURCE_DIR}/third-party/gtsam/${custom_Pangolin_BUILD_NAME}" CONFIGURE_COMMAND
 cmake .. -DBUILD_SHARED_LIBS=0 -DGTSAM_USE_SYSTEM_EIGEN=1
 -DCMAKE_INSTALL_PREFIX:PATH=${PROJECT_SOURCE_DIR}/third-party/gtsam/${custom_Pangolin_BUILD_NAME}
 INSTALL_COMMAND make install )

if(${DEBUG_PRINT_PROGRESS})
  list(APPEND GDR_FLAGS "-DDEBUG_PRINT_PROGRESS")
endif()

set(SIFTGPU_LIBS ${PROJECT_SOURCE_DIR}/siftgpu/bin/libsiftgpu.so)
set(GTSAM_LIBS ${PROJECT_SOURCE_DIR}/third-party/gtsam/${custom_Pangolin_BUILD_NAME}/lib/libgtsam.a)
set(GTSAM_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/third-party/gtsam/${custom_Pangolin_BUILD_NAME}/include)



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
    ${PROJECT_SOURCE_DIR}/include/umeyama.h
    ${PROJECT_SOURCE_DIR}/include/poseEstimation.h
    ${PROJECT_SOURCE_DIR}/include/printer.h
    ${PROJECT_SOURCE_DIR}/include/errors.h
    ${PROJECT_SOURCE_DIR}/include/ICP.h
    ${PROJECT_SOURCE_DIR}/include/pointCloud.h
    ${PROJECT_SOURCE_DIR}/include/translationMeasurement.h
    ${PROJECT_SOURCE_DIR}/include/translationAveraging.h
    ${PROJECT_SOURCE_DIR}/include/poseInfo.h)

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
    ${PROJECT_SOURCE_DIR}/src/umeyama.cpp
    ${PROJECT_SOURCE_DIR}/src/poseEstimation.cpp
    ${PROJECT_SOURCE_DIR}/src/ICP.cpp
    ${PROJECT_SOURCE_DIR}/src/pointCloud.cpp
    ${PROJECT_SOURCE_DIR}/src/translationAveraging.cpp
    ${PROJECT_SOURCE_DIR}/src/translationMeasurement.cpp
    ${PROJECT_SOURCE_DIR}/src/poseInfo.cpp)

add_library(GDR_LIB SHARED ${GDR_SOURCE_FILES} ${GDR_HEADER_FILES})

add_dependencies(GDR_LIB siftgpu icpCuda gtsam)

target_compile_definitions(GDR_LIB PUBLIC ${GDR_FLAGS})

target_link_libraries(
  GDR_LIB
  ${CUDA_LIBRARIES}
  ${SIFTGPU_LIBS}
  ${OpenCV_LIBS}
  ${GTSAM_LIBS}
  ${Boost_LIBRARIES}
  ${METIS_LIBRARIES}
  ${TBB_LIBRARIES}
  ${ICPCUDA_LIBS}
  ${Pangolin_LIBRARIES}
  ${CUDA_LIBRARIES})

target_include_directories(
  GDR_LIB
  PUBLIC ${GTSAM_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/siftgpu/src/SiftGPU
        ${EIGEN3_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        ${ICPCUDA_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
  PRIVATE ${OpenGL_INCLUDE_DIR})

add_executable(main main.cpp)
add_dependencies(main icpCuda)

target_link_libraries(main
    ${ICPCUDA_LIBS}
    ${Pangolin_LIBRARIES}
    ${CUDA_LIBRARIES}
    )

target_include_directories(main
    PUBLIC
    ${Sophus_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${ICPCUDA_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    )

enable_testing()
add_subdirectory(test)
