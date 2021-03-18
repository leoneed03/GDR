cmake_minimum_required(VERSION 3.12.0)

project(GDR)
#enable_language(CUDA)
set(CMAKE_CXX_STANDARD 17)
set(GCC_COVERAGE_COMPILE_FLAGS " -O3")

#set(GCC_COVERAGE_COMPILE_FLAGS " -march=native")

set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules")
set(GDR_FLAGS "")
set(OpenGL_GL_PREFERENCE "LEGACY")

find_package(TBB REQUIRED)
find_package(METIS REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
#find_package(GLEW REQUIRED)
find_package(CUDA REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

#find_package(Python3 REQUIRED COMPONENTS Development Interpreter NumPy)
find_package(Ceres REQUIRED)

find_package(
  Boost
  COMPONENTS program_options timer filesystem serialization graph
  REQUIRED)

set(local_custom_Pangolin_INSTALL_DIR _installed_Pangolin)
set(custom_Pangolin_BUILD_NAME g-build-concurrent)

set(ICPCUDA_PROJECT_PATH ${PROJECT_SOURCE_DIR}/third-party/ICPCUDA)
set(ICPCUDA_LIBS ${ICPCUDA_PROJECT_PATH}/${custom_Pangolin_BUILD_NAME}/libICP.so)
set(ICPCUDA_third_party ${ICPCUDA_PROJECT_PATH}/third-party)
set(ICPCUDA_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/third-party/ICPCUDA/include")

set(Pangolin_INSTALL_DIR "${PROJECT_SOURCE_DIR}/third-party/ICPCUDA/${custom_Pangolin_BUILD_NAME}/${local_custom_Pangolin_INSTALL_DIR}" CACHE PATH "Pangolin install directory")
set(Pangolin_INCLUDE_DIRS "${Pangolin_INSTALL_DIR}/include")
set(Pangolin_LIBRARIES ${Pangolin_INSTALL_DIR}/lib/libpangolin.so)

set(Sophus_INCLUDE_DIRS ${ICPCUDA_third_party}/Sophus)

set(GTSAM_BUILD_NAME mbuild-concurrent)

include(ExternalProject)

ExternalProject_Add(
    icpCuda
    SOURCE_DIR "${PROJECT_SOURCE_DIR}/third-party/ICPCUDA"
    BINARY_DIR "${PROJECT_SOURCE_DIR}/third-party/ICPCUDA/${custom_Pangolin_BUILD_NAME}"
    CMAKE_ARGS -DPangolin_LOCAL_INSTALL_DIR=${local_custom_Pangolin_INSTALL_DIR}
    -DPangolin_BUILD_NAME=${custom_Pangolin_BUILD_NAME}
    INSTALL_COMMAND "")

ExternalProject_Add(gtsam SOURCE_DIR "${PROJECT_SOURCE_DIR}/third-party/gtsam"
 BINARY_DIR "${PROJECT_SOURCE_DIR}/third-party/gtsam/${GTSAM_BUILD_NAME}" CONFIGURE_COMMAND
 cmake .. -DBUILD_SHARED_LIBS=0 -DGTSAM_USE_SYSTEM_EIGEN=1 -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
     -DCMAKE_INSTALL_PREFIX:PATH=${PROJECT_SOURCE_DIR}/third-party/gtsam/${GTSAM_BUILD_NAME}
 INSTALL_COMMAND make install )

if(${DEBUG_PRINT_PROGRESS})
  list(APPEND GDR_FLAGS "-DDEBUG_PRINT_PROGRESS")
endif()

#set(SIFTGPU_LIBS ${PROJECT_SOURCE_DIR}/siftgpu/bin/libsiftgpu.so)
#set(SIFTGPU_LIBS /home/leo/Desktop/projects/linuxSift/SiftGPU_Linux/siftgpu/bin/libsiftgpu.so)
#set(SIFTGPU_LIBS /home/leo/CLionProjects/colmap/lib/SiftGPU/cmake-build-debug/libsift_gpu.a)
#set(GTSAM_LIBS ${PROJECT_SOURCE_DIR}/third-party/gtsam/${custom_Pangolin_BUILD_NAME}/lib/libgtsam.a)
#set(GTSAM_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/third-party/gtsam/${custom_Pangolin_BUILD_NAME}/include)

set(GTSAM_LIBS ${PROJECT_SOURCE_DIR}/third-party/gtsam/${GTSAM_BUILD_NAME}/lib/libgtsam.a)
set(GTSAM_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/third-party/gtsam/${GTSAM_BUILD_NAME}/include)

set(GDR_HEADER_FILES
        include/filesystem/fileProc.h
    ${PROJECT_SOURCE_DIR}/include/VertexCG.h
    ${PROJECT_SOURCE_DIR}/include/CorrespondenceGraph.h
        include/parametrization/transformationRt.h
        include/parametrization/cameraRGBD.h
        include/keyPointDetectionAndMatching/SiftModuleGPU.h
    ${PROJECT_SOURCE_DIR}/include/groundTruthTransformer.h
    ${PROJECT_SOURCE_DIR}/include/rotationAveraging.h
    ${PROJECT_SOURCE_DIR}/include/quaternions.h
    ${PROJECT_SOURCE_DIR}/include/umeyama.h
    ${PROJECT_SOURCE_DIR}/include/poseEstimation.h
    ${PROJECT_SOURCE_DIR}/include/printer.h
    ${PROJECT_SOURCE_DIR}/include/errors.h
        include/relativePoseRefinement/ICP.h
    ${PROJECT_SOURCE_DIR}/include/pointCloud.h
    ${PROJECT_SOURCE_DIR}/include/translationMeasurement.h
    ${PROJECT_SOURCE_DIR}/include/translationAveraging.h
    ${PROJECT_SOURCE_DIR}/include/poseInfo.h
    ${PROJECT_SOURCE_DIR}/include/rotationMeasurement.h
    ${PROJECT_SOURCE_DIR}/include/relativePose.h
    ${PROJECT_SOURCE_DIR}/include/Vectors3d.h
        include/parametrization/Rotation3d.h
    ${PROJECT_SOURCE_DIR}/include/Point3d.h
    ${PROJECT_SOURCE_DIR}/include/RotationOptimizationRobust.h
    ${PROJECT_SOURCE_DIR}/include/sparsePointCloud/CloudProjector.h
    ${PROJECT_SOURCE_DIR}/include/KeyPointInfo.h
        include/sparsePointCloud/PointMatcher.h
    ${PROJECT_SOURCE_DIR}/include/ImageDrawer.h
    ${PROJECT_SOURCE_DIR}/include/AbsolutePose.h
        include/bundleAdjustment/BundleAdjuster.h
    ${PROJECT_SOURCE_DIR}/include/SmoothPointCloud.h
    ${PROJECT_SOURCE_DIR}/include/ThreadPool.h
    ${PROJECT_SOURCE_DIR}/include/KeyPointsDepthDescriptor.h
    ${PROJECT_SOURCE_DIR}/include/ConnectedComponent.h
        include/parametrization/RelativePoseSE3.h
    ${PROJECT_SOURCE_DIR}/include/ImagesAssociator.h
        include/optimization/lossFunctions/LossHuber.h
        ${PROJECT_SOURCE_DIR}/include/gnuplot_interface.h
        ${PROJECT_SOURCE_DIR}/include/relativePoseEstimators/Estimator3Points.h
        ${PROJECT_SOURCE_DIR}/include/relativePoseEstimators/InlierCounter.h
        ${PROJECT_SOURCE_DIR}/include/relativePoseEstimators/IEstimatorRelativePoseRobust.h
        ${PROJECT_SOURCE_DIR}/include/relativePoseEstimators/ParamsRANSAC.h
        ${PROJECT_SOURCE_DIR}/include/relativePoseRefinement/IRefinerRelativePose.h include/parametrization/MatchableInfo.h include/bundleAdjustment/IBundleAdjuster.h include/sparsePointCloud/IPointMatcher.h include/sparsePointCloud/ICloudProjector.h)

set(GDR_SOURCE_FILES
        src/filesystem/fileProc.cpp
    ${PROJECT_SOURCE_DIR}/src/VertexCG.cpp
    ${PROJECT_SOURCE_DIR}/src/CorrespondenceGraph.cpp
        src/parametrization/transformationRt.cpp
        src/parametrization/cameraRGBD.cpp
        src/keyPointDetectionAndMatching/SiftModuleGPU.cpp
    ${PROJECT_SOURCE_DIR}/src/groundTruthTransformer.cpp
    ${PROJECT_SOURCE_DIR}/src/rotationAveraging.cpp
    ${PROJECT_SOURCE_DIR}/src/quaternions.cpp
    ${PROJECT_SOURCE_DIR}/src/umeyama.cpp
    ${PROJECT_SOURCE_DIR}/src/poseEstimation.cpp
        src/relativePoseRefinement/ICP.cpp
    ${PROJECT_SOURCE_DIR}/src/pointCloud.cpp
    ${PROJECT_SOURCE_DIR}/src/translationAveraging.cpp
    ${PROJECT_SOURCE_DIR}/src/translationMeasurement.cpp
    ${PROJECT_SOURCE_DIR}/src/poseInfo.cpp
    ${PROJECT_SOURCE_DIR}/src/rotationMeasurement.cpp
    ${PROJECT_SOURCE_DIR}/src/relativePose.cpp
    ${PROJECT_SOURCE_DIR}/src/Vectors3d.cpp
    ${PROJECT_SOURCE_DIR}/src/ThreadPool.cpp
        src/parametrization/Rotation3d.cpp
    ${PROJECT_SOURCE_DIR}/src/RotationOptimizationRobust.cpp
    ${PROJECT_SOURCE_DIR}/src/Point3d.cpp
        src/sparsePointCloud/CloudProjector.cpp
    ${PROJECT_SOURCE_DIR}/src/KeyPointInfo.cpp
        src/sparsePointCloud/PointMatcher.cpp
    ${PROJECT_SOURCE_DIR}/src/ImageDrawer.cpp
    ${PROJECT_SOURCE_DIR}/src/AbsolutePose.cpp
        src/bundleAdjustment/BundleAdjuster.cpp
    ${PROJECT_SOURCE_DIR}/src/SmoothPointCloud.cpp
    ${PROJECT_SOURCE_DIR}/src/KeyPointsDepthDescriptor.cpp
    ${PROJECT_SOURCE_DIR}/src/ConnectedComponent.cpp
        src/parametrization/RelativePoseSE3.cpp
    ${PROJECT_SOURCE_DIR}/src/ImagesAssociator.cpp
        ${PROJECT_SOURCE_DIR}/src/KeyPoint2D.cpp
        ${PROJECT_SOURCE_DIR}/include/KeyPoint2D.h
        include/optimization/lossFunctions/LossFunctions.h
        include/keyPointDetectionAndMatching/ISiftModule.h
        ${PROJECT_SOURCE_DIR}/src/relativePoseEstimators/Estimator3Points.cpp
        ${PROJECT_SOURCE_DIR}/src/relativePoseEstimators/EstimatorNPoints.cpp
        ${PROJECT_SOURCE_DIR}/include/relativePoseEstimators/EstimatorNPoints.h
        ${PROJECT_SOURCE_DIR}/src/relativePoseEstimators/EstimatorRobustLoRANSAC.cpp
        ${PROJECT_SOURCE_DIR}/include/relativePoseEstimators/EstimatorRobustLoRANSAC.h
        ${PROJECT_SOURCE_DIR}/include/parametrization/SE3.h
        ${PROJECT_SOURCE_DIR}/src/parametrization/SE3.cpp
        ${PROJECT_SOURCE_DIR}/src/relativePoseEstimators/InlierCounter.cpp
        ${PROJECT_SOURCE_DIR}/src/relativePoseEstimators/ParamsRANSAC.cpp src/parametrization/MatchableInfo.cpp src/statistics/RobustEstimators.cpp include/statistics/RobustEstimators.h)

add_executable(readerVisionTumICP ${ICPCUDA_PROJECT_PATH}/ICP.cpp)

add_dependencies(readerVisionTumICP icpCuda)

target_link_libraries(readerVisionTumICP
    ${ICPCUDA_LIBS}
    ${Pangolin_LIBRARIES}
    ${CUDA_LIBRARIES})
target_include_directories(
    readerVisionTumICP
    PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
    ${ICPCUDA_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS})
add_subdirectory(lib)


add_library(GDR_LIB SHARED ${GDR_SOURCE_FILES} ${GDR_HEADER_FILES})

add_dependencies(GDR_LIB siftgpu icpCuda gtsam)

#add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY)
target_compile_definitions(GDR_LIB PUBLIC ${GDR_FLAGS} PRIVATE CERES_USE_CXX11_THREADS=1
    )

add_definitions(${PCL_DEFINITIONS})

target_link_libraries(
  GDR_LIB
  ${CUDA_LIBRARIES}
  siftgpu
  ${OpenCV_LIBS}
  ${GTSAM_LIBS}
  ${Boost_LIBRARIES}
  ${METIS_LIBRARIES}
  ${TBB_LIBRARIES}
  ${ICPCUDA_LIBS}
  ${Pangolin_LIBRARIES}
    ${CERES_LIBRARIES}
#    ${PCL_LIBRARY_DIRS}
    ${PCL_LIBRARIES}
#        Python3::Python
#        Python3::NumPy
#  ceres
)


target_include_directories(
  GDR_LIB
  PUBLIC ${GTSAM_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include
#        ${PROJECT_SOURCE_DIR}/siftgpu/src/SiftGPU
        ${EIGEN3_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        ${ICPCUDA_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
  PRIVATE ${OpenGL_INCLUDE_DIR})


enable_testing()
add_subdirectory(test)


message("CUDA home directories ${CUDA_BIN_PATH} " )
message("CUDA library directories ${CUDA_LIBRARIES} " )
message("CUDA incl directory ${CUDA_INCLUDE_DIRS} " )
