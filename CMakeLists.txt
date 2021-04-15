cmake_minimum_required(VERSION 3.10.0)

project(GDR)

set(CMAKE_CXX_STANDARD 17)
set(GCC_COVERAGE_COMPILE_FLAGS " -O3")

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
find_package(CUDA REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

find_package(
  Boost
  COMPONENTS program_options timer filesystem serialization graph
  REQUIRED)

set(local_custom_Pangolin_INSTALL_DIR _installed_Pangolin)
set(custom_Pangolin_BUILD_NAME g-build-concurrent)

set(ICPCUDA_PROJECT_PATH ${PROJECT_SOURCE_DIR}/third-party/ICPCUDA)
set(ICPCUDA_LIBS
    ${ICPCUDA_PROJECT_PATH}/${custom_Pangolin_BUILD_NAME}/libICP.so)
set(ICPCUDA_third_party ${ICPCUDA_PROJECT_PATH}/third-party)
set(ICPCUDA_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/third-party/ICPCUDA/include")

set(Pangolin_INSTALL_DIR
    "${PROJECT_SOURCE_DIR}/third-party/ICPCUDA/${custom_Pangolin_BUILD_NAME}/${local_custom_Pangolin_INSTALL_DIR}"
    CACHE PATH "Pangolin install directory")
set(Pangolin_INCLUDE_DIRS "${Pangolin_INSTALL_DIR}/include")
set(Pangolin_LIBRARIES ${Pangolin_INSTALL_DIR}/lib/libpangolin.so)

set(Sophus_INCLUDE_DIRS ${ICPCUDA_third_party}/Sophus)

set(GTSAM_BUILD_NAME mbuild-concurrent)

include(ExternalProject)

ExternalProject_Add(
  icpCuda
  SOURCE_DIR "${PROJECT_SOURCE_DIR}/third-party/ICPCUDA"
  BINARY_DIR
    "${PROJECT_SOURCE_DIR}/third-party/ICPCUDA/${custom_Pangolin_BUILD_NAME}"
  CMAKE_ARGS -DPangolin_LOCAL_INSTALL_DIR=${local_custom_Pangolin_INSTALL_DIR}
             -DPangolin_BUILD_NAME=${custom_Pangolin_BUILD_NAME}
  INSTALL_COMMAND "")

ExternalProject_Add(
  gtsam
  SOURCE_DIR "${PROJECT_SOURCE_DIR}/third-party/gtsam"
  BINARY_DIR "${PROJECT_SOURCE_DIR}/third-party/gtsam/${GTSAM_BUILD_NAME}"
  CONFIGURE_COMMAND
    cmake .. -DBUILD_SHARED_LIBS=0 -DGTSAM_USE_SYSTEM_EIGEN=1
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
    -DCMAKE_INSTALL_PREFIX:PATH=${PROJECT_SOURCE_DIR}/third-party/gtsam/${GTSAM_BUILD_NAME}
  INSTALL_COMMAND make install)

if(${DEBUG_PRINT_PROGRESS})
  list(APPEND GDR_FLAGS "-DDEBUG_PRINT_PROGRESS")
endif()

# TODO: how to include gtsam without hardcoding library name?
set(GTSAM_LIBS
    ${PROJECT_SOURCE_DIR}/third-party/gtsam/${GTSAM_BUILD_NAME}/lib/libgtsam.a)
set(GTSAM_INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/third-party/gtsam/${GTSAM_BUILD_NAME}/include)

set(GDR_HEADER_FILES
    ${PROJECT_SOURCE_DIR}/include/directoryTraversing/DirectoryReader.h
    ${PROJECT_SOURCE_DIR}/include/poseGraph/VertexCG.h
    ${PROJECT_SOURCE_DIR}/include/poseGraph/CorrespondenceGraph.h
    ${PROJECT_SOURCE_DIR}/include/parametrization/RelativeSE3.h
        include/cameraModel/CameraRGBD.h
    ${PROJECT_SOURCE_DIR}/include/keyPointDetectionAndMatching/SiftModuleGPU.h
    ${PROJECT_SOURCE_DIR}/include/absolutePoseEstimation/rotationAveraging/RotationAverager.h
    ${PROJECT_SOURCE_DIR}/include/relativePoseRefinement/ICPCUDA.h
    ${PROJECT_SOURCE_DIR}/include/absolutePoseEstimation/translationAveraging/TranslationMeasurement.h
    ${PROJECT_SOURCE_DIR}/include/absolutePoseEstimation/translationAveraging/TranslationAverager.h
        include/readerDataset/readerTUM/PoseFullInfo.h
    ${PROJECT_SOURCE_DIR}/include/absolutePoseEstimation/rotationAveraging/RotationMeasurement.h
    ${PROJECT_SOURCE_DIR}/include/parametrization/Vectors3d.h
    ${PROJECT_SOURCE_DIR}/include/parametrization/SO3.h
    ${PROJECT_SOURCE_DIR}/include/parametrization/Point3d.h
    ${PROJECT_SOURCE_DIR}/include/absolutePoseEstimation/rotationAveraging/RotationRobustOptimizer.h
    ${PROJECT_SOURCE_DIR}/include/sparsePointCloud/CloudProjector.h
    ${PROJECT_SOURCE_DIR}/include/keyPoints/KeyPointInfo.h
    ${PROJECT_SOURCE_DIR}/include/sparsePointCloud/PointClassifier.h
    ${PROJECT_SOURCE_DIR}/include/visualization/2D/ImageDrawer.h
    ${PROJECT_SOURCE_DIR}/include/bundleAdjustment/BundleAdjuster.h
    ${PROJECT_SOURCE_DIR}/include/visualization/3D/SmoothPointCloud.h
    ${PROJECT_SOURCE_DIR}/include/computationHandlers/ThreadPool.h
    ${PROJECT_SOURCE_DIR}/include/keyPoints/KeyPointsDepthDescriptor.h
    ${PROJECT_SOURCE_DIR}/include/poseGraph/ConnectedComponent.h
        include/readerDataset/readerTUM/ImagesAssociator.h
    ${PROJECT_SOURCE_DIR}/include/relativePoseEstimators/Estimator3Points.h
    ${PROJECT_SOURCE_DIR}/include/relativePoseEstimators/InlierCounter.h
    ${PROJECT_SOURCE_DIR}/include/relativePoseEstimators/IEstimatorRelativePoseRobust.h
    ${PROJECT_SOURCE_DIR}/include/relativePoseEstimators/ParamsRANSAC.h
    ${PROJECT_SOURCE_DIR}/include/relativePoseRefinement/IRefinerRelativePose.h
    ${PROJECT_SOURCE_DIR}/include/parametrization/MatchableInfo.h
    ${PROJECT_SOURCE_DIR}/include/bundleAdjustment/IBundleAdjuster.h
    ${PROJECT_SOURCE_DIR}/include/sparsePointCloud/IPointClassifier.h
    ${PROJECT_SOURCE_DIR}/include/sparsePointCloud/ICloudProjector.h
    ${PROJECT_SOURCE_DIR}/include/keyPoints/KeyPoint2DAndDepth.h
    ${PROJECT_SOURCE_DIR}/include/optimization/lossFunctions/LossFunctions.h
    ${PROJECT_SOURCE_DIR}/include/keyPointDetectionAndMatching/IFeatureDetectorMatcher.h
    ${PROJECT_SOURCE_DIR}/include/relativePoseEstimators/EstimatorNPoints.h
    ${PROJECT_SOURCE_DIR}/include/relativePoseEstimators/EstimatorRobustLoRANSAC.h
    ${PROJECT_SOURCE_DIR}/include/parametrization/SE3.h
    ${PROJECT_SOURCE_DIR}/include/statistics/RobustEstimators.h
        include/readerDataset/readerTUM/ReaderTum.h
    ${PROJECT_SOURCE_DIR}/include/keyPointDetectionAndMatching/FeatureDetectorMatcherCreator.h
    ${PROJECT_SOURCE_DIR}/include/keyPointDetectionAndMatching/KeyPointsAndDescriptors.h
    ${PROJECT_SOURCE_DIR}/include/computationHandlers/RelativePosesComputationHandler.h
    ${PROJECT_SOURCE_DIR}/include/poseGraph/graphAlgorithms/GraphTraverser.h
    ${PROJECT_SOURCE_DIR}/include/computationHandlers/AbsolutePosesComputationHandler.h
    ${PROJECT_SOURCE_DIR}/include/keyPointDetectionAndMatching/Match.h
    ${PROJECT_SOURCE_DIR}/include/sparsePointCloud/ProjectableInfo.h
    ${PROJECT_SOURCE_DIR}/include/absolutePoseEstimation/rotationAveraging/IRotationRobustOptimizer.h
    ${PROJECT_SOURCE_DIR}/include/relativePoseEstimators/EstimatorRelativePoseRobustCreator.h
    ${PROJECT_SOURCE_DIR}/include/relativePoseRefinement/RefinerRelativePoseCreator.h
    ${PROJECT_SOURCE_DIR}/include/bundleAdjustment/BundleAdjusterCreator.h
    ${PROJECT_SOURCE_DIR}/include/sparsePointCloud/CloudProjectorCreator.h
    ${PROJECT_SOURCE_DIR}/include/sparsePointCloud/PointClassifierCreator.h
    ${PROJECT_SOURCE_DIR}/include/absolutePoseEstimation/rotationAveraging/RotationRobustOptimizerCreator.h
    ${PROJECT_SOURCE_DIR}/include/parametrization/SparseMatrixClass.h
    ${PROJECT_SOURCE_DIR}/include/absolutePoseEstimation/rotationAveraging/RelativeRotationError.h
)

set(GDR_SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/src/directoryTraversing/DirectoryReader.cpp
    ${PROJECT_SOURCE_DIR}/src/poseGraph/VertexCG.cpp
    ${PROJECT_SOURCE_DIR}/src/poseGraph/CorrespondenceGraph.cpp
    ${PROJECT_SOURCE_DIR}/src/parametrization/RelativeSE3.cpp
        src/cameraModel/CameraRGBD.cpp
    ${PROJECT_SOURCE_DIR}/src/keyPointDetectionAndMatching/SiftModuleGPU.cpp
    ${PROJECT_SOURCE_DIR}/src/absolutePoseEstimation/rotationAveraging/RotationAverager.cpp
    ${PROJECT_SOURCE_DIR}/src/relativePoseRefinement/ICPCUDA.cpp
    ${PROJECT_SOURCE_DIR}/src/absolutePoseEstimation/translationAveraging/TranslationAverager.cpp
    ${PROJECT_SOURCE_DIR}/src/absolutePoseEstimation/translationAveraging/TranslationMeasurement.cpp
        src/readerDataset/readerTUM/PoseFullInfo.cpp
    ${PROJECT_SOURCE_DIR}/src/absolutePoseEstimation/rotationAveraging/RotationMeasurement.cpp
    ${PROJECT_SOURCE_DIR}/src/parametrization/Vectors3d.cpp
    ${PROJECT_SOURCE_DIR}/src/computationHandlers/ThreadPool.cpp
    ${PROJECT_SOURCE_DIR}/src/parametrization/SO3.cpp
    ${PROJECT_SOURCE_DIR}/src/absolutePoseEstimation/rotationAveraging/RotationRobustOptimizer.cpp
    ${PROJECT_SOURCE_DIR}/src/parametrization/Point3d.cpp
    ${PROJECT_SOURCE_DIR}/src/sparsePointCloud/CloudProjector.cpp
    ${PROJECT_SOURCE_DIR}/src/keyPoints/KeyPointInfo.cpp
    ${PROJECT_SOURCE_DIR}/src/sparsePointCloud/PointClassifier.cpp
    ${PROJECT_SOURCE_DIR}/src/visualization/2D/ImageDrawer.cpp
    ${PROJECT_SOURCE_DIR}/src/bundleAdjustment/BundleAdjuster.cpp
    ${PROJECT_SOURCE_DIR}/src/visualization/3D/SmoothPointCloud.cpp
    ${PROJECT_SOURCE_DIR}/src/keyPoints/KeyPointsDepthDescriptor.cpp
    ${PROJECT_SOURCE_DIR}/src/poseGraph/ConnectedComponent.cpp
        src/readerDataset/readerTUM/ImagesAssociator.cpp
    ${PROJECT_SOURCE_DIR}/src/keyPoints/KeyPoint2DAndDepth.cpp
    ${PROJECT_SOURCE_DIR}/src/relativePoseEstimators/Estimator3Points.cpp
    ${PROJECT_SOURCE_DIR}/src/relativePoseEstimators/EstimatorNPoints.cpp
    ${PROJECT_SOURCE_DIR}/src/relativePoseEstimators/EstimatorRobustLoRANSAC.cpp
    ${PROJECT_SOURCE_DIR}/src/parametrization/SE3.cpp
    ${PROJECT_SOURCE_DIR}/src/relativePoseEstimators/InlierCounter.cpp
    ${PROJECT_SOURCE_DIR}/src/relativePoseEstimators/ParamsRANSAC.cpp
    ${PROJECT_SOURCE_DIR}/src/parametrization/MatchableInfo.cpp
    ${PROJECT_SOURCE_DIR}/src/statistics/RobustEstimators.cpp
        src/readerDataset/readerTUM/ReaderTum.cpp
    ${PROJECT_SOURCE_DIR}/src/keyPointDetectionAndMatching/FeatureDetectorMatcherCreator.cpp
    ${PROJECT_SOURCE_DIR}/src/keyPointDetectionAndMatching/KeyPointsAndDescriptors.cpp
    ${PROJECT_SOURCE_DIR}/src/keyPointDetectionAndMatching/Match.cpp
    ${PROJECT_SOURCE_DIR}/src/sparsePointCloud/ProjectableInfo.cpp
    ${PROJECT_SOURCE_DIR}/src/computationHandlers/RelativePosesComputationHandler.cpp
    ${PROJECT_SOURCE_DIR}/src/poseGraph/graphAlgorithms/GraphTraverser.cpp
    ${PROJECT_SOURCE_DIR}/src/computationHandlers/AbsolutePosesComputationHandler.cpp
    ${PROJECT_SOURCE_DIR}/src/bundleAdjustment/BundleAdjusterCreator.cpp
    ${PROJECT_SOURCE_DIR}/src/relativePoseEstimators/EstimatorRelativePoseRobustCreator.cpp
    ${PROJECT_SOURCE_DIR}/src/relativePoseRefinement/RefinerRelativePoseCreator.cpp
    ${PROJECT_SOURCE_DIR}/src/absolutePoseEstimation/rotationAveraging/RotationRobustOptimizerCreator.cpp
    ${PROJECT_SOURCE_DIR}/src/sparsePointCloud/CloudProjectorCreator.cpp
    ${PROJECT_SOURCE_DIR}/src/sparsePointCloud/PointClassifierCreator.cpp
    ${PROJECT_SOURCE_DIR}/src/parametrization/SparseMatrixClass.cpp
    ${PROJECT_SOURCE_DIR}/src/absolutePoseEstimation/rotationAveraging/RelativeRotationError.cpp
        include/readerDataset/readerBundleFusion/ReaderBundleFusion.cpp include/readerDataset/readerBundleFusion/ReaderBundleFusion.h src/readerDataset/readerTUM/ClosestMatchFinder.cpp include/readerDataset/readerTUM/ClosestMatchFinder.h src/keyPoints/KeyPointMatches.cpp include/keyPoints/KeyPointMatches.h src/cameraModel/MeasurementErrorDeviationEstimators.cpp include/cameraModel/MeasurementErrorDeviationEstimators.h)

add_subdirectory(lib)

add_library(GDR_LIB SHARED ${GDR_SOURCE_FILES} ${GDR_HEADER_FILES})

add_dependencies(GDR_LIB siftgpu icpCuda gtsam)

target_compile_definitions(
  GDR_LIB
  PUBLIC ${GDR_FLAGS}
  PRIVATE CERES_USE_CXX11_THREADS=1)

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
  ${PCL_LIBRARIES})

target_include_directories(
  GDR_LIB
  PUBLIC ${PROJECT_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIRS}
         ${Sophus_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS}
  PRIVATE ${OpenGL_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${ICPCUDA_INCLUDE_DIRS}
          ${Pangolin_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIRS})

enable_testing()
add_subdirectory(test)
