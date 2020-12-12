cmake_minimum_required(VERSION 2.8.3)
project(GlobalReconstruction)

set(CMAKE_CXX_STANDARD 17)
set(GCC_COVERAGE_COMPILE_FLAGS " -Os -DNDEBUG")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")


find_package(GTSAM REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PCL 1.8 REQUIRED)

set(ICPCUDA "externals/ICPCUDA")

include_directories(${Sophus_INCLUDE_DIR} ${ICPCUDA})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${OpenGL_INCLUDE_DIR})
include_directories("siftgpu/src/SiftGPU")
include_directories(${CUDA_INCLUDE_DIR})
set(SIFTGPU_LIBS "${CMAKE_CURRENT_SOURCE_DIR}/siftgpu/bin/libsiftgpu.so")



FIND_PACKAGE(Boost COMPONENTS thread system filesystem REQUIRED)
MESSAGE("   Boost information:")
MESSAGE("   Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
MESSAGE("   Boost_LIBRARIES: ${Boost_LIBRARIES}")
MESSAGE("   Boost_LIBRARY_DIRS: ${Boost_LIBRARY_DIRS}")

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})


LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})
add_executable(test test/main.cpp src/files.cpp src/features.cpp include/features.h src/VertexCG.cpp include/VertexCG.h src/CorrespondenceGraph.cpp include/CorrespondenceGraph.h src/transformationRt.cpp include/transformationRt.h src/cameraRGBD.cpp include/cameraRGBD.h src/siftModule.cpp include/siftModule.h src/groundTruthTransformer.cpp include/groundTruthTransformer.h test/testEigen.cpp src/images.cpp include/images.h src/rotationAveraging.cpp include/rotationAveraging.h src/util.cpp include/util.h src/quaternions.cpp include/quaternions.h src/ICP.cpp include/ICP.h)
#add_executable(trySIFT test/trysift.cpp src/files.cpp src/features.cpp include/features.h src/VertexCG.cpp include/VertexCG.h src/CorrespondenceGraph.cpp include/CorrespondenceGraph.h)
#add_executable(transformGroundTruth test/transformGroundTruth.cpp src/groundTruthTransformer.cpp src/files.cpp)
#add_executable(testEigen test/testEigen.cpp)

#target_link_libraries(transformGroundTruth ${Boost_LIBRARIES})
target_link_libraries(test
        ${TBB_LIBRARIES}
        ${OpenCV_LIBS}
        ${SIFTGPU_LIBS}
        ${GLEW_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES}
        ${CUDA_LIB_PATH}
        ${PCL_LIBRARIES}
        Eigen3::Eigen
        pthread
        gtsam
        )

#target_link_libraries(trySIFT
#        ${OpenCV_LIBS}
#        ${SIFTGPU_LIBS}
#        ${GLEW_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES}
#        ${CUDA_LIB_PATH}
#        )
