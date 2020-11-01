cmake_minimum_required(VERSION 2.8.3)
project(test_siftgpu)
set(CMAKE_CXX_STANDARD 17)

# OpenCV
find_package(OpenCV REQUIRED)

# OpenGL
find_package(OpenGL REQUIRED)

# GLUT
find_package(GLUT REQUIRED)

# Glew
find_package(GLEW REQUIRED)

# SiftGPU
include_directories("../siftgpu/src/SiftGPU/" ${OpenGL_INCLUDE_DIR} ${CUDA_INCLUDE_DIR})
set(SIFTGPU_LIBS "${CMAKE_CURRENT_SOURCE_DIR}/../siftgpu/bin/libsiftgpu.so")


find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(testSIFTGPU main.cpp ../src/files.cpp ../src/features.cpp ../include/features.h ../src/vertexCG.cpp ../include/vertexCG.h ../src/CG.cpp ../include/CG.h ../src/essentialMatrix.cpp ../include/essentialMatrix.h ../src/cameraRGBD.cpp ../include/cameraRGBD.h ../src/siftModule.cpp ../include/siftModule.h)
add_executable(trySIFT trysift.cpp ../src/files.cpp ../src/features.cpp ../include/features.h ../src/vertexCG.cpp ../include/vertexCG.h ../src/CG.cpp ../include/CG.h)
target_link_libraries(testSIFTGPU
        ${OpenCV_LIBS}
        ${SIFTGPU_LIBS}
        ${GLEW_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES}
        ${CUDA_LIB_PATH}
        Eigen3::Eigen
        )
target_link_libraries(trySIFT
        ${OpenCV_LIBS}
        ${SIFTGPU_LIBS}
        ${GLEW_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES}
        ${CUDA_LIB_PATH}
        )
