Global Depth Reconstructor
===============================================
![Image of Yaktocat](https://drive.google.com/uc?export=view&id=1GC3GDO4HylncCDziv9ANbkMy_F1a4iMH)

You are free to use this code in non-commercial applications (Please take a look at [LICENSE](LICENSE)).

This project is an implementation of global Structure-From-Motion pipeline for RGB-D reconstruction.


About
--------
This library provides you a GPU-based solution for offline reconstruction using RGB-D cameras like Kinect. CUDA is intensively used for Keypoint detection, matching and ICP, so make shure you have capatible device installed.

## Dependencies

Make shure you have these dependencies preinstalled
```bash
sudo apt install nvidia-cuda-dev nvidia-cuda-toolkit
sudo apt install libglu1-mesa-dev freeglut3-dev mesa-common-dev
sudo apt install libglew-dev
sudo apt install libpcl-dev
sudo apt install libboost-all-dev
sudo apt install libceres-dev
sudo apt install libtbb-dev
sudo apt install libeigen3-dev
sudo apt install libopencv-dev
sudo apt install libmetis-dev
```


## Project

```bash
mkdir build
cd build
cmake ..
make
cd test && ctest
```

## Usage

[TUM](https://vision.in.tum.de/data/datasets/rgbd-dataset/download)-formatted datasets can be easily processed, for trajectory evaluation see 

```bash
./reconstructorTUM
```

For dense model creation see
```bash
./visualizerTUM
```

