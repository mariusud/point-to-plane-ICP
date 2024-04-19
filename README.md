# Point-to-plane ICP

Implementation of point-to-plane Iterative Closest Point (ICP) using symforce.

## Problem Description

This project implements the point-to-plane ICP algorithm using the symforce library. The ICP algorithm is commonly used for aligning two point clouds by minimizing the distance between corresponding points and their corresponding point, line or plane correspodences.

### Residual

![Alt Text](assets/factor_graph.png)

The residual function computes the difference between points and their estimated plane correspondences in the point cloud using the centroid and normal of the plane. As long as the centroid/point correspondence is on the plane, it does not actually matter which point you choose.

## Install

This repo has code for visualizing and running point-to-plane ICP using symforce in both Python and C++

### Run Python

To visualize the point-to-plane ICP optimization on a simple cube in python, you can run:

```bash
pip install -r requirements.txt
python3 visualize.py
```

This will also visualize the optimization using Open3D
![Alt Text](assets/animation.gif)

### Run C++

To run C++ code, you need to compile the project

```bash
mkdir build
cd build
cmake ..
make
./run_dynamic_size
```

### Installing Symforce

To install symforce, follow the official [installation guide](https://github.com/symforce-org/symforce?tab=readme-ov-file#build-with-cmake)
and remember to run the following to use symforce in another CMake project

```bash
make install
```

The CMake files will automatically find symforce, Eigen, FMT and spdlog for you, and are taken from https://github.com/gcross-zipline/find_symforce_example
