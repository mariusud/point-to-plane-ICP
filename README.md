# point-to-plane-ICP

Implementation of point-to-plane Iterative Closest Point (ICP) using symforce.

![Alt Text](assets/animation.gif)

## Problem Description

This project implements the point-to-plane ICP algorithm using the symforce library. The ICP algorithm is commonly used for aligning two point clouds by minimizing the distance between corresponding points and their corresponding point, line or plane correspodences.

### Residual

The residual function computes the difference between points and their estimated plane correspondences in the point cloud using the centroid and normal of the plane. As long as the centroid/point correspondence is on the plane, it does not actually matter which point you choose.

## Install

To install the required dependencies, use the following command:

```bash
pip install -r requirements.txt
```

then you can simply run with

```bash
python3 generate_fixed.py
```
