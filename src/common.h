#pragma once
#include <Eigen/Core>

#include <sym/pose3.h>
#include <symforce/opt/key.h>
#include <symforce/opt/optimizer.h>
#include <symforce/opt/values.h>

#include "../gen/keys.h"

template <typename Scalar>
inline sym::Values<Scalar> build_cube_values()
{
    sym::Values<Scalar> values;

    int kNumPointsPerFace = 10;

    // Define cube vertices
    std::vector<Eigen::Vector3d> cube_vertices = {
        {-0.5, -0.5, -0.5},
        {-0.5, -0.5, 0.5},
        {-0.5, 0.5, -0.5},
        {-0.5, 0.5, 0.5},
        {0.5, -0.5, -0.5},
        {0.5, -0.5, 0.5},
        {0.5, 0.5, -0.5},
        {0.5, 0.5, 0.5}};

    // Define cube faces
    std::vector<std::vector<int>> cube_faces = {
        {0, 1, 3, 2}, // Front face
        {4, 5, 7, 6}, // Back face
        {0, 1, 5, 4}, // Left face
        {2, 3, 7, 6}, // Right face
        {0, 2, 6, 4}, // Bottom face
        {1, 3, 7, 5}  // Top face
    };

    // Generate points on the surface of the cube
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> normals;
    std::vector<Eigen::Vector3d> centroids;
    for (const auto &face : cube_faces)
    {
        const auto &v0 = cube_vertices[face[0]];
        const auto &v1 = cube_vertices[face[1]];
        const auto &v2 = cube_vertices[face[2]];
        const auto &v3 = cube_vertices[face[3]];
        Eigen::Vector3d normal = (v1 - v0).cross(v3 - v0).normalized();

        // Calculate the centroid of the face
        Eigen::Vector3d centroid = (v0 + v1 + v2 + v3) / 4.0;

        for (int i = 0; i < kNumPointsPerFace; ++i)
        {
            double u = static_cast<double>(std::rand()) / RAND_MAX;
            double v = static_cast<double>(std::rand()) / RAND_MAX;

            Eigen::Vector3d point = (1 - u) * (1 - v) * v0 + u * (1 - v) * v1 + u * v * v2 + (1 - u) * v * v3;
            normals.push_back(normal);
            points.push_back(point);
            centroids.push_back(centroid);
        }
    }

    for (int i = 0; i < points.size(); ++i)
    {
        // Set points in the sym::Values object|
        values.Set(sym::Keys::POINTS.WithSuper(i), points[i]);
        values.Set(sym::Keys::NORMALS.WithSuper(i), normals[i]);
        values.Set(sym::Keys::CENTROIDS.WithSuper(i), centroids[i]);
    }

    // Set initial pose
    sym::Pose3d initial_pose;
    values.Set(sym::Keys::WORLD_T_LIDAR, sym::Pose3<Scalar>(initial_pose));

    // Set epsilon
    values.Set(sym::Keys::EPSILON, sym::kDefaultEpsilon<Scalar>);

    return values;
}
