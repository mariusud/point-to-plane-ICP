import symforce

# symforce.set_epsilon_to_symbol()

import numpy as np
import sym
from symforce import typing as T
from symforce.values import Values
from symforce.opt.factor import Factor
from symforce.opt.optimizer import Optimizer
from symforce.opt.factor import visualize_factors
import open3d as o3d
import symforce.symbolic as sf
from symforce import logger
import open3d as o3d
import time

if symforce.get_symbolic_api() != "symengine":
    logger.warning("The 3D Localization example is very slow on sympy. Use symengine.")


def generate_points_on_cube_surface(num_points_per_face):
    # Generate points on the surface of a cube
    points = []
    normals = []
    centroids = []

    # Define the vertices of a unit cube
    vertices = np.array(
        [
            [-0.5, -0.5, -0.5],
            [-0.5, -0.5, 0.5],
            [-0.5, 0.5, -0.5],
            [-0.5, 0.5, 0.5],
            [0.5, -0.5, -0.5],
            [0.5, -0.5, 0.5],
            [0.5, 0.5, -0.5],
            [0.5, 0.5, 0.5],
        ],
        dtype=np.float64,
    )

    faces = [
        [0, 1, 2, 3],
        [4, 6, 7, 5],
        [0, 4, 5, 1],
        [2, 3, 7, 6],
        [0, 2, 6, 4],
        [1, 7, 5, 3],
    ]

    # faces = [[0, 1, 2, 3]]

    for face in faces:
        # Get the vertices of the face
        face_vertices = vertices[face]
        # Calculate the centroid of the face
        centroid = np.mean(face_vertices, axis=0)

        normal = np.cross(
            face_vertices[1] - face_vertices[0], face_vertices[2] - face_vertices[0]
        )
        normal /= np.linalg.norm(normal)

        for _ in range(num_points_per_face):
            # Randomly choose a point within the face by interpolating between vertices
            u, v = np.random.rand(2)
            point = (
                (1 - u) * (1 - v) * face_vertices[0]
                + u * (1 - v) * face_vertices[1]
                + u * v * face_vertices[2]
                + (1 - u) * v * face_vertices[3]
            )
            points.append(point)
            centroids.append(centroid)
            normals.append(normal)

    return np.array(points, dtype=np.float64), np.array(centroids), np.array(normals)


def build_cube_values(num_points_per_face) -> T.Tuple[Values, int]:
    np.random.seed(42)

    values = Values()

    points, centroids, normals = generate_points_on_cube_surface(num_points_per_face)

    values["points"] = np.array(points)
    values["centroids"] = np.array(centroids)
    values["normals"] = np.array(normals)

    print(values["points"].shape)

    # values["points"] = [np.array(point) for point in points.tolist()]
    # values["centroids"] = [np.array(centroid) for centroid in centroids.tolist()]
    # values["normals"] = [np.array(normal) for normal in normals.tolist()]

    t = 40
    tangent_vec = np.array(
        [
            -1 * t,
            -2 * t,
            -3 * t,
            8 * np.sin(t * np.pi / 1.3),
            9 * np.sin(t * np.pi / 2),
            5 * np.sin(t * np.pi / 1.8),
        ]
    )

    values["world_T_lidar"] = sym.Pose3.from_tangent(
        tangent_vec, epsilon=sf.numeric_epsilon
    )

    values["epsilon"] = sf.numeric_epsilon

    return values
