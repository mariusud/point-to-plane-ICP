# -----------------------------------------------------------------------------
# Set the default epsilon to a symbol
# -----------------------------------------------------------------------------
import symforce

symforce.set_epsilon_to_symbol()

# -----------------------------------------------------------------------------
# Create initial Values
# -----------------------------------------------------------------------------
import numpy as np
import sym
from symforce import typing as T
from symforce.values import Values
from symforce.opt.factor import Factor
from symforce.opt.optimizer import Optimizer

from symforce.opt.factor import visualize_factors
from symforce.opt.noise_models import IsotropicNoiseModel
import open3d as o3d

import symforce.symbolic as sf
from symforce import logger
import open3d as o3d

if symforce.get_symbolic_api() != "symengine":
    logger.warning("The 3D Localization example is very slow on sympy. Use symengine.")


def visualize_cube_and_points(points, normals, centroids):
    # Create Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # Create Open3D line set for the cube
    cube_lines = [
        [0, 1],
        [1, 2],
        [2, 3],
        [3, 0],
        [4, 5],
        [5, 6],
        [6, 7],
        [7, 4],
        [0, 4],
        [1, 5],
        [2, 6],
        [3, 7],
    ]
    cube_vertices = [
        [-0.5, -0.5, -0.5],
        [-0.5, -0.5, 0.5],
        [-0.5, 0.5, -0.5],
        [-0.5, 0.5, 0.5],
        [0.5, -0.5, -0.5],
        [0.5, -0.5, 0.5],
        [0.5, 0.5, -0.5],
        [0.5, 0.5, 0.5],
    ]
    cube_lines_set = []
    for vertices in cube_lines:
        cube_lines_set.append([cube_vertices[vertices[0]], cube_vertices[vertices[1]]])
    cube_line_set = o3d.geometry.LineSet()
    cube_line_set.points = o3d.utility.Vector3dVector(cube_vertices)
    cube_line_set.lines = o3d.utility.Vector2iVector(cube_lines)

    # Create Open3D lines for the normals
    normal_lines = []
    for i in range(len(centroids)):
        normal_lines.append([centroids[i], centroids[i] + normals[i]])

    normal_line_set = o3d.geometry.LineSet()
    normal_line_set.points = o3d.utility.Vector3dVector(
        np.concatenate(normal_lines, axis=0)
    )
    lines = []
    for i in range(len(normal_lines)):
        lines.append([2 * i, 2 * i + 1])
    normal_line_set.lines = o3d.utility.Vector2iVector(lines)

    # Visualize
    o3d.visualization.draw_geometries([pcd, cube_line_set, normal_line_set])


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

    # Define the faces of the cube
    faces = [
        [0, 1, 3, 2],  # Front face
        [4, 5, 7, 6],  # Back face
        [0, 1, 5, 4],  # Left face
        [2, 3, 7, 6],  # Right face
        [0, 2, 6, 4],  # Bottom face
        [1, 3, 7, 5],  # Top face
    ]

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

    values["points"] = points
    values["centroids"] = centroids
    values["normals"] = normals

    values["points"] = [np.array(point) for point in points.tolist()]
    values["centroids"] = [np.array(centroid) for centroid in centroids.tolist()]
    values["normals"] = [np.array(normal) for normal in normals.tolist()]

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


def point_to_plane_residual(
    world_T_lidar: sf.Pose3, point: sf.V3, centroid: sf.V3, normal: sf.V3
) -> sf.V3:

    estimated_position = world_T_lidar * point
    v = estimated_position - centroid

    d_pred = normal.T * v

    return d_pred


def build_factors(num_correspondences: int) -> T.Iterator[Factor]:
    for i in range(num_correspondences):
        yield Factor(
            residual=point_to_plane_residual,
            keys=[
                f"world_T_lidar",
                f"points[{i}]",
                f"centroids[{i}]",
                f"normals[{i}]",
            ],
        )


def build_residual(num_correspondences, values: Values) -> sf.Matrix:
    residuals: T.List[sf.Matrix] = []
    for i in range(num_correspondences):
        residuals.append(
            point_to_plane_residual(
                values.attr.world_T_lidar,
                values.attr.points[i],
                values.attr.centroids[i],
                values.attr.normals[i],
            )
        )
    return sf.Matrix.block_matrix([[residual] for residual in residuals])


def main() -> None:
    num_poses = 1
    num_points_per_face = 100

    values = build_cube_values(num_points_per_face)
    NUM_FACES = 6

    # for key, value in values.items_recursive():
    # print(f"{key}:  {value}")

    factors = build_factors(num_points_per_face * NUM_FACES)

    num_factors_to_visualize = 1
    selected_factors = []

    for i, factor in enumerate(factors):
        selected_factors.append(factor)
        if i + 1 == num_factors_to_visualize:
            break

    # dot_graph = visualize_factors(selected_factors)
    # dot_graph.view()
    # dot_graph.render("factor_graph", format="png", cleanup=True)

    optimized_keys = [f"world_T_lidar"]

    optimizer = Optimizer(
        factors=factors,
        optimized_keys=optimized_keys,
        # Return problem stats for every iteration
        debug_stats=True,
        # Customize optimizer behavior
        params=Optimizer.Params(
            verbose=True, initial_lambda=1e4, lambda_down_factor=1 / 2.0
        ),
    )

    # Solve and return the result
    result = optimizer.optimize(values)

    values_per_iter = [
        optimizer.load_iteration_values(stats.values) for stats in result.iterations
    ]

    for i, vals in enumerate(values_per_iter):
        world_T_lidar = vals["world_T_lidar"]

        print(
            f"Iteration {i}: t = {world_T_lidar.position()}, R = {world_T_lidar.rotation().to_tangent()}"
        )

        # Rotate the points
        # rotated_points = [
        # [world_T_lidar.rotation() * sf.V3(*point) for point in values["points"]]
        # ]

        # Visualize the rotated points

        points = [world_T_lidar * point for point in vals["points"]]
        if i % 5 == 0:
            visualize_cube_and_points(
                points,
                vals["normals"],
                vals["centroids"],
            )

    # Print some values
    print(f"Num iterations: {len(result.iterations) - 1}")
    print(f"Final error: {result.error():.6f}")
    print(f"Status: {result.status}")


main()
