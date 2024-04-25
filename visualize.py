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


def visualize_factor_graph(factors, num_factors_to_visualize):
    selected_factors = []
    for i, factor in enumerate(factors):
        selected_factors.append(factor)
        if i + 1 == num_factors_to_visualize:
            break

    dot_graph = visualize_factors(selected_factors)
    dot_graph.view()
    dot_graph.render("factor_graph", format="png", cleanup=True)


def visualize(values, values_per_iter, save_image=False):
    vis = o3d.visualization.Visualizer()
    vis.create_window("Point-to-plane ICP", width=1080, height=1080)
    for i, vals in enumerate(values_per_iter):
        if i == 0:
            cube_line_set, normal_line_set = create_cube(
                values["centroids"], values["normals"]
            )

            geometry = o3d.geometry.PointCloud()
            init_points = [
                values["world_T_lidar"] * point for point in values["points"]
            ]
            geometry.points = o3d.utility.Vector3dVector(init_points)
            vis.add_geometry(geometry)
            vis.add_geometry(cube_line_set)

        world_T_lidar = vals["world_T_lidar"]
        print(
            f"Iteration {i}: t = {world_T_lidar.position()}, R = {world_T_lidar.rotation().to_tangent()}"
        )
        if save_image:
            vis.capture_screen_image("temp_%04d.jpg" % i)

        points = [world_T_lidar * point for point in vals["points"]]
        geometry.points = o3d.utility.Vector3dVector(points)
        vis.update_geometry(geometry)
        vis.poll_events()
        vis.update_renderer()

        vis.poll_events()
        vis.update_renderer()
        time.sleep(0.2)
    vis.destroy_window()


def create_cube(centroids, normals):
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

    # Create Open3D line set for the cube
    cube_line_set = o3d.geometry.LineSet()
    cube_line_set.points = o3d.utility.Vector3dVector(cube_vertices)
    cube_line_set.lines = o3d.utility.Vector2iVector(cube_lines)

    # Create Open3D lines for the normals
    normal_lines = []
    for centroid, normal in zip(centroids, normals):
        normal_lines.append([centroid, centroid + normal])
    normal_line_set = o3d.geometry.LineSet()
    normal_line_set.points = o3d.utility.Vector3dVector(
        np.concatenate(normal_lines, axis=0)
    )
    lines = [[2 * i, 2 * i + 1] for i in range(len(normal_lines))]
    normal_line_set.lines = o3d.utility.Vector2iVector(lines)

    return cube_line_set, normal_line_set


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

    # faces = [
    # [0, 1, 2, 3],
    # [4, 6, 7, 5],
    # [0, 4, 5, 1],
    # [2, 3, 7, 6],
    # [0, 2, 6, 4],
    # [1, 7, 5, 3],
    # ]

    faces = [[0, 1, 2, 3]]

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
    world_T_lidar: sf.Pose3,
    point_lidar: sf.V3,
    centroid_world: sf.V3,
    normal_world: sf.V3,
) -> sf.V3:

    estimated_position = world_T_lidar * point_lidar
    v = estimated_position - centroid_world

    d_pred = normal_world.T * v

    return d_pred  # .squared_norm()


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


def main() -> None:
    num_poses = 1
    num_points_per_face = 100

    values = build_cube_values(num_points_per_face)
    NUM_FACES = 6

    # for key, value in values.items_recursive():
    # print(f"{key}:  {value}")

    factors = build_factors(num_points_per_face * NUM_FACES)

    for factor in factors:
        print("factor: ", factor)

    # visualize_factor_graph(factors, num_factors_to_visualize=1)

    optimized_keys = [f"world_T_lidar"]

    optimizer = Optimizer(
        factors=factors,
        optimized_keys=optimized_keys,
        params=Optimizer.Params(
            verbose=True,
            initial_lambda=1e4,
            lambda_down_factor=1 / 2.0,
            debug_stats=True,
        ),
    )

    result = optimizer.optimize(values)

    values_per_iter = [
        optimizer.load_iteration_values(stats.values) for stats in result.iterations
    ]
    visualize(values, values_per_iter)

    print(f"Num iterations: {len(result.iterations) - 1}")
    print(f"Final error: {result.error():.6f}")
    print(f"Status: {result.status}")


if __name__ == "__main__":
    # symforce.set_epsilon_to_symbol()
    main()
