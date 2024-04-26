import symforce

symforce.set_epsilon_to_symbol()


from pathlib import Path
import symforce.symbolic as sf
from symforce.codegen import Codegen
from symforce.codegen import CodegenConfig
from symforce.codegen import CppConfig
from symforce.codegen import values_codegen

from visualize import build_cube_values, point_to_plane_residual
import numpy as np
import sym
from symforce import typing as T
from symforce.values import Values

import re
import shutil
import textwrap

NUM_POINTS_PER_FACE = 20
NUM_CORRESPONDENCES = NUM_POINTS_PER_FACE * 6


def build_residual(num_correspondences, values: Values) -> Values:
    residuals = []
    for i in range(num_correspondences):
        residuals.append(
            sf.V1(
                point_to_plane_residual(
                    values.attr.world_T_lidar,
                    values.attr.points[i],
                    values.attr.centroids[i],
                    values.attr.normals[i],
                )
            )
        )
    return sf.Matrix.block_matrix([[residual] for residual in residuals])


def build_codegen_object(config: T.Optional[CodegenConfig] = None) -> Codegen:
    """
    Create Codegen object for the linearization function
    """
    if config is None:
        config = CppConfig()

    values = build_cube_values(NUM_POINTS_PER_FACE)

    def symbolic(k: str, v: T.Any) -> T.Any:
        if isinstance(v, sym.Pose3):
            return sf.Pose3.symbolic(k)
        elif isinstance(v, float):
            return sf.Symbol(k)
        elif isinstance(v, np.ndarray):
            if len(v.shape) == 1:
                return sf.Matrix(v.shape[0], 1).symbolic(k)
            else:
                return sf.Matrix(*v.shape).symbolic(k)
        else:
            assert False, k

    values = Values(**{key: symbolic(key, v) for key, v in values.items_recursive()})

    residual = build_residual(NUM_CORRESPONDENCES, values)

    flat_keys = {key: re.sub(r"[\.\[\]]+", "_", key) for key in values.keys_recursive()}

    inputs = Values(
        **{flat_keys[key]: value for key, value in values.items_recursive()}
    )
    outputs = Values(residual=residual)

    optimized_keys = ["world_T_lidar"]

    linearization_func = Codegen(
        inputs=inputs,
        outputs=outputs,
        config=config,
        docstring=textwrap.dedent(
            """
            This function was autogenerated. Do not modify by hand.

            Computes the linearization of the residual around the given state,
            and returns the relevant information about the resulting linear system.

            Input args: The state to linearize around

            Output args:
                residual (Eigen::Matrix*): The residual vector
            """
        ),
    ).with_linearization(
        name="linearization",
        which_args=[flat_keys[key] for key in optimized_keys],
        sparse_linearization=True,
    )

    return linearization_func


def generate_point_to_plane_residual_code(
    output_dir: T.Optional[Path] = None, print_code: bool = False
) -> None:
    """
    Generate C++ code for the point-to-plane residual function. A C++ Factor can then be
    constructed and optimized from this function without any Python dependency.
    """
    # Create a Codegen object for the symbolic residual function, targeted at C++
    codegen = Codegen.function(point_to_plane_residual, config=CppConfig())

    # Create a Codegen object that computes a linearization from the residual Codegen object,
    # by introspecting and symbolically differentiating the given arguments
    codegen_with_linearization = codegen.with_linearization(
        which_args=["world_T_lidar"]
    )

    # Generate the function and print the code
    generated_paths = codegen_with_linearization.generate_function(
        output_dir=output_dir,
        namespace="ICP",
        skip_directory_nesting=True,
    )
    if print_code:
        print(generated_paths.generated_files[0].read_text())

    # generate_function writes to a new temporary directory if output_dir is None. Delete the
    # temporary directory.
    if output_dir is None:
        shutil.rmtree(generated_paths.output_dir)


def generate(output_dir: Path) -> None:
    generate_point_to_plane_residual_code(output_dir)
    values_codegen.generate_values_keys(
        build_cube_values(NUM_POINTS_PER_FACE),
        output_dir,
        config=CppConfig(),
        skip_directory_nesting=True,
    )
    build_codegen_object(config=CppConfig()).generate_function(
        output_dir,
        namespace="ICP",
        skip_directory_nesting=True,
    )


if __name__ == "__main__":
    generate(Path("gen"))
