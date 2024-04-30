
inline sym::Valuesd generate_points(int num_points)
{
    sym::Valuesd values;

    static const std::vector<Eigen::Vector3d> cube_vertices = {
        {-0.5, -0.5, -0.5},
        {-0.5, -0.5, 0.5},
        {-0.5, 0.5, -0.5},
        {-0.5, 0.5, 0.5},
        {0.5, -0.5, -0.5},
        {0.5, -0.5, 0.5},
        {0.5, 0.5, -0.5},
        {0.5, 0.5, 0.5}};

    for (int i = 0; i < num_points; ++i)
    {
        int face_index = std::rand() % 6;

        const auto &v0 = cube_vertices[face_index * 4];
        const auto &v1 = cube_vertices[face_index * 4 + 1];
        const auto &v2 = cube_vertices[face_index * 4 + 2];
        const auto &v3 = cube_vertices[face_index * 4 + 3];

        double u = static_cast<double>(std::rand()) / RAND_MAX;
        double v = static_cast<double>(std::rand()) / RAND_MAX;

        Eigen::Vector3d point = (1 - u) * (1 - v) * v0 + u * (1 - v) * v1 + u * v * v2 + (1 - u) * v * v3;
        Eigen::Vector3d normal = (v1 - v0).cross(v3 - v0).normalized();

        Eigen::Vector3d centroid = (v0 + v1 + v2 + v3) / 4.0;

        values.Set(sym::Keys::POINTS.WithSuper(i), point);
        values.Set(sym::Keys::NORMALS.WithSuper(i), normal);
        values.Set(sym::Keys::CENTROIDS.WithSuper(i), centroid);

        // Set initial pose
        sym::Pose3d initial_pose;
        double t = 40;

        // Calculate tangent vector components
        Eigen::Matrix<double, 6, 1> tangent_vec;
        tangent_vec << 0, 0, 0, 1, 1, 1;

        // Create the pose from the tangent vector
        sym::Pose3<double> world_T_lidar = sym::Pose3<double>::FromTangent(tangent_vec);
        values.Set(sym::Keys::WORLD_T_LIDAR.WithSuper(i), world_T_lidar);
    }

    // Set epsilon
    values.Set(sym::Keys::EPSILON, sym::kDefaultEpsilon<double>);

    return values;
}