#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include "./visualize.h"
#include "../gen/keys.h"
#include "./common.h"

void visualize_values(pcl::visualization::PCLVisualizer::Ptr &viewer, const sym::Values<double> &values, int kNumPoints)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr normals_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    sym::Pose3d lidar_T_world = values.At<sym::Pose3d>(sym::Keys::WORLD_T_LIDAR.WithSuper(0)).Inverse();

    for (int i = 0; i < kNumPoints; ++i)
    {
        Eigen::Vector3d point = values.At<Eigen::Vector3d>(sym::Keys::POINTS.WithSuper(i));
        Eigen::Vector3d transformed_point = lidar_T_world.Compose(point);

        pcl::PointXYZ pcl_point;
        pcl_point.x = transformed_point.x();
        pcl_point.y = transformed_point.y();
        pcl_point.z = transformed_point.z();
        cloud->points.push_back(pcl_point);

        pcl::PointXYZ normal_point;
        normal_point.x = point.x();
        normal_point.y = point.y();
        normal_point.z = point.z();
        normals_cloud->points.push_back(normal_point);

        Eigen::Vector3d normal = values.At<Eigen::Vector3d>(sym::Keys::NORMALS.WithSuper(i));
        pcl::Normal pcl_normal;
        pcl_normal.normal_x = normal.x();
        pcl_normal.normal_y = normal.y();
        pcl_normal.normal_z = normal.z();
        normals->points.push_back(pcl_normal);
    }

    viewer->addPointCloud<pcl::PointXYZ>(cloud, "points");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, "points");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "points");

    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(normals_cloud, normals, 1, 0.1, "normals");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "normals");
}

void visualize_cube_faces(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    auto [cube_vertices, cube_faces] = generate_cube();

    for (size_t i = 0; i < cube_faces.size(); ++i)
    {
        const auto &face = cube_faces[i];
        const auto &v0 = cube_vertices[face[0]];
        const auto &v1 = cube_vertices[face[1]];
        const auto &v2 = cube_vertices[face[2]];

        Eigen::Vector3d normal = (v1 - v0).cross(v2 - v0).normalized();

        pcl::ModelCoefficients coeffs;
        coeffs.values.resize(4);
        coeffs.values[0] = normal.x();
        coeffs.values[1] = normal.y();
        coeffs.values[2] = normal.z();
        coeffs.values[3] = -normal.dot(v0); // d parameter of the plane equation

        std::string plane_name = "plane_" + std::to_string(i);
        viewer->addPlane(coeffs, plane_name);
    }
}

void visualize(const sym::Values<double> &values, int kNumPoints)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();

    visualize_cube_faces(viewer);
    visualize_values(viewer, values, kNumPoints);

    viewer->spinOnce(5000);
}
