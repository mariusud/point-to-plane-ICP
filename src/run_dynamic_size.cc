#include <vector>
#include <chrono>
#include <spdlog/spdlog.h>

#include <symforce/opt/assert.h>
#include <symforce/opt/factor.h>
#include <symforce/opt/optimizer.h>
#include <symforce/opt/tic_toc.h>

#include "../gen/keys.h"
#include "../gen/point_to_plane_factor.h"
#include "./common.h"
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include "./visualize.h"

namespace ICP
{

    template <typename Scalar>
    sym::Factor<Scalar> CreatePointToPlaneFactor(const int i)
    {
        return sym::Factor<Scalar>::Hessian(
            ICP::PointToPlaneFactor<Scalar>,
            {sym::Keys::WORLD_T_LIDAR.WithSuper(0),
             sym::Keys::POINTS.WithSuper(i),
             sym::Keys::CENTROIDS.WithSuper(i),
             sym::Keys::NORMALS.WithSuper(i)},
            {sym::Keys::WORLD_T_LIDAR.WithSuper(0)});
    }

    template <typename Scalar>
    std::vector<sym::Factor<Scalar>> BuildDynamicFactors(const int kNumPoints)
    {
        std::vector<sym::Factor<Scalar>> factors;
        for (int i = 0; i < kNumPoints; ++i)
        {
            factors.push_back(CreatePointToPlaneFactor<Scalar>(i));
        }

        return factors;
    }

    void RunDynamic()
    {
        const int kNumPointsPerFace = 20;             // for each face, 6 faces on a cube
        const int kNumPoints = kNumPointsPerFace * 6; // kNumPointsPerFace * 6;
        const int kNumPoses = 1;

        // auto values = generate_points<double>(kNumPoints);
        auto values = build_cube_values<double>(kNumPointsPerFace);

        const std::vector<sym::Factor<double>> factors = BuildDynamicFactors<double>(kNumPoints);
        sym::Optimizer<double> optimizer(sym::DefaultOptimizerParams(), factors,
                                         "Point-To-PlaneOptimizerDynamic");

        spdlog::info("Initial pose: {}", values.At<sym::Pose3d>(sym::Keys::WORLD_T_LIDAR.WithSuper(0)));
        // spdlog::info("values: {}", values);
        spdlog::info("factors: {}", factors);
        auto start = std::chrono::steady_clock::now();

        const sym::Optimizerd::Stats stats = optimizer.Optimize(values);

        auto end = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        spdlog::info("Optimization time: {} milliseconds", elapsedTime);

        const auto &iteration_stats = stats.iterations;
        const auto &first_iter = iteration_stats.front();
        const auto &last_iter = iteration_stats.back();

        const auto &best_iter = iteration_stats[stats.best_index];

        spdlog::info("Iterations: {}", last_iter.iteration);
        spdlog::info("Lambda: {}", last_iter.current_lambda);
        spdlog::info("Initial error: {}", first_iter.new_error);
        spdlog::info("Final error: {}", best_iter.new_error);
        spdlog::info("Final pose: {}", values.At<sym::Pose3d>(sym::Keys::WORLD_T_LIDAR.WithSuper(0)));

        spdlog::info("Status: {}", stats.status);
        visualize(values, kNumPoints);
    }
}

int main()
{
    ICP::RunDynamic();
    return 0;
}