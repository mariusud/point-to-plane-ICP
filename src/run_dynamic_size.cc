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
        const int kNumPointsPerFace = 1;              // for each face, 6 faces on a cube
        const int kNumPoints = kNumPointsPerFace * 6; // kNumPointsPerFace * 6;
        const int kNumPoses = 1;

        // auto values = generate_points<double>(kNumPoints);
        sym::Valuesd values = build_cube_values(kNumPointsPerFace);

        const std::vector<sym::Factord> factors = BuildDynamicFactors<double>(kNumPoints);

        sym::VectorX<double> res;
        factors[0].Linearize(values, &res);
        spdlog::info("res: {}", res);
        for (int i = 0; i < kNumPoints; i++)
        {
            sym::VectorX<double> res;
            factors[i].Linearize(values, &res);
            spdlog::info("res {}: {}", i, res);
            spdlog::info("point: {}", values.At<sym::Vector3d>(sym::Keys::POINTS.WithSuper(i)));
            spdlog::info("centroid: {}", values.At<sym::Vector3d>(sym::Keys::CENTROIDS.WithSuper(i)));
            spdlog::info("normal: {}", values.At<sym::Vector3d>(sym::Keys::NORMALS.WithSuper(i)));
        }

        sym::optimizer_params_t params = sym::DefaultOptimizerParams();
        params.verbose = true;
        params.debug_stats = true;
        params.check_derivatives = true;
        params.include_jacobians = true;
        params.iterations = 500;

        sym::Optimizer<double> optimizer(params, factors,
                                         "Point-To-PlaneOptimizerDynamic");

        spdlog::info("Initial pose: {}", values.At<sym::Pose3d>(sym::Keys::WORLD_T_LIDAR.WithSuper(0)));

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