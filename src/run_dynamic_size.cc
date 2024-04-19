#include <vector>

#include <spdlog/spdlog.h>

#include <symforce/opt/assert.h>
#include <symforce/opt/factor.h>
#include <symforce/opt/optimizer.h>
#include <symforce/opt/tic_toc.h>

#include "../gen/keys.h"
#include "../gen/point_to_plane_factor.h"
#include "./common.h"

template <typename Scalar>
sym::Factor<Scalar> CreatePointToPlaneFactor(const int i)
{
    return sym::Factor<Scalar>::Hessian(
        sym::PointToPlaneFactor<Scalar>,
        {sym::Keys::WORLD_T_LIDAR.WithSuper(i),
         sym::Keys::POINTS.WithSuper(i),
         sym::Keys::CENTROIDS.WithSuper(i),
         sym::Keys::NORMALS.WithSuper(i)},
        {sym::Keys::WORLD_T_LIDAR.WithSuper(i)});
}

template <typename Scalar>
std::vector<sym::Factor<Scalar>> BuildDynamicFactors(const int num_factors)
{
    std::vector<sym::Factor<Scalar>> factors;
    for (int i = 0; i < num_factors; ++i)
    {
        factors.push_back(CreatePointToPlaneFactor<Scalar>(i));
    }
    return factors;
}

int main()
{
    spdlog::info("Test");

    int kNumPointsPerFace = 1000;
    auto values = build_cube_values<double>(kNumPointsPerFace);
    spdlog::info("Initial values: {}", values);

    int num_poses = 1;

    const std::vector<sym::Factor<double>> factors = BuildDynamicFactors<double>(num_poses);
    spdlog::info("factors: {}", factors);
    // exit(0);
    sym::Optimizer<double> optimizer(sym::DefaultOptimizerParams(), factors,
                                     "Point-To-PlaneOptimizerDynamic");

    // Optimize
    spdlog::info("Optimizing...");
    const sym::Optimizerd::Stats stats = optimizer.Optimize(values);

    // Print out results
    spdlog::info("Optimized State:");
    for (int i = 0; i < num_poses; i++)
    {
        spdlog::info("Pose {}: {}", i, values.At<sym::Pose3d>(sym::Keys::WORLD_T_LIDAR.WithSuper(i)));
    }

    const auto &iteration_stats = stats.iterations;
    const auto &first_iter = iteration_stats.front();
    const auto &last_iter = iteration_stats.back();

    const auto &best_iter = iteration_stats[stats.best_index];

    spdlog::info("Iterations: {}", last_iter.iteration);
    spdlog::info("Lambda: {}", last_iter.current_lambda);
    spdlog::info("Initial error: {}", first_iter.new_error);
    spdlog::info("Final error: {}", best_iter.new_error);
    spdlog::info("Status: {}", stats.status);
}