#include <vector>
#include <chrono>

#include <spdlog/spdlog.h>

#include <symforce/opt/factor.h>
#include <symforce/opt/optimizer.h>
#include <symforce/opt/tic_toc.h>

#include "../gen/keys.h"
#include "../gen/point_to_plane_factor.h"
#include "./common.h"
#include "../gen/linearization.h"

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include "./visualize.h"

namespace ICP
{

    template <typename Scalar>
    sym::Factor<Scalar> BuildFixedFactor(const int kNumPoses, const int kNumPoints)
    {
        std::vector<sym::Key> factor_keys;

        for (int i = 0; i < kNumPoints; i++)
        {
            factor_keys.push_back(sym::Keys::POINTS.WithSuper(i));
            factor_keys.push_back(sym::Keys::CENTROIDS.WithSuper(i));
            factor_keys.push_back(sym::Keys::NORMALS.WithSuper(i));
        }

        for (int i = 0; i < kNumPoses; i++)
        {
            factor_keys.push_back(sym::Keys::WORLD_T_LIDAR.WithSuper(i));
        }

        factor_keys.push_back(sym::Keys::EPSILON);

        std::vector<sym::Key> optimized_keys;
        for (int i = 0; i < kNumPoses; i++)
        {
            optimized_keys.push_back(sym::Keys::WORLD_T_LIDAR.WithSuper(i));
        }
        spdlog::info("factor_keys: {}", factor_keys);
        return sym::Factor<Scalar>::Hessian(Linearization<Scalar>, factor_keys, optimized_keys);
    }

    void RunFixed()
    {
        const int kNumPointsPerFace = 1; // for each face, 6 faces on a cube
        const int kNumPoints = 2;        // kNumPointsPerFace * 6;
        const int kNumPoses = 1;

        // auto values = build_cube_values<double>(kNumPointsPerFace);
        auto values = generate_points<double>(kNumPoints);
        spdlog::info("values: {}", values);

        // visualize(values, kNumPoints);
        const std::vector<sym::Factor<double>>
            factors = {BuildFixedFactor<double>(kNumPoses, kNumPoints)};
        spdlog::info("factors: {}", factors);

        sym::optimizer_params_t params = sym::DefaultOptimizerParams();
        params.verbose = true;

        sym::Optimizer<double> optimizer(params, factors,
                                         "Point-To-PlaneOptimizerFixed");

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
        // viewer->spinOnce(5000);
    }

} // namespace ICP

int main()
{
    ICP::RunFixed();
    return 0;
}