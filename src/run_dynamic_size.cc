#include <vector>

#include <spdlog/spdlog.h>

#include <symforce/opt/assert.h>
#include <symforce/opt/factor.h>
#include <symforce/opt/optimizer.h>
#include <symforce/opt/tic_toc.h>

#include "../gen/keys.h"
#include "../gen/point_to_plane_factor.h"

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

    auto values = BuildValues<double>();

    int num_poses = 10;

    const std::vector<sym::Factor<double>> factors = BuildDynamicFactors<double>(num_poses);

    sym::Optimizer<double> optimizer(sym::DefaultOptimizerParams(), factors,
                                     "Point-To-PlaneOptimizerDynamic");

    // Optimize
    const sym::Optimizerd::Stats stats = optimizer.Optimize(values);

    // Print out results
    spdlog::info("Optimized State:");
    for (int i = 0; i < num_poses; i++)
    {
        spdlog::info("Pose {}: {}", i, values.At<sym::Pose3d>(sym::Keys::WORLD_T_BODY.WithSuper(i)));
    }
}