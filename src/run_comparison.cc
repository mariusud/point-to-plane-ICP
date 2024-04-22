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
#include "../gen/linearization.h"
#include "./run_dynamic_size"
#include "./run_fixed_size"

int main()
{
    auto start = std::chrono::steady_clock::now();
    ICP::RunFixed(kNumPoses, kNumPoints);
    auto end = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    spdlog::info("Fixed-size total time elapsed: {} milliseconds", elapsedTime);

    auto start = std::chrono::steady_clock::now();
    ICP::RunDynamic(kNumPoses, kNumPoints);
    auto end = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    spdlog::info("Dynamic-size total time elapsed: {} milliseconds", elapsedTime);

    return 0;
}