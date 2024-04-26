#pragma once

#include <symforce/opt/factor.h>
namespace ICP
{

    void RunFixed();
    extern template sym::Factor<float> BuildFixedFactor<double>();
} // namespace ICP
