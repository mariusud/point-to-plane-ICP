#pragma once

#include <sym/pose3.h>
#include <symforce/opt/values.h>

void visualize_dynamic(const sym::Values<double> &values, int kNumPoints);
void visualize_fixed(const sym::Values<double> &values, int kNumPoints);
