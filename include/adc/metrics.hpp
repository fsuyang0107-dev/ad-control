#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "adc/types.hpp"

namespace adc {

struct Metrics {
    std::size_t steps{0};
    double rmse_y{0.0};
    double max_abs_y{0.0};
    double rmse_yaw{0.0};
    double max_abs_yaw{0.0};
    double rmse_v{0.0};
    double max_jerk{0.0};
};

/// 把角度归一化到 (-pi, pi]
double NormalizeAngle(double a);

Metrics ComputeMetrics(const std::vector<SimRecord>& recs);

bool WriteCsv(const std::vector<SimRecord>& recs, const std::string& path);

void PrintMetrics(const Metrics& m);

}  // namespace adc
