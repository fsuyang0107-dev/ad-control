#include "adc/metrics.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace adc {

namespace {
constexpr double kPi = 3.14159265358979323846;
}  // namespace

double NormalizeAngle(double a) {
    const double two_pi = 2.0 * kPi;
    a = std::fmod(a, two_pi);
    if (a > kPi) a -= two_pi;
    if (a <= -kPi) a += two_pi;   // -pi 映射为 +pi,保证区间是 (-pi, pi]
    return a;
}

Metrics ComputeMetrics(const std::vector<SimRecord>& recs) {
    Metrics m;
    m.steps = recs.size();
    if (recs.empty()) return m;

    double sum_y2 = 0.0, sum_yaw2 = 0.0, sum_v2 = 0.0;
    for (const auto& r : recs) {
        sum_y2 += r.e_y * r.e_y;
        sum_yaw2 += r.e_yaw * r.e_yaw;
        sum_v2 += r.e_v * r.e_v;
        m.max_abs_y = std::max(m.max_abs_y, std::abs(r.e_y));
        m.max_abs_yaw = std::max(m.max_abs_yaw, std::abs(r.e_yaw));
    }

    const double n = static_cast<double>(recs.size());
    m.rmse_y = std::sqrt(sum_y2 / n);
    m.rmse_yaw = std::sqrt(sum_yaw2 / n);
    m.rmse_v = std::sqrt(sum_v2 / n);

    for (std::size_t i = 1; i < recs.size(); ++i) {
        const double dt = recs[i].t - recs[i - 1].t;
        if (dt > 1e-12) {
            // jerk 用【实际加速度】—— 这才是乘客感受到的量
            const double j = (recs[i].state.accel - recs[i - 1].state.accel) / dt;
            m.max_jerk = std::max(m.max_jerk, std::abs(j));
        }
    }
    return m;
}

bool WriteCsv(const std::vector<SimRecord>& recs, const std::string& path) {
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (f == nullptr) return false;

    std::fprintf(f,
                 "t,x,y,yaw,v,yaw_rate,steer,accel,e_y,e_yaw,e_v,v_ref,ref_index,ref_x,ref_y\n");
    for (const auto& r : recs) {
        std::fprintf(f,
                     "%.6f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%zu,"
                     "%.9f,%.9f\n",
                     r.t, r.state.x, r.state.y, r.state.yaw, r.state.v,
                     r.state.yaw_rate, r.cmd.steer, r.cmd.accel, r.e_y, r.e_yaw,
                     r.e_v, r.v_ref, r.ref_index, r.ref_x, r.ref_y);
    }
    std::fclose(f);
    return true;
}

void PrintMetrics(const Metrics& m) {
    std::printf("  steps      : %zu\n", m.steps);
    std::printf("  rmse_y     : %.6f m\n", m.rmse_y);
    std::printf("  max|e_y|   : %.6f m\n", m.max_abs_y);
    std::printf("  rmse_yaw   : %.6f rad (%.4f deg)\n", m.rmse_yaw,
                m.rmse_yaw * 180.0 / kPi);
    std::printf("  max|e_yaw| : %.6f rad (%.4f deg)\n", m.max_abs_yaw,
                m.max_abs_yaw * 180.0 / kPi);
    std::printf("  rmse_v     : %.6f m/s\n", m.rmse_v);
    std::printf("  max|jerk|  : %.6f m/s^3\n", m.max_jerk);
}

}  // namespace adc
