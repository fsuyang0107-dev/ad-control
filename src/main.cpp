#include <cstdio>
#include <string>

#include "adc/trajectory.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

void DumpTrajectory(const std::string& name, const adc::Trajectory& t) {
    std::printf("\n=== %s ===\n", name.c_str());
    std::printf("点数: %zu  长度: %.3f m\n", t.size(), t.length());
    if (t.empty()) return;

    double max_k = 0.0;
    for (const auto& p : t.points()) max_k = std::max(max_k, std::abs(p.curvature));

    std::printf("最大曲率: %.6f 1/m  (对应最小转弯半径 %.2f m)\n", max_k,
                max_k > 1e-9 ? 1.0 / max_k : 0.0);
    std::printf("%8s %10s %10s %10s %12s\n", "index", "x(m)", "y(m)", "yaw(rad)", "curvature");
    const std::size_t step = t.size() / 8 < 1 ? 1 : t.size() / 8;
    for (std::size_t i = 0; i < t.size(); i += step)
        std::printf("%8zu %10.3f %10.3f %10.4f %12.6f\n", i, t[i].x, t[i].y, t[i].yaw,
                    t[i].curvature);
}

}  // namespace

int main() {
    const double ds = 0.1;
    const double v_ref = 10.0;

    DumpTrajectory("直线 (20 m)", adc::MakeStraight(20.0, ds, v_ref));
    DumpTrajectory("圆弧 (R=25 m, 90 deg)", adc::MakeArc(25.0, kPi / 2.0, ds, v_ref));
    DumpTrajectory("双移线 (60 m, 横移 3.5 m)",
                   adc::MakeDoubleLaneChange(60.0, 3.5, ds, v_ref));
    DumpTrajectory("正弦 (50 m, 振幅 2 m, 2 周期)",
                   adc::MakeSine(50.0, 2.0, 2.0, ds, v_ref));

    return 0;
}
