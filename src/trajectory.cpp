#include "adc/trajectory.hpp"

#include <cmath>

namespace adc {

namespace {

constexpr double kPi = 3.14159265358979323846;

/// 由 x、y 及其对 x 的一阶、二阶导数构造一个轨迹点
TrajPoint MakePoint(double x, double y, double dydx, double d2ydx2, double v_ref) {
    TrajPoint p;
    p.x = x;
    p.y = y;
    p.yaw = std::atan(dydx);
    // 曲率:kappa = y'' / (1 + y'^2)^(3/2)
    const double denom = std::pow(1.0 + dydx * dydx, 1.5);
    p.curvature = (denom > 1e-12) ? (d2ydx2 / denom) : 0.0;
    p.v_ref = v_ref;
    return p;
}

}  // namespace

double Trajectory::length() const {
    double total = 0.0;
    for (std::size_t i = 1; i < pts_.size(); ++i) {
        const double dx = pts_[i].x - pts_[i - 1].x;
        const double dy = pts_[i].y - pts_[i - 1].y;
        total += std::hypot(dx, dy);
    }
    return total;
}

Trajectory MakeStraight(double length, double ds, double v_ref) {
    Trajectory traj;
    if (length <= 0.0 || ds <= 0.0) return traj;

    const int n = static_cast<int>(std::lround(length / ds));
    traj.reserve(static_cast<std::size_t>(n + 1));
    for (int i = 0; i <= n; ++i) {
        const double x = ds * i;
        traj.push_back(MakePoint(x, 0.0, 0.0, 0.0, v_ref));
    }
    return traj;
}

Trajectory MakeArc(double radius, double sweep_rad, double ds, double v_ref) {
    Trajectory traj;
    if (radius <= 0.0 || ds <= 0.0 || sweep_rad <= 0.0) return traj;

    const double arc_len = radius * sweep_rad;
    const int n = static_cast<int>(std::lround(arc_len / ds));
    traj.reserve(static_cast<std::size_t>(n + 1));

    const double kappa = 1.0 / radius;
    for (int i = 0; i <= n; ++i) {
        const double t = (sweep_rad / n) * i;  // 圆心角
        TrajPoint p;
        p.x = radius * std::sin(t);
        p.y = radius * (1.0 - std::cos(t));
        p.yaw = t;
        p.curvature = kappa;
        p.v_ref = v_ref;
        traj.push_back(p);
    }
    return traj;
}

///单移线
Trajectory MakeLaneChange(double length, double offset, double ds, double v_ref) {
    Trajectory traj;
    if (length <= 0.0 || ds <= 0.0) return traj;

    const int n = static_cast<int>(std::lround(length / ds));
    traj.reserve(static_cast<std::size_t>(n + 1));

    for (int i = 0; i <= n; ++i) {
        const double x = ds * i;
        const double u = x / length;  // 归一化进度 [0, 1]

        // 五次多项式:保证两端位置、速度、加速度连续
        const double y = offset * (10.0 * std::pow(u, 3) - 15.0 * std::pow(u, 4) +
                                   6.0 * std::pow(u, 5));
        const double dydx = offset * (30.0 * u * u - 60.0 * std::pow(u, 3) +
                                      30.0 * std::pow(u, 4)) / length;
        const double d2ydx2 = offset * (60.0 * u - 180.0 * u * u +
                                        120.0 * std::pow(u, 3)) / (length * length);

        traj.push_back(MakePoint(x, y, dydx, d2ydx2, v_ref));
    }
    return traj;
}

/// 双移线:0 → offset → 0,峰值在中间
Trajectory MakeDoubleLaneChange(double length, double offset, double ds, double v_ref) {
    Trajectory traj;
    if (length <= 0.0 || ds <= 0.0) return traj;

    const int n = static_cast<int>(std::lround(length / ds));
    if (n <= 0) return traj;
    traj.reserve(static_cast<std::size_t>(n + 1));

    // 五次多项式 S(u) 及其对 u 的一、二阶导
    auto S   = [](double u) { return 10.0*std::pow(u,3) - 15.0*std::pow(u,4) + 6.0*std::pow(u,5); };
    auto Sp  = [](double u) { return 30.0*u*u - 60.0*std::pow(u,3) + 30.0*std::pow(u,4); };
    auto Spp = [](double u) { return 60.0*u - 180.0*u*u + 120.0*std::pow(u,3); };

    for (int i = 0; i <= n; ++i) {
        const double x = ds * i;
        const double r = x / length;           // 全程归一化进度 [0,1]

        double y, dydx, d2ydx2;
        if (r <= 0.5) {
            // 前半段:0 → offset,归一化到 [0,1]
            const double u = 2.0 * r;
            y      = offset * S(u);
            dydx   = offset * Sp(u)  * (2.0 / length);          // du/dx = 2/L
            d2ydx2 = offset * Spp(u) * (2.0 / length) * (2.0 / length);
        } else {
            // 后半段:offset → 0
            const double u = 2.0 * (r - 0.5);
            y      = offset * (1.0 - S(u));
            dydx   = -offset * Sp(u)  * (2.0 / length);
            d2ydx2 = -offset * Spp(u) * (2.0 / length) * (2.0 / length);
        }
        traj.push_back(MakePoint(x, y, dydx, d2ydx2, v_ref));
    }
    return traj;
}

Trajectory MakeSine(double length, double amplitude, double periods, double ds,
                    double v_ref) {
    Trajectory traj;
    if (length <= 0.0 || ds <= 0.0 || periods <= 0.0) return traj;

    const double w = 2.0 * kPi * periods / length;  // 空间角频率
    const double a2w2 = amplitude * amplitude * w * w;  // (A*w)^2

    const int n = static_cast<int>(std::lround(length / ds));
    traj.reserve(static_cast<std::size_t>(n + 1));

    // 用牛顿迭代把"沿弧长均匀采样"换成对应 x
    for (int i = 0; i <= n; ++i) {
        const double s = ds * i;

        // 弧长近似:s(x) ≈ x·(1 + eps/4) + eps/(8w)·sin(2wx)
        // 来自 ∫√(1 + eps·cos²(wx))dx 的一阶展开
        const double k1=1.0+0.25*a2w2;
        const double k2=a2w2/(8*w);
        double x = s/k1;
        for (int it = 0; it < 8; ++it) {
            const double fval = k1*x + k2 * std::sin(2.0 * w * x) - s;
            const double dfdx = k1 + 2* w * k2 * std::cos(2.0 * w * x);
            x -= fval / dfdx;
        }

        const double y = amplitude * std::sin(w * x);
        const double dydx = amplitude * w * std::cos(w * x);
        const double d2ydx2 = -amplitude * w * w * std::sin(w * x);

        traj.push_back(MakePoint(x, y, dydx, d2ydx2, v_ref));
    }
    return traj;
}

}  // namespace adc
