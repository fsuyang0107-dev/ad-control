#include "adc/vehicle_model.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "adc/integrator.hpp"

namespace adc {

void VehicleModel::StepKinematic(VehicleState& s, const ControlCmd& u, double dt) const {
    const double L = params_.wheelbase;
    const double v = s.v;  // 本文只做运动学,纵向速度视为本步常量

    std::vector<double> y{s.x, s.y, s.yaw};

    // 运动学自行车模型的导数:x' 和 y' 只依赖 yaw;yaw' 由转向角决定
    const auto f = [v, L, &u](const std::vector<double>& st) -> std::vector<double> {
        const double yaw = st[2];
        return {v * std::cos(yaw),
                v * std::sin(yaw),
                v / L * std::tan(u.steer)};
    };

    Rk4Step(y, dt, f);

    s.x = y[0];
    s.y = y[1];
    s.yaw = y[2];
    s.yaw_rate = v / L * std::tan(u.steer);
}

void VehicleModel::StepKinematicEuler(VehicleState& s, const ControlCmd& u, double dt) const {
    const double L = params_.wheelbase;
    const double v = s.v;

    s.x += dt * v * std::cos(s.yaw);
    s.y += dt * v * std::sin(s.yaw);
    s.yaw += dt * (v / L) * std::tan(u.steer);
    s.yaw_rate = v / L * std::tan(u.steer);
}

void VehicleModel::StepLongitudinal(VehicleState& s, const ControlCmd& u, double dt) const {
    // 1) 指令限幅到物理可行范围
    const double a_cmd = std::clamp(u.accel, params_.max_decel, params_.max_accel);

    // 2) 执行器一阶滞后:a' = (a_cmd - a) / tau
    //    用解析解,对任意 dt 都稳定
    if (params_.actuator_tau > 1e-6) {
        const double alpha = 1.0 - std::exp(-dt / params_.actuator_tau);
        s.accel += (a_cmd - s.accel) * alpha;
    } else {
        s.accel = a_cmd;
    }

    // 3) 速度积分
    s.v += s.accel * dt;
    if (s.v < 0.0) {
        s.v = 0.0;
        s.accel = std::max(s.accel, 0.0);
    }
}

}  // namespace adc
