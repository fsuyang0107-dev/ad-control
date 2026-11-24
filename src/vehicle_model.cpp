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

void VehicleModel::UpdateActuator(VehicleState& s, const ControlCmd& u, double dt) const {
    // 1) 指令限幅到物理可行范围
    const double a_cmd = std::clamp(u.accel, params_.max_decel, params_.max_accel);

    // 2) 一阶滞后(精确解,对任意 dt 稳定)
    if (params_.actuator_tau > 1e-6) {
        const double alpha = 1.0 - std::exp(-dt / params_.actuator_tau);
        s.accel += (a_cmd - s.accel) * alpha;
    } else {
        s.accel = a_cmd;
    }
}

void VehicleModel::StepLongitudinal(VehicleState& s, const ControlCmd& u, double dt) const {
    UpdateActuator(s, u, dt);

    // 速度积分(运动学模型:纵向独立)
    s.v += s.accel * dt;
    if (s.v < 0.0) {
        s.v = 0.0;
        s.accel = std::max(s.accel, 0.0);
    }
}

void VehicleModel::StepDynamic(VehicleState& s, const ControlCmd& u, double dt) const {
    UpdateActuator(s, u, dt);



    const double L = params_.lf + params_.lr;
    const double m = params_.mass;
    const double Iz = params_.Iz;
    const double lf = params_.lf;
    const double lr = params_.lr;
    const double Cf = params_.Cf;
    const double Cr = params_.Cr;
    const double a_cmd = s.accel;          // 执行器输出,视为本步常量
    const double delta = std::clamp(u.steer, -params_.max_steer, params_.max_steer);

    // 前/后轴静态载荷与附着极限
    const double Fzf = m * params_.g * lr / L;
    const double Fzr = m * params_.g * lf / L;
    const double Fmax_f = params_.mu * Fzf;
    const double Fmax_r = params_.mu * Fzr;

    // 状态 = [vx, vy, r, X_rear, Y_rear, psi]
    std::vector<double> y{s.v, s.vy, s.yaw_rate, s.x, s.y, s.yaw};

    const auto f = [&](const std::vector<double>& st) -> std::vector<double> {
        const double vx = st[0];
        const double vy = st[1];
        const double r = st[2];
        const double psi = st[5];

        // 低速保护:避免 1/vx 病态
        const double vxs = std::max(vx, 0.5);

        // 侧偏角
        const double alpha_f = delta - std::atan2(vy + lf * r, vxs);
        const double alpha_r = -std::atan2(vy - lr * r, vxs);

        // 低速正则化:静止时轮胎不产生侧向力
        // (否则 alpha 在 v->0 处奇异,会凭空产生横摆力矩)
        const double force_scale = std::clamp(vx / 1.0, 0.0, 1.0);

        // 线性轮胎 + 附着饱和(硬截断)
        const double Fyf = std::clamp(Cf * alpha_f, -Fmax_f, Fmax_f) * force_scale;
        const double Fyr = std::clamp(Cr * alpha_r, -Fmax_r, Fmax_r) * force_scale;

        const double cosd = std::cos(delta);

        // 三个动力学方程(含 vx*r、vy*r 伪力项)
        const double dvx = a_cmd + vy * r;
        const double dvy = (Fyf * cosd + Fyr) / m - vx * r;
        const double dr = (lf * Fyf * cosd - lr * Fyr) / Iz;

        // 后轴速度(刚体运动学):v_rear = (vx, vy - lr*r)
        const double vrx = vx;
        const double vry = vy - lr * r;

        const double dX = vrx * std::cos(psi) - vry * std::sin(psi);
        const double dY = vrx * std::sin(psi) + vry * std::cos(psi);
        const double dpsi = r;
        return {dvx, dvy, dr, dX, dY, dpsi};
    };

    Rk4Step(y, dt, f);

    s.v = std::max(y[0], 0.0);
    s.vy = y[1];
    s.yaw_rate = y[2];
    s.x = y[3];
    s.y = y[4];
    s.yaw = y[5];
}

}  // namespace adc
