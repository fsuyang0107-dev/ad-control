#pragma once

#include "adc/types.hpp"

namespace adc {

/// 车辆模型。
/// Day 4 只实现运动学自行车模型(假设无侧滑、低速):
///     x'   = v * cos(yaw)
///     y'   = v * sin(yaw)
///     yaw' = v / L * tan(steer)
class VehicleModel {
public:
    explicit VehicleModel(const VehicleParams& p) : params_(p) {}

    const VehicleParams& params() const { return params_; }

    /// 用 RK4 积分推进一个时间步(推荐)
    void StepKinematic(VehicleState& s, const ControlCmd& u, double dt) const;

    /// 用显式欧拉法推进一个时间步(仅用于精度对比)
    void StepKinematicEuler(VehicleState& s, const ControlCmd& u, double dt) const;

    /// 执行器一阶滞后:更新 s.accel
    void UpdateActuator(VehicleState& s, const ControlCmd& u, double dt) const;

    /// 纵向(运动学配套):执行器 + 速度积分
    void StepLongitudinal(VehicleState& s, const ControlCmd& u, double dt) const;

    /// 二自由度动力学模型:6 状态 RK4
    /// 状态 = [vx, vy, r, X_rear, Y_rear, psi]
    void StepDynamic(VehicleState& s, const ControlCmd& u, double dt) const;

private:
    VehicleParams params_;
};

}  // namespace adc
