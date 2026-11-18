#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "adc/metrics.hpp"
#include "adc/trajectory.hpp"
#include "adc/types.hpp"
#include "adc/vehicle_model.hpp"

namespace adc {

std::size_t FindNearest(const Trajectory& traj, const VehicleState& s);

/// 沿轨迹向前找"弧长距离 >= lookahead"的点
std::size_t FindLookahead(const Trajectory& traj, std::size_t idx, double lookahead);

/// 纯前馈(baseline)
ControlCmd FeedForward(const TrajPoint& rp, const VehicleParams& p);

/// Pure Pursuit:几何法,朝前视点转
ControlCmd PurePursuitCmd(const Trajectory& traj, const VehicleState& s, std::size_t idx,
                          const VehicleParams& p, const CtrlParams& cp);

/// Stanley:航向误差 + 横向误差 + 曲率前馈
ControlCmd StanleyCmd(const VehicleState& s, const TrajPoint& rp, const VehicleParams& p,
                      const CtrlParams& cp);

/// 按模式分发
ControlCmd ComputeControl(CtrlMode mode, const Trajectory& traj, const VehicleState& s,
                          const TrajPoint& rp, std::size_t idx, const SimConfig& cfg);

std::string CtrlModeName(CtrlMode m);

/// 纵向 PI(D)速度控制器(含条件积分抗饱和)
class LongitudinalPid {
public:
    LongitudinalPid(const LongCtrlParams& p, double a_min, double a_max)
        : p_(p), a_min_(a_min), a_max_(a_max) {}
    void Reset();
    /// 返回加速度指令,m/s^2;a_ff 为参考加速度前馈
    double Compute(double v_ref, double v, double a_ff, double dt);
    double integral() const { return integ_; }

private:
    LongCtrlParams p_;
    double a_min_;
    double a_max_;
    double integ_{0.0};
    double prev_err_{0.0};
};

class SimLoop {
public:
    explicit SimLoop(const SimConfig& cfg) : cfg_(cfg), model_(cfg.vehicle) {}

    std::vector<SimRecord> Run(const Trajectory& ref) const;
    const SimConfig& config() const { return cfg_; }

private:
    SimConfig cfg_;
    VehicleModel model_;
};

}  // namespace adc
