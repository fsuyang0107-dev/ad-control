#pragma once

#include <cstddef>
#include <string>

namespace adc {

struct VehicleState {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double v{0.0};
    double yaw_rate{0.0};
    double accel{0.0};  // 实际加速度(经执行器滞后后),m/s^2
    double vy{0.0};          // 重心侧向速度,m/s(仅动力学模型使用)
};

struct ControlCmd {
    double steer{0.0};
    double accel{0.0};
};

struct VehicleParams {
    double wheelbase{2.7};
    double mass{1500.0};
    double lf{1.2};
    double lr{1.5};
    double Cf{80000.0};
    double Cr{80000.0};
    double Iz{2500.0};
    double max_steer{0.5236};
    double max_accel{3.0};
    double max_decel{-5.0};
    double actuator_tau{0.15};  // 执行器一阶滞后时间常数,s
    double mu{0.9};             // 路面附着系数
    double g{9.81};             // 重力加速度,m/s^2
};

struct TrajPoint {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double v_ref{0.0};
    double curvature{0.0};
};

struct SimRecord {
    double t{0.0};
    VehicleState state;
    ControlCmd cmd;
    double e_y{0.0};
    double e_yaw{0.0};
    double e_v{0.0};
    double v_ref{0.0};
    std::size_t ref_index{0};
    double ref_x{0.0};   // 最近参考点坐标(便于离线画图对比)
    double ref_y{0.0};
};

/// 车辆模型类型
enum class ModelKind { kKinematic = 0, kDynamic = 1 };

/// 线性 MPC 整定参数(放在这里以便 SimConfig 直接持有,避免循环依赖)
struct MpcParams {
    int horizon{20};          ///< 预测步数
    double tau{0.02};         ///< 预测步长 [s]
    double q_ey{20.0};        ///< 横向误差权重
    double q_eyaw{5.0};       ///< 航向误差权重
    double q_vy{0.2};         ///< 侧向速度权重
    double q_r{0.2};          ///< 横摆角速度权重
    double qf_scale{5.0};     ///< 终端权重倍数
    double r_steer{2.0};      ///< 转角权重
    bool use_constraints{true};  ///< 是否在优化内处理 |δ| ≤ max_steer
};

/// 控制器类型
enum class CtrlMode { kFeedForward = 0, kPurePursuit = 1, kStanley = 2, kMpc = 3 };

/// 控制器参数
struct CtrlParams {
    // Pure Pursuit:前视距离 Ld = lookahead_base + lookahead_time * v
    double lookahead_base{2.0};
    double lookahead_time{0.08};
    // Stanley:横向误差增益
    double stanley_gain{4.0};
    // Stanley:航向误差增益(运动学模型可设 1.0;动力学模型需要更小以免振荡)
    double stanley_yaw_gain{1.0};
    // 是否叠加曲率前馈
    bool feedforward{true};
};

/// 纵向 PI(D)控制参数
struct LongCtrlParams {
    double kp{3.0};              // (m/s^2) / (m/s)
    double ki{0.25};             // (m/s^2) / (m/s * s)
    double kd{0.0};
    double integral_limit{1.0};  // 积分限幅,抗饱和
};

struct SimConfig {
    double dt{0.02};
    double duration{20.0};
    std::string scenario{"straight"};
    double init_y_offset{0.0};
    double init_yaw_offset{0.0};
    ModelKind model{ModelKind::kKinematic};
    CtrlMode ctrl_mode{CtrlMode::kPurePursuit};
    MpcParams mpc;
    CtrlParams ctrl;
    LongCtrlParams long_ctrl;
    VehicleParams vehicle;
};

}  // namespace adc
