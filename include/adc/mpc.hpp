#pragma once

#include <vector>

#include "adc/types.hpp"

namespace adc {

/// 路径相对误差状态:x = [e_y, e_yaw, v_y, r]
///
///   ė_y   = v_x·e_yaw + v_y
///   ė_yaw = r − v_x·κ        ← 参考曲率作为已知扰动
///   v̇_y   = a11·v_y + a12·r + (C_f/m)·δ
///   ṙ     = a21·v_y + a22·r + (l_f·C_f/I_z)·δ
struct MpcState {
    double e_y{0.0};
    double e_yaw{0.0};
    double v_y{0.0};
    double r{0.0};
};

/// 一次求解的诊断信息
struct MpcInfo {
    bool ok{false};          ///< Cholesky 是否成功
    double cost{0.0};        ///< 代价值 ½uᵀHu + fᵀu(即按 ½ 约定定义的 J,不含常数项)
    double steer{0.0};       ///< 首步最优转角(限幅前)
    bool saturated{false};   ///< 是否超出转向限幅
};

/// 线性 MPC(稠密二次规划)
///
/// 目标:  min  0.5·UᵀHU + fᵀU
///        s.t. 由线性化动力学与参考曲率滚动预测
/// 当前版本无约束求解(闭式);约束版用 BoxQp(见下)。
class LateralMpc {
  public:
    explicit LateralMpc(const MpcParams& params = MpcParams{}) : params_(params) {}

    /// @param s      当前车辆状态(s.v 作线性化工作点)
    /// @param x      当前路径相对误差状态
    /// @param kappa  预测时域内的参考曲率(长度 ≥ horizon 使用前 horizon 个)
    /// @param p      车辆参数
    /// @param info   诊断输出(可为 nullptr)
    ControlCmd Compute(const VehicleState& s, const MpcState& x,
                       const std::vector<double>& kappa, const VehicleParams& p,
                       MpcInfo* info = nullptr);

    void SetParams(const MpcParams& p) { params_ = p; }
    const MpcParams& params() const { return params_; }

  private:
    MpcParams params_;
};

// ---- 暴露给测试的底层工具 ----

/// 连续时间线性化:A(4x4)、B(4)、d0(4,κ=1 对应的扰动列)
void MpcLinearize(const VehicleParams& p, double vx, double A[4][4], double B[4],
                  double d0[4]);

/// 对称正定 Cholesky 求解 H·u = b(H 会被就地覆盖为下三角因子)
bool CholeskySolve(std::vector<std::vector<double>>& H, std::vector<double>& b,
                   std::vector<double>& u);

/// box 约束 QP:min 0.5uᵀHu + fᵀu,s.t. lb ≤ u ≤ ub(ADMM)
bool BoxQp(const std::vector<std::vector<double>>& H, const std::vector<double>& f,
           const std::vector<double>& lb, const std::vector<double>& ub,
           std::vector<double>& u, int max_iter = 20000, double tol = 1e-10);

}  // namespace adc
