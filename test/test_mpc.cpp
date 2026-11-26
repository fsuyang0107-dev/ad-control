#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "adc/metrics.hpp"
#include "adc/mpc.hpp"
#include "adc/sim_loop.hpp"
#include "adc/trajectory.hpp"
#include "adc/types.hpp"

namespace {

struct Cl { double rmse, ss; };

Cl RunDlc(adc::CtrlMode mode, adc::ModelKind mk, double v, double init_y,
          const adc::MpcParams& mp = {}) {
    auto traj = adc::MakeDoubleLaneChange(60.0, 3.5, 0.1, v);
    adc::ApplySpeedLimit(traj, v, 3.0, 4.0, 2.0);
    double tt = 0.0;
    for (std::size_t i = 1; i < traj.size(); ++i) {
        const double ds = std::hypot(traj[i].x - traj[i-1].x, traj[i].y - traj[i-1].y);
        tt += ds / std::max(0.5 * (traj[i].v_ref + traj[i-1].v_ref), 0.5);
    }
    adc::SimConfig cfg;
    cfg.dt = 0.01;
    cfg.duration = tt;
    cfg.ctrl_mode = mode;
    cfg.model = mk;
    cfg.init_y_offset = init_y;
    cfg.mpc = mp;
    adc::SimLoop loop(cfg);
    const auto recs = loop.Run(traj);
    const auto m = adc::ComputeMetrics(recs);
    double ss = 0.0;
    for (std::size_t i = recs.size() * 8 / 10; i < recs.size(); ++i)
        ss = std::max(ss, std::abs(recs[i].e_y));
    return {m.rmse_y, ss};
}

}  // namespace

// ============================================================
// 一、线性化:与解析式一致
// ============================================================

TEST(Mpc, LinearizationMatchesAnalyticAB) {
    adc::VehicleParams p;
    const double v = 12.0;
    double A[4][4], B[4], d0[4];
    adc::MpcLinearize(p, v, A, B, d0);

    const double a11 = -(p.Cf + p.Cr) / (p.mass * v);
    const double a12 = -v - (p.lf * p.Cf - p.lr * p.Cr) / (p.mass * v);
    const double a21 = -(p.lf * p.Cf - p.lr * p.Cr) / (p.Iz * v);
    const double a22 = -(p.lf * p.lf * p.Cf + p.lr * p.lr * p.Cr) / (p.Iz * v);

    EXPECT_NEAR(A[2][2], a11, 1e-12);
    EXPECT_NEAR(A[2][3], a12, 1e-12);
    EXPECT_NEAR(A[3][2], a21, 1e-12);
    EXPECT_NEAR(A[3][3], a22, 1e-12);
    EXPECT_NEAR(B[2], p.Cf / p.mass, 1e-12);
    EXPECT_NEAR(B[3], p.lf * p.Cf / p.Iz, 1e-12);
    EXPECT_NEAR(A[0][1], v, 1e-12);   // ė_y 依赖 e_yaw
    EXPECT_NEAR(A[1][3], 1.0, 1e-12); // ė_yaw 依赖 r
    EXPECT_NEAR(d0[1], -v, 1e-12);    // 曲率扰动系数
}

TEST(Mpc, LateralErrorUsesRearAxleVelocityNotCgVelocity) {
    // 回归测试:e_y 定义在后轴,后轴横向速度 = v_y − l_r·r,
    // 必须出现在 ė_y 里。漏掉这一项会使 MPC 内部模型偏差
    // l_r·r ≈ 1.5×0.45 = 0.675 m/s,足以让闭环振荡。
    adc::VehicleParams p;
    double A[4][4], B[4], d0[4];
    adc::MpcLinearize(p, 12.0, A, B, d0);

    EXPECT_NEAR(A[0][2], 1.0, 1e-12) << "v_y 的系数应为 1";
    EXPECT_NEAR(A[0][3], -p.lr, 1e-12)
        << "r 的系数应为 −l_r(后轴横向速度 = v_y − l_r·r)";
}

TEST(Mpc, LowSpeedUsesGuardedOperatingPoint) {
    // v→0 时 1/v 会病态,线性化内部应把工作点抬到下界
    adc::VehicleParams p;
    double A1[4][4], B1[4], d1[4];
    double A2[4][4], B2[4], d2[4];
    adc::MpcLinearize(p, 0.5, A1, B1, d1);
    adc::MpcLinearize(p, 2.0, A2, B2, d2);
    for (int i = 0; i < 4; ++i) {
        EXPECT_TRUE(std::isfinite(A1[i][i]));
        EXPECT_NEAR(A1[i][i], A2[i][i], 1e-12) << "低速应被钳到下界";
    }
}

// ============================================================
// 二、QP 求解器
// ============================================================

TEST(Mpc, CholeskySolvesKnownSystem) {
    // H = [[4,1],[1,3]],b = [1,2] → u = [1/11, 7/11]
    std::vector<std::vector<double>> H = {{4.0, 1.0}, {1.0, 3.0}};
    std::vector<double> b = {1.0, 2.0}, u;
    ASSERT_TRUE(adc::CholeskySolve(H, b, u));
    EXPECT_NEAR(u[0], 1.0 / 11.0, 1e-12);
    EXPECT_NEAR(u[1], 7.0 / 11.0, 1e-12);
}

TEST(Mpc, CholeskyRejectsNonPositiveDefinite) {
    std::vector<std::vector<double>> H = {{1.0, 2.0}, {2.0, 1.0}};
    std::vector<double> b = {1.0, 1.0}, u;
    EXPECT_FALSE(adc::CholeskySolve(H, b, u));
}

TEST(Mpc, BoxQpMatchesUnconstrainedWhenBoundsInactive) {
    // 关键一致性测试:约束不起作用时,ADMM 应给出与闭式解相同的答案
    const int n = 4;
    std::vector<std::vector<double>> H(n, std::vector<double>(n, 0.0));
    for (int i = 0; i < n; ++i) {
        H[i][i] = 4.0 + i;
        if (i + 1 < n) {
            H[i][i + 1] = 0.5;
            H[i + 1][i] = 0.5;
        }
    }
    std::vector<double> f = {1.0, -2.0, 0.5, 3.0};

    std::vector<double> negf = f;
    for (auto& x : negf) x = -x;
    std::vector<std::vector<double>> Hc = H;
    std::vector<double> u_free;
    ASSERT_TRUE(adc::CholeskySolve(Hc, negf, u_free));

    std::vector<double> u_box;
    const std::vector<double> lb(n, -1e3), ub(n, 1e3);
    adc::BoxQp(H, f, lb, ub, u_box, 200000, 1e-12);

    for (int i = 0; i < n; ++i)
        EXPECT_NEAR(u_box[i], u_free[i], 1e-5) << "i=" << i;
}

TEST(Mpc, BoxQpRespectsActiveBounds) {
    // 一维问题:min 0.5·u² − 10u 无约束解 u=10,加 |u|≤1 后应贴在 u=1
    std::vector<std::vector<double>> H = {{1.0}};
    std::vector<double> f = {-10.0};
    std::vector<double> lb = {-1.0}, ub = {1.0};
    std::vector<double> u;
    adc::BoxQp(H, f, lb, ub, u, 200000, 1e-12);
    EXPECT_NEAR(u[0], 1.0, 1e-4);
}

// ============================================================
// 三、闭环性能:MPC 应显著优于几何控制器
// ============================================================

TEST(Mpc, BeatsStanleyOnDynamicModel) {
    adc::MpcParams mp;
    mp.r_steer = 2.0;
    mp.q_ey = 20.0;

    const auto stan = RunDlc(adc::CtrlMode::kStanley, adc::ModelKind::kDynamic, 12.0, 0.5);
    const auto mpc =
        RunDlc(adc::CtrlMode::kMpc, adc::ModelKind::kDynamic, 12.0, 0.5, mp);

    EXPECT_LT(mpc.ss, stan.ss * 0.2) << "MPC 稳态误差应显著优于 Stanley";
    EXPECT_LT(mpc.rmse, stan.rmse);
    EXPECT_LT(mpc.ss, 0.05) << "MPC 稳态误差应在 5 cm 内";
}

TEST(Mpc, ComparedWithModelFreeControllersOnDynamicModel) {
    // 模型类控制器在动力学模型上应全面占优
    adc::MpcParams mp;
    mp.r_steer = 2.0;
    mp.q_ey = 20.0;

    const auto pp =
        RunDlc(adc::CtrlMode::kPurePursuit, adc::ModelKind::kDynamic, 12.0, 0.5);
    const auto mpc =
        RunDlc(adc::CtrlMode::kMpc, adc::ModelKind::kDynamic, 12.0, 0.5, mp);
    EXPECT_LT(mpc.rmse, pp.rmse);
    EXPECT_LT(mpc.ss, pp.ss);
}

TEST(Mpc, StraightLineWithZeroErrorStaysStraight) {
    auto traj = adc::MakeStraight(60.0, 0.1, 12.0);
    for (auto& p : traj.points_mutable()) p.v_ref = 12.0;

    adc::SimConfig cfg;
    cfg.dt = 0.01;
    cfg.duration = 4.0;
    cfg.ctrl_mode = adc::CtrlMode::kMpc;
    cfg.model = adc::ModelKind::kDynamic;
    adc::SimLoop loop(cfg);
    const auto recs = loop.Run(traj);

    double max_abs_y = 0.0, max_steer = 0.0;
    for (const auto& r : recs) {
        max_abs_y = std::max(max_abs_y, std::abs(r.e_y));
        max_steer = std::max(max_steer, std::abs(r.cmd.steer));
    }
    EXPECT_LT(max_abs_y, 1e-6) << "零误差直线不应产生横向偏移";
    EXPECT_LT(max_steer, 1e-6) << "零误差直线不应打方向";
}

TEST(Mpc, ArcTrackingKeepsLateralErrorSmall) {
    constexpr double kPi = 3.14159265358979323846;
    auto traj = adc::MakeArc(25.0, kPi, 0.1, 12.0);
    for (auto& p : traj.points_mutable()) p.v_ref = 12.0;

    adc::SimConfig cfg;
    cfg.dt = 0.01;
    cfg.duration = 5.0;
    cfg.ctrl_mode = adc::CtrlMode::kMpc;
    cfg.model = adc::ModelKind::kDynamic;
    adc::SimLoop loop(cfg);
    const auto recs = loop.Run(traj);

    // 注:圆弧起点曲率非零,但初始横摆角速度被置为 0(项目约定),
    // 所以前 ~1 s 存在"建立侧偏/横摆"的瞬态。这里只考核瞬态之后。
    double max_abs_y = 0.0;
    for (std::size_t i = recs.size() / 2; i < recs.size(); ++i)
        max_abs_y = std::max(max_abs_y, std::abs(recs[i].e_y));
    EXPECT_LT(max_abs_y, 0.10) << "圆弧进入稳态后横向误差应小于 10 cm";

    double all_max = 0.0;
    for (const auto& r : recs) all_max = std::max(all_max, std::abs(r.e_y));
    EXPECT_LT(all_max, 1.0) << "含瞬态也不应发散";
}

TEST(Mpc, AllStatesFiniteUnderExtremeInitialError) {
    auto traj = adc::MakeDoubleLaneChange(60.0, 3.5, 0.1, 12.0);
    for (auto& p : traj.points_mutable()) p.v_ref = 12.0;

    adc::SimConfig cfg;
    cfg.dt = 0.01;
    cfg.duration = 5.0;
    cfg.ctrl_mode = adc::CtrlMode::kMpc;
    cfg.model = adc::ModelKind::kDynamic;
    cfg.init_y_offset = 5.0;   // 极端初始偏差
    adc::SimLoop loop(cfg);
    const auto recs = loop.Run(traj);
    for (const auto& r : recs) {
        ASSERT_TRUE(std::isfinite(r.cmd.steer));
        ASSERT_TRUE(std::isfinite(r.state.x));
        ASSERT_TRUE(std::isfinite(r.state.yaw));
        ASSERT_LE(std::abs(r.cmd.steer), cfg.vehicle.max_steer + 1e-9);
    }
}
