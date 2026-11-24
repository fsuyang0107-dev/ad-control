#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>

#include "adc/metrics.hpp"
#include "adc/sim_loop.hpp"
#include "adc/trajectory.hpp"
#include "adc/types.hpp"
#include "adc/vehicle_model.hpp"

namespace {

/// 稳态圆周:恒定转角与初速,返回最后 10 s 的平均横摆角速度与侧向速度
void SteadyCircle(const adc::VehicleParams& p, double v, double delta, double& r_out,
                  double& vy_out) {
    adc::VehicleModel m(p);
    adc::VehicleState s;
    s.v = v;
    adc::ControlCmd u;
    u.steer = delta;

    const double dt = 0.001;
    const int steps = 20000;
    double rsum = 0.0, vysum = 0.0;
    int cnt = 0;
    for (int k = 0; k < steps; ++k) {
        m.StepDynamic(s, u, dt);
        if (k > steps / 2) {
            rsum += s.yaw_rate;
            vysum += s.vy;
            ++cnt;
        }
    }
    r_out = rsum / cnt;
    vy_out = vysum / cnt;
}

double UndersteerGradient(const adc::VehicleParams& p) {
    const double L = p.lf + p.lr;
    return p.mass * (p.lr * p.Cr - p.lf * p.Cf) / (L * p.Cf * p.Cr);
}

struct Cl { double rmse, ss; };

Cl RunDlc(double v, adc::ModelKind mk) {
    auto traj = adc::MakeDoubleLaneChange(60.0, 3.5, 0.1, v);
    for (auto& p : traj.points_mutable()) p.v_ref = v;
    adc::SimConfig cfg;
    cfg.dt = 0.005;
    cfg.duration = traj.length() / v;
    cfg.ctrl_mode = adc::CtrlMode::kStanley;
    cfg.model = mk;
    cfg.init_y_offset = 0.5;
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
// 一、稳态转向特性:与不足转向理论对比
// ============================================================

TEST(Dynamic, UndersteerGradientIsPositiveForDefaultParams) {
    adc::VehicleParams p;
    EXPECT_GT(UndersteerGradient(p), 0.0) << "默认参数应为不足转向";
}

TEST(Dynamic, SteadyStateYawRateMatchesTheory) {
    adc::VehicleParams p;
    const double L = p.lf + p.lr;
    const double Kus = UndersteerGradient(p);
    const double delta = 0.01;

    for (double v : {5.0, 10.0, 15.0, 20.0}) {
        double r = 0.0, vy = 0.0;
        SteadyCircle(p, v, delta, r, vy);
        const double r_theory = v * delta / (L + Kus * v * v);
        EXPECT_NEAR(r, r_theory, std::abs(r_theory) * 0.01)
            << "v=" << v << " 稳态横摆角速度偏离理论超过 1%";
    }
}

TEST(Dynamic, NeutralSteerReproducesKinematicYawRate) {
    // 令 lf*Cf == lr*Cr -> K_us = 0,动力学应退化为运动学
    adc::VehicleParams p;
    p.Cr = p.lf * p.Cf / p.lr;
    EXPECT_NEAR(UndersteerGradient(p), 0.0, 1e-12);

    const double v = 15.0, delta = 0.01;
    double r = 0.0, vy = 0.0;
    SteadyCircle(p, v, delta, r, vy);
    const double r_kin = v / (p.lf + p.lr) * std::tan(delta);
    EXPECT_NEAR(r, r_kin, r_kin * 0.01);
}

TEST(Dynamic, HigherSpeedMeansMoreUndersteer) {
    // 同一转角下,车速越高,动力学横摆角速度相对运动学的偏差越大
    adc::VehicleParams p;
    const double L = p.lf + p.lr;
    const double delta = 0.01;

    double prev_ratio = 1.0;
    for (double v : {5.0, 10.0, 15.0, 20.0}) {
        double r = 0.0, vy = 0.0;
        SteadyCircle(p, v, delta, r, vy);
        const double r_kin = v / L * std::tan(delta);
        const double ratio = r / r_kin;
        EXPECT_LT(ratio, prev_ratio) << "v=" << v << " 处比值应随速度下降";
        prev_ratio = ratio;
    }
}

// ============================================================
// 二、基本物理行为
// ============================================================

TEST(Dynamic, ZeroSpeedProducesNoMotion) {
    adc::VehicleParams p;
    adc::VehicleModel m(p);
    adc::VehicleState s;
    s.v = 0.0;
    s.x = 1.0;
    s.y = 2.0;
    adc::ControlCmd u;
    u.steer = 0.2;
    for (int k = 0; k < 1000; ++k) m.StepDynamic(s, u, 0.001);
    EXPECT_NEAR(s.x, 1.0, 1e-6);
    EXPECT_NEAR(s.y, 2.0, 1e-6);
    EXPECT_NEAR(s.yaw, 0.0, 1e-9);
}

TEST(Dynamic, StraightLineWithZeroSteerKeepsZeroLateralVelocity) {
    adc::VehicleParams p;
    adc::VehicleModel m(p);
    adc::VehicleState s;
    s.v = 15.0;
    adc::ControlCmd u;
    u.steer = 0.0;
    for (int k = 0; k < 5000; ++k) m.StepDynamic(s, u, 0.001);
    EXPECT_NEAR(s.vy, 0.0, 1e-9);
    EXPECT_NEAR(s.yaw_rate, 0.0, 1e-9);
    EXPECT_NEAR(s.y, 0.0, 1e-6);
}

TEST(Dynamic, FrictionSaturationPreventsDivergence) {
    adc::VehicleParams p;
    adc::VehicleModel m(p);
    adc::VehicleState s;
    s.v = 20.0;
    adc::ControlCmd u;
    u.steer = p.max_steer;   // 打满
    for (int k = 0; k < 5000; ++k) {
        m.StepDynamic(s, u, 0.001);
        ASSERT_TRUE(std::isfinite(s.yaw_rate));
        ASSERT_TRUE(std::isfinite(s.vy));
        ASSERT_TRUE(std::isfinite(s.x));
        ASSERT_LT(std::abs(s.yaw_rate), 5.0) << "横摆角速度不应发散";
    }
}

TEST(Dynamic, AllStatesFiniteUnderExtremeInput) {
    adc::VehicleParams p;
    adc::VehicleModel m(p);
    adc::VehicleState s;
    s.v = 30.0;
    adc::ControlCmd u;
    u.steer = -p.max_steer;
    u.accel = p.max_decel;
    for (int k = 0; k < 3000; ++k) {
        m.StepDynamic(s, u, 0.005);
        ASSERT_TRUE(std::isfinite(s.v));
        ASSERT_TRUE(std::isfinite(s.vy));
        ASSERT_TRUE(std::isfinite(s.yaw_rate));
        ASSERT_TRUE(std::isfinite(s.x));
        ASSERT_TRUE(std::isfinite(s.y));
        ASSERT_TRUE(std::isfinite(s.yaw));
        ASSERT_GE(s.v, 0.0);
    }
}

TEST(Dynamic, LowSpeedApproachesKinematicModel) {
    // 低速时所需侧向力小 -> 侧偏角小 -> 动力学接近运动学
    const double v = 5.0;
    const auto kin = RunDlc(v, adc::ModelKind::kKinematic);
    const auto dyn = RunDlc(v, adc::ModelKind::kDynamic);
    EXPECT_LT(std::abs(dyn.rmse - kin.rmse), 0.05)
        << "低速下两模型 RMSE 应接近: kin=" << kin.rmse << " dyn=" << dyn.rmse;
}

// ============================================================
// 三、核心结论:几何控制器在动力学模型上随速度退化
// ============================================================

TEST(Dynamic, KinematicTunedControllerDegradesWithSpeed) {
    const auto slow = RunDlc(5.0, adc::ModelKind::kDynamic);
    const auto fast = RunDlc(15.0, adc::ModelKind::kDynamic);

    // 低速时 Stanley 仍能收敛到厘米级
    EXPECT_LT(slow.ss, 0.05) << "低速稳态误差应在 5 cm 内";

    // 高速时严重退化(这是 Day 9 的核心发现)
    EXPECT_GT(fast.ss, 0.20) << "高速下运动学整定的 Stanley 应显著退化";
    EXPECT_GT(fast.ss, slow.ss * 10.0) << "退化应随速度单调加剧";
}

TEST(Dynamic, DynamicModelIsStricterThanKinematic) {
    const double v = 12.0;
    const auto kin = RunDlc(v, adc::ModelKind::kKinematic);
    const auto dyn = RunDlc(v, adc::ModelKind::kDynamic);
    EXPECT_LT(kin.ss, 0.01) << "运动学模型下应精确跟踪";
    EXPECT_GT(dyn.ss, kin.ss * 10.0) << "动力学模型下同一控制器应明显更差";
}
