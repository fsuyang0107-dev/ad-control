#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "adc/metrics.hpp"
#include "adc/sim_loop.hpp"
#include "adc/trajectory.hpp"
#include "adc/types.hpp"
#include "adc/vehicle_model.hpp"

namespace {

double TravelTime(const adc::Trajectory& t) {
    double sum = 0.0;
    for (std::size_t i = 1; i < t.size(); ++i) {
        const double ds = std::hypot(t[i].x - t[i - 1].x, t[i].y - t[i - 1].y);
        sum += ds / std::max(0.5 * (t[i].v_ref + t[i - 1].v_ref), 0.5);
    }
    return sum;
}

adc::Trajectory MakeSpeedStep(double v1, double v2, double switch_x) {
    adc::Trajectory traj;
    for (int i = 0; i < 900; ++i) {
        adc::TrajPoint tp;
        tp.x = 0.1 * i;
        tp.v_ref = (tp.x < switch_x) ? v1 : v2;
        traj.push_back(tp);
    }
    return traj;
}

}  // namespace

// ---------------- 执行器 ----------------

TEST(Actuator, FirstOrderLagReachesTarget) {
    adc::VehicleParams p;
    adc::VehicleModel m(p);
    adc::VehicleState s;
    adc::ControlCmd u;
    u.accel = 3.0;
    for (int k = 0; k < 150; ++k) m.StepLongitudinal(s, u, 0.01);  // 10 个 tau
    EXPECT_NEAR(s.accel, 3.0, 1e-3);
}

TEST(Actuator, OneTimeConstantReaches63Percent) {
    adc::VehicleParams p;
    adc::VehicleModel m(p);
    adc::VehicleState s;
    adc::ControlCmd u;
    u.accel = 3.0;
    m.StepLongitudinal(s, u, p.actuator_tau);
    EXPECT_NEAR(s.accel, 3.0 * (1.0 - std::exp(-1.0)), 1e-9);
}

TEST(Actuator, ClampsCommandToMaxAccel) {
    adc::VehicleParams p;
    adc::VehicleModel m(p);
    adc::VehicleState s;
    adc::ControlCmd u;
    u.accel = 100.0;
    for (int k = 0; k < 200; ++k) m.StepLongitudinal(s, u, 0.01);
    EXPECT_NEAR(s.accel, p.max_accel, 1e-3);
}

TEST(Actuator, ClampsCommandToMaxDecel) {
    adc::VehicleParams p;
    adc::VehicleModel m(p);
    adc::VehicleState s;
    s.v = 10.0;
    adc::ControlCmd u;
    u.accel = -100.0;
    m.StepLongitudinal(s, u, 0.01);
    EXPECT_GE(s.accel, p.max_decel);
}

TEST(Actuator, SpeedNeverBecomesNegative) {
    adc::VehicleParams p;
    adc::VehicleModel m(p);
    adc::VehicleState s;
    s.v = 1.0;
    adc::ControlCmd u;
    u.accel = -5.0;
    for (int k = 0; k < 200; ++k) {
        m.StepLongitudinal(s, u, 0.01);
        EXPECT_GE(s.v, 0.0);
    }
}

TEST(Actuator, SmallerTauRespondsFaster) {
    adc::VehicleParams fast; fast.actuator_tau = 0.05;
    adc::VehicleParams slow; slow.actuator_tau = 0.5;
    adc::VehicleModel mf(fast), ms(slow);
    adc::VehicleState sf, ss;
    adc::ControlCmd u;
    u.accel = 3.0;
    for (int k = 0; k < 20; ++k) {
        mf.StepLongitudinal(sf, u, 0.01);
        ms.StepLongitudinal(ss, u, 0.01);
    }
    EXPECT_GT(sf.accel, ss.accel);
}

// ---------------- 纵向 PI ----------------

TEST(LongPid, ZeroErrorZeroFeedforwardGivesZero) {
    adc::LongCtrlParams p;
    adc::LongitudinalPid pid(p, -5.0, 3.0);
    pid.Reset();
    EXPECT_NEAR(pid.Compute(10.0, 10.0, 0.0, 0.01), 0.0, 1e-12);
}

TEST(LongPid, PositiveErrorAccelerates) {
    adc::LongCtrlParams p;
    adc::LongitudinalPid pid(p, -5.0, 3.0);
    pid.Reset();
    EXPECT_GT(pid.Compute(12.0, 10.0, 0.0, 0.01), 0.0);
}

TEST(LongPid, NegativeErrorDecelerates) {
    adc::LongCtrlParams p;
    adc::LongitudinalPid pid(p, -5.0, 3.0);
    pid.Reset();
    EXPECT_LT(pid.Compute(8.0, 10.0, 0.0, 0.01), 0.0);
}

TEST(LongPid, FeedForwardPassesThroughWithZeroError) {
    adc::LongCtrlParams p;
    adc::LongitudinalPid pid(p, -5.0, 3.0);
    pid.Reset();
    EXPECT_NEAR(pid.Compute(10.0, 10.0, 1.5, 0.01), 1.5, 1e-12);
}

TEST(LongPid, OutputRespectsLimits) {
    adc::LongCtrlParams p;
    adc::LongitudinalPid pid(p, -2.0, 2.0);
    pid.Reset();
    for (int k = 0; k < 50; ++k) {
        const double u = pid.Compute(100.0, 0.0, 0.0, 0.01);
        EXPECT_LE(u, 2.0 + 1e-12);
        EXPECT_GE(u, -2.0 - 1e-12);
    }
}

TEST(LongPid, AntiWindupKeepsIntegralBounded) {
    adc::LongCtrlParams p;
    p.integral_limit = 0.5;
    adc::LongitudinalPid pid(p, -2.0, 2.0);
    pid.Reset();
    for (int k = 0; k < 2000; ++k) pid.Compute(50.0, 0.0, 0.0, 0.01);
    EXPECT_LE(std::abs(pid.integral()), 0.5 + 1e-12);
}

TEST(LongPid, ResetClearsIntegral) {
    adc::LongCtrlParams p;
    adc::LongitudinalPid pid(p, -5.0, 5.0);
    pid.Reset();
    for (int k = 0; k < 100; ++k) pid.Compute(10.2, 10.0, 0.0, 0.01);
    EXPECT_GT(pid.integral(), 0.0);
    pid.Reset();
    EXPECT_DOUBLE_EQ(pid.integral(), 0.0);
}

TEST(LongPid, AntiWindupFreezesIntegralWhenSaturated) {
    adc::LongCtrlParams p;
    adc::LongitudinalPid pid(p, -3.0, 3.0);
    pid.Reset();
    for (int k = 0; k < 100; ++k) pid.Compute(100.0, 0.0, 0.0, 0.01);
    EXPECT_LT(std::abs(pid.integral()), 1e-9)
        << "饱和且误差加剧饱和时,积分应冻结";
}

// ---------------- 速度剖面 ----------------

TEST(SpeedLimit, NeverExceedsVmax) {
    auto traj = adc::MakeDoubleLaneChange(60.0, 3.5, 0.1, 15.0);
    adc::ApplySpeedLimit(traj, 12.0, 3.0, 4.0, 2.0);
    ASSERT_FALSE(traj.empty());
    for (const auto& p : traj.points()) {
        EXPECT_LE(p.v_ref, 12.0 + 1e-9);
        EXPECT_GE(p.v_ref, 0.0);
    }
}

TEST(SpeedLimit, SlowsDownWhereCurvatureIsHigh) {
    auto traj = adc::MakeDoubleLaneChange(60.0, 3.5, 0.1, 15.0);
    std::size_t kmax_i = 0;
    double kmax = 0.0;
    for (std::size_t i = 0; i < traj.size(); ++i) {
        if (std::abs(traj[i].curvature) > kmax) {
            kmax = std::abs(traj[i].curvature);
            kmax_i = i;
        }
    }
    adc::ApplySpeedLimit(traj, 15.0, 3.0, 4.0, 2.0);
    EXPECT_LT(traj[kmax_i].v_ref, 15.0 - 0.5);
    EXPECT_NEAR(traj[kmax_i].v_ref, std::sqrt(3.0 / kmax), 0.5);
}

TEST(SpeedLimit, DecelerationIsFeasible) {
    auto traj = adc::MakeDoubleLaneChange(60.0, 3.5, 0.1, 15.0);
    const double a_dec = 4.0;
    adc::ApplySpeedLimit(traj, 15.0, 3.0, a_dec, 2.0);
    const auto& pts = traj.points();
    for (std::size_t i = 1; i < pts.size(); ++i) {
        const double ds = std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
        if (ds <= 0.0) continue;
        const double need = (pts[i - 1].v_ref * pts[i - 1].v_ref -
                             pts[i].v_ref * pts[i].v_ref) / (2.0 * ds);
        EXPECT_LE(need, a_dec + 1e-6) << "index " << i;
    }
}

TEST(SpeedLimit, AccelerationIsFeasible) {
    auto traj = adc::MakeDoubleLaneChange(60.0, 3.5, 0.1, 15.0);
    const double a_acc = 2.0;
    adc::ApplySpeedLimit(traj, 15.0, 3.0, 4.0, a_acc);
    const auto& pts = traj.points();
    for (std::size_t i = 1; i < pts.size(); ++i) {
        const double ds = std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
        if (ds <= 0.0) continue;
        const double need = (pts[i].v_ref * pts[i].v_ref -
                             pts[i - 1].v_ref * pts[i - 1].v_ref) / (2.0 * ds);
        EXPECT_LE(need, a_acc + 1e-6) << "index " << i;
    }
}

TEST(SpeedLimit, EmptyTrajectoryIsSafe) {
    adc::Trajectory t;
    adc::ApplySpeedLimit(t, 15.0, 3.0, 4.0, 2.0);
    EXPECT_TRUE(t.empty());
}

// ---------------- 闭环纵向 ----------------

TEST(Longitudinal, SpeedStepReachesTargetWithSmallError) {
    const auto traj = MakeSpeedStep(15.0, 20.0, 30.0);
    adc::SimConfig cfg;
    cfg.dt = 0.01;
    cfg.duration = 7.0;
    adc::SimLoop loop(cfg);
    const auto recs = loop.Run(traj);
    ASSERT_GT(recs.size(), 100u);
    double ss = 0.0;
    for (std::size_t i = recs.size() * 9 / 10; i < recs.size(); ++i)
        ss = std::max(ss, std::abs(recs[i].state.v - recs[i].v_ref));
    EXPECT_LT(ss, 0.05);
    EXPECT_NEAR(recs.back().state.v, 20.0, 0.05);
}

TEST(Longitudinal, SpeedStepHasSmallOvershoot) {
    const auto traj = MakeSpeedStep(15.0, 20.0, 30.0);
    adc::SimConfig cfg;
    cfg.dt = 0.01;
    cfg.duration = 7.0;
    adc::SimLoop loop(cfg);
    const auto recs = loop.Run(traj);
    double max_v = 0.0;
    for (const auto& r : recs) max_v = std::max(max_v, r.state.v);
    EXPECT_LT(max_v, 20.0 * 1.02);
}

TEST(Longitudinal, TracksVaryingProfileOnDlc) {
    auto traj = adc::MakeDoubleLaneChange(60.0, 3.5, 0.1, 15.0);
    adc::ApplySpeedLimit(traj, 15.0, 3.0, 4.0, 2.0);
    adc::SimConfig cfg;
    cfg.dt = 0.01;
    cfg.duration = TravelTime(traj);
    cfg.init_y_offset = 0.5;
    adc::SimLoop loop(cfg);
    const auto recs = loop.Run(traj);
    const auto m = adc::ComputeMetrics(recs);
    EXPECT_LT(m.rmse_v, 0.60);
    EXPECT_LT(m.rmse_y, 0.20);
}

TEST(Longitudinal, ConstantProfileUnchanged) {
    auto traj = adc::MakeDoubleLaneChange(60.0, 3.5, 0.1, 15.0);
    adc::SimConfig cfg;
    cfg.dt = 0.01;
    cfg.duration = TravelTime(traj);
    cfg.ctrl_mode = adc::CtrlMode::kPurePursuit;
    cfg.init_y_offset = 0.5;
    adc::SimLoop loop(cfg);
    const auto recs = loop.Run(traj);
    const auto m = adc::ComputeMetrics(recs);
    EXPECT_LT(m.rmse_v, 1e-9);
    EXPECT_NEAR(m.rmse_y, 0.103, 0.02);
}

TEST(Longitudinal, AllValuesFiniteWithSpeedProfile) {
    auto traj = adc::MakeSine(50.0, 2.0, 2.0, 0.1, 15.0);
    adc::ApplySpeedLimit(traj, 15.0, 3.0, 4.0, 2.0);
    adc::SimConfig cfg;
    cfg.dt = 0.01;
    cfg.duration = TravelTime(traj);
    adc::SimLoop loop(cfg);
    const auto recs = loop.Run(traj);
    ASSERT_GT(recs.size(), 10u);
    for (const auto& r : recs) {
        EXPECT_TRUE(std::isfinite(r.state.v));
        EXPECT_TRUE(std::isfinite(r.state.accel));
        EXPECT_TRUE(std::isfinite(r.e_y));
    }
}
