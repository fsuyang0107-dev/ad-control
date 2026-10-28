#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "adc/metrics.hpp"
#include "adc/sim_loop.hpp"
#include "adc/trajectory.hpp"
#include "adc/types.hpp"

namespace {
constexpr double kPi = 3.14159265358979323846;

adc::SimConfig MakeCfg(double dt, double duration) {
    adc::SimConfig c;
    c.dt = dt;
    c.duration = duration;
    return c;
}
}  // namespace

TEST(FindNearest, OnEmptyTrajectoryReturnsZero) {
    adc::Trajectory t;
    adc::VehicleState s;
    EXPECT_EQ(adc::FindNearest(t, s), 0u);
}

TEST(FindNearest, ReturnsClosestPoint) {
    const auto traj = adc::MakeStraight(10.0, 0.1, 5.0);
    adc::VehicleState s;
    s.x = 3.04;
    s.y = 0.0;
    const std::size_t i = adc::FindNearest(traj, s);
    EXPECT_NEAR(traj[i].x, 3.0, 0.11);
}

TEST(FindNearest, PicksPointMinimizingDistance) {
    const auto traj = adc::MakeStraight(10.0, 0.1, 5.0);
    adc::VehicleState s;
    s.x = 7.0;
    s.y = 0.5;
    const std::size_t i = adc::FindNearest(traj, s);
    const double best = std::hypot(traj[i].x - s.x, traj[i].y - s.y);
    for (std::size_t k = 0; k < traj.size(); ++k) {
        const double d = std::hypot(traj[k].x - s.x, traj[k].y - s.y);
        EXPECT_GE(d, best - 1e-12) << "index " << k;
    }
}

TEST(FeedForward, ZeroCurvatureGivesZeroSteer) {
    adc::VehicleParams p;
    adc::TrajPoint rp;
    rp.curvature = 0.0;
    EXPECT_NEAR(adc::FeedForward(rp, p).steer, 0.0, 1e-15);
}

TEST(FeedForward, SteerRecoversCurvature) {
    adc::VehicleParams p;
    adc::TrajPoint rp;
    rp.curvature = 0.02;
    const double steer = adc::FeedForward(rp, p).steer;
    EXPECT_NEAR(std::tan(steer) / p.wheelbase, 0.02, 1e-12);
}

TEST(FeedForward, SteerIsSaturated) {
    adc::VehicleParams p;
    adc::TrajPoint rp;
    rp.curvature = 10.0;
    const double steer = adc::FeedForward(rp, p).steer;
    EXPECT_LE(std::abs(steer), p.max_steer + 1e-12);
}

TEST(SignConvention, PositiveLateralErrorWhenVehicleIsLeft) {
    adc::SimConfig c = MakeCfg(0.02, 1.0);
    c.init_y_offset = 0.5;
    adc::SimLoop loop(c);
    const auto recs = loop.Run(adc::MakeStraight(20.0, 0.1, 5.0));
    ASSERT_GT(recs.size(), 10u);
    EXPECT_GT(recs[0].e_y, 0.4);
}

TEST(SignConvention, NegativeLateralErrorWhenVehicleIsRight) {
    adc::SimConfig c = MakeCfg(0.02, 1.0);
    c.init_y_offset = -0.5;
    adc::SimLoop loop(c);
    const auto recs = loop.Run(adc::MakeStraight(20.0, 0.1, 5.0));
    ASSERT_GT(recs.size(), 10u);
    EXPECT_LT(recs[0].e_y, -0.4);
}

TEST(SimLoop, RecordCountMatchesSteps) {
    adc::SimConfig c = MakeCfg(0.02, 2.0);
    adc::SimLoop loop(c);
    const auto recs = loop.Run(adc::MakeStraight(20.0, 0.1, 5.0));
    ASSERT_EQ(recs.size(), 101u);
    EXPECT_NEAR(recs.front().t, 0.0, 1e-12);
    EXPECT_NEAR(recs.back().t, 2.0, 1e-9);
}

TEST(SimLoop, EmptyTrajectoryProducesNoRecords) {
    adc::SimLoop loop(MakeCfg(0.02, 1.0));
    adc::Trajectory empty;
    EXPECT_TRUE(loop.Run(empty).empty());
}

TEST(SimLoop, ZeroDurationProducesNoRecords) {
    adc::SimLoop loop(MakeCfg(0.02, 0.0));
    EXPECT_TRUE(loop.Run(adc::MakeStraight(10.0, 0.1, 5.0)).empty());
}

TEST(SimLoop, PerfectInitialConditionTracksStraightExactly) {
    adc::SimConfig c = MakeCfg(0.02, 2.0);
    c.ctrl_mode = adc::CtrlMode::kFeedForward;
    adc::SimLoop loop(c);
    const auto recs = loop.Run(adc::MakeStraight(20.0, 0.1, 5.0));
    const auto m = adc::ComputeMetrics(recs);
    EXPECT_LT(m.rmse_y, 1e-9);
    EXPECT_LT(m.max_abs_y, 1e-9);
    EXPECT_LT(m.rmse_yaw, 1e-9);
}

TEST(SimLoop, FeedForwardCannotCorrectInitialOffset) {
    adc::SimConfig c = MakeCfg(0.02, 2.0);
    c.ctrl_mode = adc::CtrlMode::kFeedForward;
    c.init_y_offset = 0.5;
    adc::SimLoop loop(c);
    const auto recs = loop.Run(adc::MakeStraight(20.0, 0.1, 5.0));
    const auto m = adc::ComputeMetrics(recs);
    EXPECT_NEAR(m.rmse_y, 0.5, 0.02);
    EXPECT_NEAR(m.max_abs_y, 0.5, 0.02);
}

TEST(SimLoop, FeedForwardCannotCorrectInitialHeadingOffset) {
    adc::SimConfig c = MakeCfg(0.02, 2.0);
    c.ctrl_mode = adc::CtrlMode::kFeedForward;
    c.init_yaw_offset = 0.1;
    adc::SimLoop loop(c);
    const auto recs = loop.Run(adc::MakeStraight(20.0, 0.1, 5.0));
    const auto m = adc::ComputeMetrics(recs);
    EXPECT_NEAR(recs[0].e_yaw, 0.1, 1e-9);
    EXPECT_GT(m.max_abs_y, 0.5);
}

TEST(SimLoop, AllRecordedValuesAreFinite) {
    adc::SimConfig c = MakeCfg(0.02, 4.0);
    c.init_y_offset = 0.3;
    adc::SimLoop loop(c);
    const auto recs = loop.Run(adc::MakeDoubleLaneChange(60.0, 3.5, 0.1, 15.0));
    ASSERT_GT(recs.size(), 10u);
    for (const auto& r : recs) {
        EXPECT_TRUE(std::isfinite(r.state.x));
        EXPECT_TRUE(std::isfinite(r.state.y));
        EXPECT_TRUE(std::isfinite(r.state.yaw));
        EXPECT_TRUE(std::isfinite(r.e_y));
        EXPECT_TRUE(std::isfinite(r.e_yaw));
        EXPECT_TRUE(std::isfinite(r.cmd.steer));
    }
}

TEST(SimLoop, ReferenceIndexIsMonotonicOnStraight) {
    adc::SimConfig c = MakeCfg(0.02, 2.0);
    adc::SimLoop loop(c);
    const auto recs = loop.Run(adc::MakeStraight(30.0, 0.1, 5.0));
    ASSERT_GT(recs.size(), 10u);
    for (std::size_t i = 1; i < recs.size(); ++i) {
        EXPECT_GE(recs[i].ref_index, recs[i - 1].ref_index) << "index " << i;
    }
}
